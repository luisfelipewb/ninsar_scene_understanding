// Offline detection: read the bag directly, no `ros2 bag play`, no wall clock.
//
// TF is replayed into a BufferCore up front, so every pose lookup is exact and
// the run is deterministic - the same bag and parameters always produce the
// same detections file.
#include "ninsar_lidar_calib/corner_detector.hpp"
#include "ninsar_lidar_calib/detection.hpp"
#include "ninsar_lidar_calib/ground_plane.hpp"
#include "ninsar_lidar_calib/params.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>

#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace ninsar_lidar_calib
{
namespace
{

/// Recordings are <session>/bag/bag_N.mcap; accept either level, matching the
/// layout the bagfix tool uses.
std::string resolveBag(const std::string & path)
{
  namespace fs = std::filesystem;
  const fs::path given(path);
  if (fs::exists(given / "metadata.yaml")) {
    return given.string();
  }
  if (fs::exists(given / "bag" / "metadata.yaml")) {
    return (given / "bag").string();
  }
  return given.string();  // let rosbag2 produce the error
}

template <typename T>
T deserialize(const rosbag2_storage::SerializedBagMessage & message)
{
  T value;
  rclcpp::SerializedMessage serialized(*message.serialized_data);
  rclcpp::Serialization<T>().deserialize_message(&serialized, &value);
  return value;
}

/// Fill a BufferCore with every transform in the bag before detecting anything.
void loadTransforms(const std::string & uri, tf2::BufferCore & buffer, size_t & count)
{
  rosbag2_cpp::Reader reader;
  reader.open(uri);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const bool is_static = message->topic_name == "/tf_static";
    if (message->topic_name != "/tf" && !is_static) {
      continue;
    }
    const auto tf_message = deserialize<tf2_msgs::msg::TFMessage>(*message);
    for (const auto & transform : tf_message.transforms) {
      // "bagfix" is just the authority string tf2 records for these entries
      buffer.setTransform(transform, "bagfix", is_static);
      ++count;
    }
  }
}

}  // namespace
}  // namespace ninsar_lidar_calib

int main(int argc, char ** argv)
{
  using namespace ninsar_lidar_calib;

  rclcpp::init(argc, argv);

  // The node exists only to parse the same parameter file the live node uses,
  // so tuned values carry over untouched.
  auto node = std::make_shared<rclcpp::Node>("lidar_calib_offline");
  CalibParams params;
  params.declare(*node);
  node->declare_parameter("bag", std::string());
  params.read(*node);

  std::string bag = node->get_parameter("bag").as_string();
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--bag" && i + 1 < argc) {
      bag = argv[++i];
    }
  }

  if (bag.empty()) {
    std::cerr << "usage: detect_from_bag --bag <session or bag dir> "
                 "--ros-args --params-file <config.yaml>\n";
    rclcpp::shutdown();
    return 2;
  }

  if (const std::string error = params.validate(); !error.empty()) {
    std::cerr << "invalid configuration: " << error << '\n';
    rclcpp::shutdown();
    return 2;
  }

  const std::string uri = resolveBag(bag);
  std::cout << "bag        : " << uri << "\n"
            << "cloud topic: " << params.cloud_topic << "\n"
            << "detections : " << params.detections_path << "\n"
            << params.summary() << "\n\n";

  tf2::BufferCore tf_buffer(tf2::durationFromSec(1e9));
  size_t transforms = 0;
  try {
    loadTransforms(uri, tf_buffer, transforms);
  } catch (const std::exception & exception) {
    std::cerr << "cannot read " << uri << ": " << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "loaded " << transforms << " transforms from the bag\n";

  DetectionLog log;
  if (!log.open(params.detections_path)) {
    std::cerr << "cannot write " << params.detections_path << '\n';
    rclcpp::shutdown();
    return 1;
  }

  GroundPlaneDetector ground_detector;
  CornerDetector corner_detector;
  std::map<Reject, size_t> rejections;
  size_t scans = 0;
  size_t accepted = 0;

  rosbag2_cpp::Reader reader;
  reader.open(uri);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name != params.cloud_topic) {
      continue;
    }
    ++scans;

    const auto cloud_msg = deserialize<sensor_msgs::msg::PointCloud2>(*message);
    const std::string lidar_frame =
      params.lidar_frame.empty() ? cloud_msg.header.frame_id : params.lidar_frame;

    Detection detection;
    detection.stamp_ns = rclcpp::Time(cloud_msg.header.stamp).nanoseconds();

    try {
      detection.world_from_base = tf2::transformToEigen(tf_buffer.lookupTransform(
        params.world_frame, params.base_frame, tf2_ros::fromMsg(cloud_msg.header.stamp)));
    } catch (const tf2::TransformException &) {
      ++rejections[Reject::kNoPose];
      continue;
    }

    Eigen::Isometry3d base_from_lidar = params.initialExtrinsic();
    if (params.extrinsic_source == "tf_static") {
      try {
        base_from_lidar = tf2::transformToEigen(
          tf_buffer.lookupTransform(params.base_frame, lidar_frame, tf2::TimePointZero));
      } catch (const tf2::TransformException &) {
        // fall back to the configured guess, already assigned above
      }
    }

    Cloud cloud;
    pcl::fromROSMsg(cloud_msg, cloud);
    if (cloud.empty()) {
      ++rejections[Reject::kNoCloud];
      continue;
    }

    if (params.ground.enabled) {
      Reject reason = Reject::kNone;
      detection.ground =
        ground_detector.detect(cloud, params.ground, base_from_lidar, reason);
      if (detection.ground.valid) {
        ground_detector.accumulate(detection.ground);
      } else {
        ++rejections[reason];
      }
    }

    if (params.corner.enabled) {
      Reject reason = Reject::kNone;
      detection.corner = corner_detector.detect(
        cloud, params.corner, base_from_lidar, detection.world_from_base,
        detection.ground, reason);
      if (!detection.corner.valid) {
        ++rejections[reason];
      }
    }

    if (detection.usable()) {
      ++accepted;
      log.write(detection);
    }
  }
  log.close();

  std::cout << "\n" << scans << " scans, " << accepted << " detections written"
            << " (ground " << ground_detector.accepted()
            << ", corner " << corner_detector.accepted() << ")\n";
  for (const auto & [reason, count] : rejections) {
    std::cout << "  rejected " << count << "x: " << describe(reason) << '\n';
  }
  if (GroundObservation average; ground_detector.average(average)) {
    double roll = 0.0;
    double pitch = 0.0;
    double height = 0.0;
    groundToRollPitchHeight(average.normal, average.offset, roll, pitch, height);
    std::cout << "\nmean ground: roll " << roll * 180.0 / M_PI << " deg, pitch "
              << pitch * 180.0 / M_PI << " deg, height " << height << " m\n";
  }

  if (accepted == 0) {
    std::cerr << "\nno detections - check the topic name, the ROI and the "
                 "rejection reasons above\n";
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "\nnext: ros2 run ninsar_lidar_calib solve " << params.detections_path
            << '\n';
  rclcpp::shutdown();
  return 0;
}
