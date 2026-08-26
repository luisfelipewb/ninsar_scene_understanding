// Live calibration node: play a bag, watch the detections in RViz, and tune
// every guess with `ros2 param set` until scans start being accepted.
#include "ninsar_lidar_calib/corner_detector.hpp"
#include "ninsar_lidar_calib/detection.hpp"
#include "ninsar_lidar_calib/ground_plane.hpp"
#include "ninsar_lidar_calib/markers.hpp"
#include "ninsar_lidar_calib/params.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ninsar_lidar_calib
{

class CalibNode : public rclcpp::Node
{
public:
  CalibNode()
  : Node("lidar_calib_node")
  {
    params_.declare(*this);
    params_.read(*this);

    if (const std::string error = params_.validate(); !error.empty()) {
      RCLCPP_FATAL(get_logger(), "invalid configuration: %s", error.c_str());
      throw std::runtime_error(error);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).transient_local());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });

    // Live tuning: every guess takes effect on the next scan, and bad values
    // are refused with a reason instead of being accepted and failing later.
    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changed) {
        return onParameters(changed);
      });

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save_config",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        onSaveConfig(*response);
      });

    if (!log_.open(params_.detections_path)) {
      RCLCPP_ERROR(get_logger(), "cannot write detections to '%s'",
                   params_.detections_path.c_str());
    }

    RCLCPP_INFO(get_logger(), "subscribed to %s", params_.cloud_topic.c_str());
    RCLCPP_INFO(get_logger(), "writing detections to %s",
                params_.detections_path.c_str());
    RCLCPP_INFO(get_logger(), "%s", params_.summary().c_str());
  }

  ~CalibNode() override
  {
    log_.close();
    RCLCPP_INFO(get_logger(), "final state\n%s", params_.summary().c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & changed)
  {
    rcl_interfaces::msg::SetParametersResult result;

    // Validate against a copy so a rejected set never half-applies.
    CalibParams candidate = params_;
    for (const auto & parameter : changed) {
      candidate.applyOverride(parameter);
    }
    const std::string error = candidate.validate();
    result.successful = error.empty();
    result.reason = error.empty() ? "ok" : error;

    if (result.successful) {
      params_ = candidate;
      // A retuned ROI invalidates the axis reference built from earlier scans.
      corner_detector_.reset();
      world_corners_.clear();
      RCLCPP_INFO(get_logger(), "parameters updated\n%s", params_.summary().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "rejected parameter change: %s", error.c_str());
    }
    return result;
  }

  void onSaveConfig(std_srvs::srv::Trigger::Response & response)
  {
    const std::string path = params_.detections_path + ".params.yaml";
    std::ofstream file(path);
    if (!file) {
      response.success = false;
      response.message = "cannot write " + path;
      return;
    }
    file << params_.toYaml(std::string(get_name())) << '\n';
    response.success = true;
    response.message = "wrote " + path;
    RCLCPP_INFO(get_logger(), "saved tuned parameters to %s", path.c_str());
  }

  /// The RTK pose at this scan's stamp. Missing TF is a rejection, not a crash.
  bool lookupPose(const std_msgs::msg::Header & header, Eigen::Isometry3d & pose) const
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        params_.world_frame, params_.base_frame, header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      pose = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException & exception) {
      last_tf_error_ = exception.what();
      return false;
    }
  }

  /// base_link -> lidar: the current guess being calibrated.
  Eigen::Isometry3d baseFromLidar(const std::string & lidar_frame)
  {
    if (params_.extrinsic_source == "tf_static") {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          params_.base_frame, lidar_frame, tf2::TimePointZero);
        return tf2::transformToEigen(tf);
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no %s -> %s in TF (%s); falling back to initial_extrinsic.xyz/rpy",
          params_.base_frame.c_str(), lidar_frame.c_str(), exception.what());
      }
    }
    return params_.initialExtrinsic();
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    ++scans_;

    const std::string lidar_frame =
      params_.lidar_frame.empty() ? msg.header.frame_id : params_.lidar_frame;

    Detection detection;
    detection.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();

    if (!lookupPose(msg.header, detection.world_from_base)) {
      reject(Reject::kNoPose);
      report();
      return;
    }

    Cloud cloud;
    pcl::fromROSMsg(msg, cloud);
    if (cloud.empty()) {
      reject(Reject::kNoCloud);
      report();
      return;
    }

    const Eigen::Isometry3d base_from_lidar = baseFromLidar(lidar_frame);
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(roiMarker(params_.corner, params_.base_frame, msg.header.stamp));

    if (params_.ground.enabled) {
      Reject reason = Reject::kNone;
      detection.ground =
        ground_detector_.detect(cloud, params_.ground, base_from_lidar, reason);
      if (detection.ground.valid) {
        ground_detector_.accumulate(detection.ground);
      } else {
        reject(reason);
      }
      markers.markers.push_back(
        groundMarker(detection.ground, lidar_frame, msg.header.stamp));
    }

    if (params_.corner.enabled) {
      Reject reason = Reject::kNone;
      detection.corner = corner_detector_.detect(
        cloud, params_.corner, base_from_lidar, detection.world_from_base,
        detection.ground, reason);
      if (detection.corner.valid) {
        world_corners_.push_back(
          detection.world_from_base * base_from_lidar * detection.corner.point);
      } else {
        reject(reason);
      }
      const auto corner_markers =
        cornerMarkers(detection.corner, lidar_frame, msg.header.stamp);
      markers.markers.insert(markers.markers.end(), corner_markers.markers.begin(),
                             corner_markers.markers.end());
    }

    if (!world_corners_.empty()) {
      markers.markers.push_back(
        cornerTrailMarker(world_corners_, params_.world_frame, msg.header.stamp));
    }
    marker_pub_->publish(markers);

    if (detection.usable()) {
      ++accepted_;
      log_.write(detection);
    }
    report();
  }

  void reject(Reject reason)
  {
    ++rejections_[reason];
  }

  /// Throttled status: what is being accepted, and when not, why. This is what
  /// tells the operator which parameter to move next.
  void report()
  {
    const auto now = get_clock()->now();
    if (last_report_.nanoseconds() != 0 && (now - last_report_).seconds() < 2.0) {
      return;
    }
    last_report_ = now;

    std::ostringstream out;
    out << "scans " << scans_ << ", accepted " << accepted_
        << " (ground " << ground_detector_.accepted()
        << ", corner " << corner_detector_.accepted() << ")";
    for (const auto & [reason, count] : rejections_) {
      out << "\n  rejected " << count << "x: " << describe(reason);
    }
    if (GroundObservation average; ground_detector_.average(average)) {
      out << "\n  mean ground normal in lidar frame: [" << average.normal.x() << ", "
          << average.normal.y() << ", " << average.normal.z()
          << "], height " << average.offset << " m";
      double roll = 0.0;
      double pitch = 0.0;
      double height = 0.0;
      groundToRollPitchHeight(average.normal, average.offset, roll, pitch, height);
      out << "\n  implied lidar tilt: roll " << roll * 180.0 / M_PI
          << " deg, pitch " << pitch * 180.0 / M_PI << " deg";
    }
    if (world_corners_.size() > 1) {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto & p : world_corners_) {
        mean += p;
      }
      mean /= static_cast<double>(world_corners_.size());
      double spread = 0.0;
      for (const auto & p : world_corners_) {
        spread = std::max(spread, (p - mean).norm());
      }
      // With a correct extrinsic this collapses toward zero; it is the single
      // number that says whether the calibration is working.
      out << "\n  corner spread in " << params_.world_frame << ": " << spread << " m over "
          << world_corners_.size() << " detections";
    }
    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  CalibParams params_;
  GroundPlaneDetector ground_detector_;
  CornerDetector corner_detector_;
  DetectionLog log_;

  std::vector<Eigen::Vector3d> world_corners_;
  std::map<Reject, size_t> rejections_;
  size_t scans_ = 0;
  size_t accepted_ = 0;
  rclcpp::Time last_report_{0, 0, RCL_ROS_TIME};
  mutable std::string last_tf_error_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace ninsar_lidar_calib

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ninsar_lidar_calib::CalibNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_calib_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
