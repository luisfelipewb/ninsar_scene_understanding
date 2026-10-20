// object_detector: per-frame 3D objects from cloud_in (segmentation_msgs/SemanticPointCloud).
//
//   cloud_in -> split by class (ground/unknown dropped) -> per class VoxelGrid + Euclidean clusters
//            -> gravity-aligned yaw box per cluster
//            -> ~/detections (vision_msgs/Detection3DArray, landmark list)
//               ~/markers    (visualization_msgs/MarkerArray, RViz)
//               ~/clusters   (sensor_msgs/PointCloud2 x,y,z,rgb,cluster_id, debug)
//
// Everything is published in the cloud's frame and stamp (a measurement, not a world estimate).
// Boxes are levelled with the sensor orientation carried in cloud_in's pose, so no TF is needed.
// Input contract: docs/specs/CLOUD_IN_SPEC.md. Output format and parameters:
// docs/specs/OBJECT_DETECTOR.md.

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <segmentation_msgs/msg/semantic_point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ninsar_perception/clustering.hpp"

namespace ninsar_perception
{

using segmentation_msgs::msg::SemanticPointCloud;
using vision_msgs::msg::Detection3DArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

geometry_msgs::msg::Point to_point(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

geometry_msgs::msg::Quaternion to_quaternion(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

std_msgs::msg::ColorRGBA to_color(uint32_t rgb, float alpha = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>((rgb >> 16) & 0xFF) / 255.0f;
  c.g = static_cast<float>((rgb >> 8) & 0xFF) / 255.0f;
  c.b = static_cast<float>(rgb & 0xFF) / 255.0f;
  c.a = alpha;
  return c;
}

bool contains(const std::vector<std::string> & list, const std::string & name)
{
  return std::find(list.begin(), list.end(), name) != list.end();
}

}  // namespace

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector()
  : Node("object_detector")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "cloud_in");
    max_range_ = declare_parameter("max_range", 10.0);
    min_score_ = declare_parameter("min_score", 0.0);
    ignore_classes_ = declare_parameter<std::vector<std::string>>("ignore_classes", {"ground", kUnknown});
    // Classes to detect; [""] = every category not in ignore_classes (as voxel_query's query.classes).
    for (const auto & name : declare_parameter<std::vector<std::string>>("classes", {""})) {
      if (!name.empty()) {
        classes_.push_back(name);
      }
    }
    gravity_align_ = declare_parameter("gravity_align", true);
    publish_clusters_ = declare_parameter("publish_clusters", true);
    line_width_ = declare_parameter("marker_line_width", 0.03);

    defaults_ = declare_cluster_params("", ClusterParams());
    for (const auto & name : classes_) {
      per_class_[name] = declare_cluster_params("per_class." + name + ".", defaults_);
    }

    det_pub_ = create_publisher<Detection3DArray>("~/detections", 10);
    marker_pub_ = create_publisher<MarkerArray>("~/markers", 10);
    cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/clusters", 1);

    // Reliable, keep last 1: old frames are dropped when processing falls behind. Not best effort:
    // one lost fragment of a ~18 MB cloud would drop the whole frame.
    sub_ = create_subscription<SemanticPointCloud>(
      input_topic, rclcpp::QoS(1),
      [this](SemanticPointCloud::ConstSharedPtr msg) {cloud_cb(*msg);});

    stats_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {log_stats();});

    std::ostringstream os;
    os << "input '" << sub_->get_topic_name() << "', classes ";
    if (classes_.empty()) {
      os << "<all but ignored>";
    }
    for (const auto & c : classes_) {
      os << c << " ";
    }
    os << ", max_range " << max_range_ << " m, gravity_align " << (gravity_align_ ? "on" : "off");
    RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
  }

private:
  ClusterParams declare_cluster_params(const std::string & prefix, const ClusterParams & d)
  {
    ClusterParams p;
    p.voxel_size = declare_parameter(prefix + "voxel_size", d.voxel_size);
    p.cluster_tolerance = declare_parameter(prefix + "cluster_tolerance", d.cluster_tolerance);
    p.min_cluster_size = static_cast<int>(
      declare_parameter<int64_t>(prefix + "min_cluster_size", d.min_cluster_size));
    p.max_cluster_size = static_cast<int>(
      declare_parameter<int64_t>(prefix + "max_cluster_size", d.max_cluster_size));
    p.max_dimension = declare_parameter(prefix + "max_dimension", d.max_dimension);
    return p;
  }

  void warn_once(const std::string & key, const std::string & text)
  {
    if (warned_.insert(key).second) {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
  }

  // Which categories to detect and with which parameters; rebuilt when the category list changes.
  void update_categories(const std::vector<std::string> & categories)
  {
    if (categories == categories_) {
      return;
    }
    categories_ = categories;
    keep_.assign(categories.size(), false);
    params_.assign(categories.size(), defaults_);
    std::ostringstream os;
    for (size_t k = 0; k < categories.size(); ++k) {
      const auto & name = categories[k];
      keep_[k] = name != kUnknown && !contains(ignore_classes_, name) &&
        (classes_.empty() || contains(classes_, name));
      const auto it = per_class_.find(name);
      if (it != per_class_.end()) {
        params_[k] = it->second;
      }
      if (keep_[k]) {
        os << name << " ";
      }
    }
    for (const auto & name : classes_) {
      if (!contains(categories, name)) {
        warn_once("missing:" + name, "class '" + name + "' is not in the cloud's categories");
      }
    }
    RCLCPP_INFO(get_logger(), "detecting: %s", os.str().c_str());
  }

  void cloud_cb(const SemanticPointCloud & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    ++frames_;
    update_categories(msg.categories);

    const auto & q = msg.pose.pose.orientation;
    const Eigen::Matrix3d R = level_rotation(Eigen::Quaterniond(q.w, q.x, q.y, q.z), gravity_align_);

    SplitOptions options;
    options.max_range = max_range_;
    options.min_score = min_score_;
    options.keep_category = keep_;
    options.rotation = R.cast<float>();
    SplitStats split;
    std::map<int, Cloud::Ptr> by_class;
    try {
      by_class = split_by_class(msg.cloud, decode_instance_table(msg.instances, msg.categories),
                                options, &split);
    } catch (const std::runtime_error & e) {
      warn_once("fields", std::string("Cannot read cloud_in, skipping frames: ") + e.what());
      return;
    }
    if (split.bad_instance) {
      warn_once("instance", "instance_id outside the instances table; those points are dropped");
    }

    std::vector<ClassResult> results;
    for (const auto & [category, cloud] : by_class) {
      results.push_back(detect_class(cloud, category, params_[category]));
      if (results.back().voxel_overflow) {
        warn_once("voxel:" + categories_[category],
                  "voxel_size of '" + categories_[category] + "' overflows VoxelGrid for this extent; "
                  "clustering without downsampling (raise voxel_size or lower max_range)");
      }
    }

    publish(msg.cloud.header, R, results);

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    ++published_;
    ms_sum_ += ms;
    ms_max_ = std::max(ms_max_, ms);
  }

  void publish(const std_msgs::msg::Header & header, const Eigen::Matrix3d & R,
               const std::vector<ClassResult> & results)
  {
    Detection3DArray detections;
    detections.header = header;
    MarkerArray markers;
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const auto & result : results) {
      for (const auto & d : result.detections) {
        const std::string & name = categories_[d.category];
        const SensorPose box = box_in_sensor(d.box, R);
        const Eigen::Matrix3d cov = covariance_in_sensor(d.covariance, R);
        ++class_counts_[name];

        vision_msgs::msg::Detection3D det;
        det.header = header;
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = name;
        hyp.hypothesis.score = d.score;
        hyp.pose.pose.position = to_point(R.transpose() * d.centroid);
        hyp.pose.pose.orientation = to_quaternion(box.orientation);
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            hyp.pose.covariance[r * 6 + c] = cov(r, c);
          }
        }
        det.results.push_back(hyp);
        det.bbox.center.position = to_point(box.position);
        det.bbox.center.orientation = to_quaternion(box.orientation);
        det.bbox.size.x = d.box.size.x();
        det.bbox.size.y = d.box.size.y();
        det.bbox.size.z = d.box.size.z();
        detections.detections.push_back(det);

        add_markers(header, name, d, box, markers);
      }
    }

    det_pub_->publish(detections);
    marker_pub_->publish(markers);
    if (publish_clusters_ && cluster_pub_->get_subscription_count() > 0) {
      cluster_pub_->publish(clusters_cloud(header, R, results));
    }
  }

  void add_markers(const std_msgs::msg::Header & header, const std::string & name,
                   const Detection & d, const SensorPose & box, MarkerArray & markers)
  {
    const auto color = to_color(class_color(d.category));

    Marker m;
    m.header = header;
    m.ns = name;
    m.id = static_cast<int>(markers.markers.size());
    m.type = Marker::LINE_LIST;
    m.action = Marker::ADD;
    m.pose.position = to_point(box.position);
    m.pose.orientation = to_quaternion(box.orientation);
    m.scale.x = line_width_;
    m.color = color;
    // corner i has bit 0/1/2 set for +x/+y/+z; an edge joins corners differing in one bit
    const Eigen::Vector3d half = d.box.size / 2;
    auto corner = [&half](int i) {
        return to_point(Eigen::Vector3d(i & 1 ? half.x() : -half.x(), i & 2 ? half.y() : -half.y(),
                                        i & 4 ? half.z() : -half.z()));
      };
    for (int i = 0; i < 8; ++i) {
      for (int bit : {1, 2, 4}) {
        if (!(i & bit)) {
          m.points.push_back(corner(i));
          m.points.push_back(corner(i | bit));
        }
      }
    }
    markers.markers.push_back(m);

    Marker t;
    t.header = header;
    t.ns = name + "/label";
    t.id = static_cast<int>(markers.markers.size());
    t.type = Marker::TEXT_VIEW_FACING;
    t.action = Marker::ADD;
    t.pose.position = to_point(box.position + box.orientation * Eigen::Vector3d(0, 0, half.z() + 0.15));
    t.pose.orientation.w = 1.0;
    t.scale.z = 0.2;
    t.color = color;
    t.text = name + " (" + std::to_string(d.indices.size()) + ")";
    markers.markers.push_back(t);
  }

  // Clustered points in the sensor frame; cluster_id = index of the detection in ~/detections.
  sensor_msgs::msg::PointCloud2 clusters_cloud(const std_msgs::msg::Header & header, const Eigen::Matrix3d & R,
                                               const std::vector<ClassResult> & results)
  {
    using sensor_msgs::msg::PointField;
    size_t n = 0;
    for (const auto & result : results) {
      for (const auto & d : result.detections) {
        n += d.indices.size();
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(5, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32,
                                  "z", 1, PointField::FLOAT32, "rgb", 1, PointField::FLOAT32,
                                  "cluster_id", 1, PointField::INT32);
    modifier.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), rgb(cloud, "rgb");
    sensor_msgs::PointCloud2Iterator<int32_t> id(cloud, "cluster_id");
    const Eigen::Matrix3f Rt = R.transpose().cast<float>();
    int32_t cluster = 0;
    for (const auto & result : results) {
      for (const auto & d : result.detections) {
        const uint32_t color = class_color(d.category) | 0xFF000000;  // PCL-packed, alpha 255
        float packed;
        std::memcpy(&packed, &color, sizeof(packed));
        for (const auto i : d.indices) {
          const auto & p = (*result.cloud)[i];
          const Eigen::Vector3f s = Rt * Eigen::Vector3f(p.x, p.y, p.z);
          *x = s.x();
          *y = s.y();
          *z = s.z();
          *rgb = packed;
          *id = cluster;
          ++x, ++y, ++z, ++rgb, ++id;
        }
        ++cluster;
      }
    }
    return cloud;
  }

  void log_stats()
  {
    std::ostringstream os;
    os << "frames " << frames_ << ", published " << published_;
    if (published_) {
      os << ", ms mean " << static_cast<int>(ms_sum_ / published_) << " max " << static_cast<int>(ms_max_);
    }
    os << ", detections:";
    for (const auto & [name, count] : class_counts_) {
      os << " " << name << " " << count;
    }
    if (class_counts_.empty()) {
      os << " none";
    }
    RCLCPP_INFO(get_logger(), "%s (last 10 s)", os.str().c_str());
    frames_ = published_ = 0;
    ms_sum_ = ms_max_ = 0.0;
    class_counts_.clear();
  }

  double max_range_;
  double min_score_;
  std::vector<std::string> ignore_classes_;
  std::vector<std::string> classes_;
  bool gravity_align_;
  bool publish_clusters_;
  double line_width_;
  ClusterParams defaults_;
  std::map<std::string, ClusterParams> per_class_;

  std::vector<std::string> categories_;  // of the last cloud
  std::vector<bool> keep_;
  std::vector<ClusterParams> params_;
  std::set<std::string> warned_;

  size_t frames_ = 0, published_ = 0;
  double ms_sum_ = 0.0, ms_max_ = 0.0;
  std::map<std::string, size_t> class_counts_;

  rclcpp::Subscription<SemanticPointCloud>::SharedPtr sub_;
  rclcpp::Publisher<Detection3DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace ninsar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ninsar_perception::ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
