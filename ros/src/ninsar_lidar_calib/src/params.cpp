#include "ninsar_lidar_calib/params.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ninsar_lidar_calib
{
namespace
{

std::vector<double> toVector(const Eigen::Vector3d & v)
{
  return {v.x(), v.y(), v.z()};
}

Eigen::Vector3d toEigen(const std::vector<double> & v, const Eigen::Vector3d & fallback)
{
  if (v.size() != 3) {
    return fallback;
  }
  return {v[0], v[1], v[2]};
}

/// Declare a parameter only once, so read() can be called after every set.
template <typename T>
void declareOnce(rclcpp::Node & node, const std::string & name, const T & value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
}

}  // namespace

Eigen::Isometry3d fromXyzRpy(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                  .toRotationMatrix();
  tf.translation() = xyz;
  return tf;
}

void toXyzRpy(const Eigen::Isometry3d & tf, Eigen::Vector3d & xyz, Eigen::Vector3d & rpy)
{
  xyz = tf.translation();

  // Read the angles straight out of R = Rz(yaw) Ry(pitch) Rx(roll), matching
  // fromXyzRpy. Eigen's eulerAngles(2, 1, 0) would also be a valid inverse, but
  // it is free to return the equivalent (yaw+pi, pi-pitch, roll+pi) branch,
  // which is correct as a rotation and useless as a number to read or paste.
  const Eigen::Matrix3d & r = tf.linear();
  rpy.y() = -std::asin(std::clamp(r(2, 0), -1.0, 1.0));
  if (std::abs(r(2, 0)) > 1.0 - 1e-9) {
    // gimbal lock: roll and yaw are not separable, so put it all in yaw
    rpy.x() = 0.0;
    rpy.z() = std::atan2(-r(0, 1), r(1, 1));
  } else {
    rpy.x() = std::atan2(r(2, 1), r(2, 2));
    rpy.z() = std::atan2(r(1, 0), r(0, 0));
  }
}

Eigen::Isometry3d CalibParams::initialExtrinsic() const
{
  return fromXyzRpy(extrinsic_xyz, extrinsic_rpy);
}

void CalibParams::declare(rclcpp::Node & node)
{
  declareOnce(node, "cloud_topic", cloud_topic);
  declareOnce(node, "world_frame", world_frame);
  declareOnce(node, "base_frame", base_frame);
  declareOnce(node, "lidar_frame", lidar_frame);
  declareOnce(node, "detections_path", detections_path);

  declareOnce(node, "initial_extrinsic.source", extrinsic_source);
  declareOnce(node, "initial_extrinsic.xyz", toVector(extrinsic_xyz));
  declareOnce(node, "initial_extrinsic.rpy", toVector(extrinsic_rpy));

  declareOnce(node, "ground.enabled", ground.enabled);
  declareOnce(node, "ground.z_range", std::vector<double>{ground.z_min, ground.z_max});
  declareOnce(node, "ground.distance_threshold", ground.distance_threshold);
  declareOnce(node, "ground.eps_angle_deg", ground.eps_angle_deg);
  declareOnce(node, "ground.min_inliers", ground.min_inliers);
  declareOnce(node, "ground.voxel_leaf", ground.voxel_leaf);

  declareOnce(node, "corner.enabled", corner.enabled);
  declareOnce(node, "corner.roi_center", toVector(corner.roi_center));
  declareOnce(node, "corner.roi_size", toVector(corner.roi_size));
  declareOnce(node, "corner.size_guess", corner.size_guess);
  declareOnce(node, "corner.size_margin", corner.size_margin);
  declareOnce(node, "corner.distance_threshold", corner.distance_threshold);
  declareOnce(node, "corner.min_inliers", corner.min_inliers);
  declareOnce(node, "corner.orthogonality_tolerance_deg",
              corner.orthogonality_tolerance_deg);
  declareOnce(node, "corner.voxel_leaf", corner.voxel_leaf);
  declareOnce(node, "corner.exclude_ground", corner.exclude_ground);
  declareOnce(node, "corner.ground_margin", corner.ground_margin);
}

void CalibParams::read(const rclcpp::Node & node)
{
  cloud_topic = node.get_parameter("cloud_topic").as_string();
  world_frame = node.get_parameter("world_frame").as_string();
  base_frame = node.get_parameter("base_frame").as_string();
  lidar_frame = node.get_parameter("lidar_frame").as_string();
  detections_path = node.get_parameter("detections_path").as_string();

  extrinsic_source = node.get_parameter("initial_extrinsic.source").as_string();
  extrinsic_xyz =
    toEigen(node.get_parameter("initial_extrinsic.xyz").as_double_array(), extrinsic_xyz);
  extrinsic_rpy =
    toEigen(node.get_parameter("initial_extrinsic.rpy").as_double_array(), extrinsic_rpy);

  ground.enabled = node.get_parameter("ground.enabled").as_bool();
  const auto z_range = node.get_parameter("ground.z_range").as_double_array();
  if (z_range.size() == 2) {
    ground.z_min = z_range[0];
    ground.z_max = z_range[1];
  }
  ground.distance_threshold = node.get_parameter("ground.distance_threshold").as_double();
  ground.eps_angle_deg = node.get_parameter("ground.eps_angle_deg").as_double();
  ground.min_inliers = static_cast<int>(node.get_parameter("ground.min_inliers").as_int());
  ground.voxel_leaf = node.get_parameter("ground.voxel_leaf").as_double();

  corner.enabled = node.get_parameter("corner.enabled").as_bool();
  corner.roi_center =
    toEigen(node.get_parameter("corner.roi_center").as_double_array(), corner.roi_center);
  corner.roi_size =
    toEigen(node.get_parameter("corner.roi_size").as_double_array(), corner.roi_size);
  corner.size_guess = node.get_parameter("corner.size_guess").as_double();
  corner.size_margin = node.get_parameter("corner.size_margin").as_double();
  corner.distance_threshold = node.get_parameter("corner.distance_threshold").as_double();
  corner.min_inliers = static_cast<int>(node.get_parameter("corner.min_inliers").as_int());
  corner.orthogonality_tolerance_deg =
    node.get_parameter("corner.orthogonality_tolerance_deg").as_double();
  corner.voxel_leaf = node.get_parameter("corner.voxel_leaf").as_double();
  corner.exclude_ground = node.get_parameter("corner.exclude_ground").as_bool();
  corner.ground_margin = node.get_parameter("corner.ground_margin").as_double();
}

void CalibParams::applyOverride(const rclcpp::Parameter & parameter)
{
  const std::string & name = parameter.get_name();

  auto asVector3 = [&parameter](const Eigen::Vector3d & fallback) {
    return toEigen(parameter.as_double_array(), fallback);
  };

  if (name == "cloud_topic") { cloud_topic = parameter.as_string(); }
  else if (name == "world_frame") { world_frame = parameter.as_string(); }
  else if (name == "base_frame") { base_frame = parameter.as_string(); }
  else if (name == "lidar_frame") { lidar_frame = parameter.as_string(); }
  else if (name == "detections_path") { detections_path = parameter.as_string(); }
  else if (name == "initial_extrinsic.source") { extrinsic_source = parameter.as_string(); }
  else if (name == "initial_extrinsic.xyz") { extrinsic_xyz = asVector3(extrinsic_xyz); }
  else if (name == "initial_extrinsic.rpy") { extrinsic_rpy = asVector3(extrinsic_rpy); }
  else if (name == "ground.enabled") { ground.enabled = parameter.as_bool(); }
  else if (name == "ground.z_range") {
    const auto range = parameter.as_double_array();
    if (range.size() == 2) {
      ground.z_min = range[0];
      ground.z_max = range[1];
    }
  } else if (name == "ground.distance_threshold") {
    ground.distance_threshold = parameter.as_double();
  } else if (name == "ground.eps_angle_deg") {
    ground.eps_angle_deg = parameter.as_double();
  } else if (name == "ground.min_inliers") {
    ground.min_inliers = static_cast<int>(parameter.as_int());
  } else if (name == "ground.voxel_leaf") {
    ground.voxel_leaf = parameter.as_double();
  } else if (name == "corner.enabled") {
    corner.enabled = parameter.as_bool();
  } else if (name == "corner.roi_center") {
    corner.roi_center = asVector3(corner.roi_center);
  } else if (name == "corner.roi_size") {
    corner.roi_size = asVector3(corner.roi_size);
  } else if (name == "corner.size_guess") {
    corner.size_guess = parameter.as_double();
  } else if (name == "corner.size_margin") {
    corner.size_margin = parameter.as_double();
  } else if (name == "corner.distance_threshold") {
    corner.distance_threshold = parameter.as_double();
  } else if (name == "corner.min_inliers") {
    corner.min_inliers = static_cast<int>(parameter.as_int());
  } else if (name == "corner.orthogonality_tolerance_deg") {
    corner.orthogonality_tolerance_deg = parameter.as_double();
  } else if (name == "corner.voxel_leaf") {
    corner.voxel_leaf = parameter.as_double();
  } else if (name == "corner.exclude_ground") {
    corner.exclude_ground = parameter.as_bool();
  } else if (name == "corner.ground_margin") {
    corner.ground_margin = parameter.as_double();
  }
  // Unknown names are ignored here; rclcpp rejects undeclared parameters itself.
}

std::string CalibParams::toYaml(const std::string & node_name) const
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);

  auto vec = [](const Eigen::Vector3d & v) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(6);
    s << '[' << v.x() << ", " << v.y() << ", " << v.z() << ']';
    return s.str();
  };

  out << node_name << ":\n"
      << "  ros__parameters:\n"
      << "    cloud_topic: " << cloud_topic << "\n"
      << "    world_frame: " << world_frame << "\n"
      << "    base_frame: " << base_frame << "\n"
      << "    lidar_frame: \"" << lidar_frame << "\"\n"
      << "    detections_path: " << detections_path << "\n"
      << "    initial_extrinsic:\n"
      << "      source: " << extrinsic_source << "\n"
      << "      xyz: " << vec(extrinsic_xyz) << "\n"
      << "      rpy: " << vec(extrinsic_rpy) << "\n"
      << "    ground:\n"
      << "      enabled: " << (ground.enabled ? "true" : "false") << "\n"
      << "      z_range: [" << ground.z_min << ", " << ground.z_max << "]\n"
      << "      distance_threshold: " << ground.distance_threshold << "\n"
      << "      eps_angle_deg: " << ground.eps_angle_deg << "\n"
      << "      min_inliers: " << ground.min_inliers << "\n"
      << "      voxel_leaf: " << ground.voxel_leaf << "\n"
      << "    corner:\n"
      << "      enabled: " << (corner.enabled ? "true" : "false") << "\n"
      << "      roi_center: " << vec(corner.roi_center) << "\n"
      << "      roi_size: " << vec(corner.roi_size) << "\n"
      << "      size_guess: " << corner.size_guess << "\n"
      << "      size_margin: " << corner.size_margin << "\n"
      << "      distance_threshold: " << corner.distance_threshold << "\n"
      << "      min_inliers: " << corner.min_inliers << "\n"
      << "      orthogonality_tolerance_deg: " << corner.orthogonality_tolerance_deg << "\n"
      << "      voxel_leaf: " << corner.voxel_leaf << "\n"
      << "      exclude_ground: " << (corner.exclude_ground ? "true" : "false") << "\n"
      << "      ground_margin: " << corner.ground_margin;
  return out.str();
}

std::string CalibParams::validate() const
{
  if (ground.z_min >= ground.z_max) {
    return "ground.z_range must be [min, max] with min < max";
  }
  if (ground.distance_threshold <= 0.0) {
    return "ground.distance_threshold must be positive";
  }
  if (ground.eps_angle_deg <= 0.0 || ground.eps_angle_deg >= 90.0) {
    return "ground.eps_angle_deg must be in (0, 90)";
  }
  if (ground.min_inliers < 3) {
    return "ground.min_inliers must be at least 3";
  }
  if ((corner.roi_size.array() <= 0.0).any()) {
    return "corner.roi_size must be positive in every axis";
  }
  if (corner.distance_threshold <= 0.0) {
    return "corner.distance_threshold must be positive";
  }
  if (corner.min_inliers < 3) {
    return "corner.min_inliers must be at least 3";
  }
  if (corner.orthogonality_tolerance_deg <= 0.0 ||
      corner.orthogonality_tolerance_deg >= 45.0)
  {
    return "corner.orthogonality_tolerance_deg must be in (0, 45)";
  }
  if (corner.size_guess < 0.0) {
    return "corner.size_guess must not be negative (0 disables the check)";
  }
  if (corner.size_margin < 0.0) {
    return "corner.size_margin must not be negative";
  }
  if (corner.ground_margin < 0.0) {
    return "corner.ground_margin must not be negative";
  }
  if (ground.voxel_leaf < 0.0 || corner.voxel_leaf < 0.0) {
    return "voxel_leaf must not be negative (0 disables downsampling)";
  }
  return {};
}

std::string CalibParams::summary() const
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(3);
  out << "current guesses (paste into the config, or `ros2 param dump`):\n"
      << "  ground.z_range:             [" << ground.z_min << ", " << ground.z_max << "]\n"
      << "  ground.eps_angle_deg:       " << ground.eps_angle_deg << "\n"
      << "  ground.distance_threshold:  " << ground.distance_threshold << "\n"
      << "  ground.min_inliers:         " << ground.min_inliers << "\n"
      << "  corner.roi_center:          [" << corner.roi_center.x() << ", "
      << corner.roi_center.y() << ", " << corner.roi_center.z() << "]\n"
      << "  corner.roi_size:            [" << corner.roi_size.x() << ", "
      << corner.roi_size.y() << ", " << corner.roi_size.z() << "]\n"
      << "  corner.size_guess:          " << corner.size_guess << "\n"
      << "  corner.distance_threshold:  " << corner.distance_threshold << "\n"
      << "  corner.min_inliers:         " << corner.min_inliers << "\n"
      << "  corner.orthogonality_tolerance_deg: " << corner.orthogonality_tolerance_deg << "\n"
      << "  corner.exclude_ground:      " << (corner.exclude_ground ? "true" : "false")
      << " (margin " << corner.ground_margin << " m)";
  return out.str();
}

}  // namespace ninsar_lidar_calib
