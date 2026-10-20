// Calibration parameters: every guess the operator tunes while a bag plays.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace ninsar_lidar_calib
{

/// Ground-plane detection settings. All live-tunable.
struct GroundParams
{
  double z_min = -2.0;              ///< metres in base_link, lower ground bound
  double z_max = 0.2;               ///< metres in base_link, upper ground bound
  double distance_threshold = 0.03; ///< RANSAC inlier distance
  double eps_angle_deg = 15.0;      ///< how far from the guessed normal RANSAC may go
  int min_inliers = 500;
  double voxel_leaf = 0.05;         ///< 0 disables downsampling
  bool enabled = true;
};

/// Corner-target detection settings. All live-tunable.
struct CornerParams
{
  Eigen::Vector3d roi_center{3.0, 0.0, 0.5};  ///< ROI box centre in base_link
  Eigen::Vector3d roi_size{2.0, 2.0, 1.5};    ///< ROI box extents in base_link
  double size_guess = 0.0;          ///< cube side in metres; 0 disables the check
  double size_margin = 0.3;         ///< generous tolerance on size_guess
  double distance_threshold = 0.02;
  int min_inliers = 200;
  double orthogonality_tolerance_deg = 8.0;
  double voxel_leaf = 0.02;
  bool enabled = true;
  /// Drop points lying on the detected ground before segmenting the faces.
  /// Without this a ground patch inside the ROI is happily accepted as one of
  /// the three perpendicular faces, which it genuinely is.
  bool exclude_ground = true;
  double ground_margin = 0.08;  ///< metres either side of the ground plane
};

/// Everything the detectors and the solver need.
struct CalibParams
{
  std::string cloud_topic = "/robot/lidar/points";
  std::string world_frame = "map";
  std::string base_frame = "robot_base_link";
  std::string lidar_frame;          ///< empty: take it from the cloud header

  /// Initial base_link -> lidar guess. "tf_static" reads it from the bag.
  std::string extrinsic_source = "tf_static";
  Eigen::Vector3d extrinsic_xyz{0.0, 0.0, 0.0};
  Eigen::Vector3d extrinsic_rpy{0.0, 0.0, 0.0};

  GroundParams ground;
  CornerParams corner;

  std::string detections_path = "detections.jsonl";

  /// The current base_link -> lidar guess as a transform.
  Eigen::Isometry3d initialExtrinsic() const;

  /// Declare every field on `node`, seeded from this struct's defaults.
  void declare(rclcpp::Node & node);

  /// Re-read every field from `node`'s current parameter values.
  void read(const rclcpp::Node & node);

  /// Apply one incoming parameter, so a change can be validated on a copy
  /// before it is allowed to touch the live values.
  void applyOverride(const rclcpp::Parameter & parameter);

  /// The tunable values as a ROS parameter file, ready to be reloaded with
  /// `--params-file`.
  std::string toYaml(const std::string & node_name) const;

  /// Reject nonsensical values, so a bad `ros2 param set` fails loudly.
  /// Returns an empty string when the values are usable.
  std::string validate() const;

  /// Multi-line, paste-able summary of the tunable values, for the status log.
  std::string summary() const;
};

/// Compose a transform from a translation and fixed-axis roll/pitch/yaw.
Eigen::Isometry3d fromXyzRpy(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy);

/// Inverse of fromXyzRpy: returns (x, y, z, roll, pitch, yaw).
void toXyzRpy(const Eigen::Isometry3d & tf, Eigen::Vector3d & xyz, Eigen::Vector3d & rpy);

}  // namespace ninsar_lidar_calib
