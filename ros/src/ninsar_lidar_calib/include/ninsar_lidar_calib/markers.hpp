// RViz markers: the tuning aids and the before/after view of the calibration.
#pragma once

#include "ninsar_lidar_calib/detection.hpp"
#include "ninsar_lidar_calib/params.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace ninsar_lidar_calib
{

/// Wireframe of the ROI box, drawn in `base_frame` so it rides with the robot.
/// This is what the operator drags into place with `ros2 param set`.
visualization_msgs::msg::Marker roiMarker(
  const CornerParams & params, const std::string & base_frame,
  const builtin_interfaces::msg::Time & stamp);

/// The detected ground plane, as a thin slab at the plane's position.
visualization_msgs::msg::Marker groundMarker(
  const GroundObservation & ground, const std::string & lidar_frame,
  const builtin_interfaces::msg::Time & stamp);

/// The corner point (sphere) and its three face normals (arrows).
visualization_msgs::msg::MarkerArray cornerMarkers(
  const CornerObservation & corner, const std::string & lidar_frame,
  const builtin_interfaces::msg::Time & stamp);

/// Every corner estimate so far, mapped into the world frame. A tight cluster
/// means the extrinsic is right; a spread-out cloud is the error to remove.
visualization_msgs::msg::Marker cornerTrailMarker(
  const std::vector<Eigen::Vector3d> & world_points, const std::string & world_frame,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace ninsar_lidar_calib
