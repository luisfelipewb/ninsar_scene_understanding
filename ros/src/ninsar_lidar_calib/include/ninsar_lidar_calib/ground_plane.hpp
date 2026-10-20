// Stage A: find the ground plane, which pins down roll, pitch and lidar height.
#pragma once

#include "ninsar_lidar_calib/detection.hpp"
#include "ninsar_lidar_calib/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <memory>

namespace ninsar_lidar_calib
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

/// Detects the ground with a RANSAC plane constrained to lie near the normal
/// implied by the current extrinsic guess, so it cannot latch onto a wall.
class GroundPlaneDetector
{
public:
  /// `base_from_lidar` is the current extrinsic guess; it supplies both the
  /// expected ground normal and the height band to search in.
  GroundObservation detect(
    const Cloud & cloud,
    const GroundParams & params,
    const Eigen::Isometry3d & base_from_lidar,
    Reject & reason) const;

  /// Mean of the accepted unit normals, renormalized, and the mean offset.
  /// Returns false while fewer than one observation has been accumulated.
  bool average(GroundObservation & out) const;

  void accumulate(const GroundObservation & observation);
  void reset();

  size_t accepted() const { return count_; }

private:
  Eigen::Vector3d normal_sum_ = Eigen::Vector3d::Zero();
  double offset_sum_ = 0.0;
  size_t count_ = 0;
};

/// Roll and pitch that rotate `normal` (in the lidar frame) onto world +Z,
/// keeping yaw at zero, plus the lidar height implied by `offset`.
/// This is what the ground plane alone can determine.
void groundToRollPitchHeight(
  const Eigen::Vector3d & normal, double offset,
  double & roll, double & pitch, double & height);

}  // namespace ninsar_lidar_calib
