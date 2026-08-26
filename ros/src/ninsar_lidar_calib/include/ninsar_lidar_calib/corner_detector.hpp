// Stage B: find the corner target - three mutually perpendicular faces.
#pragma once

#include "ninsar_lidar_calib/detection.hpp"
#include "ninsar_lidar_calib/ground_plane.hpp"
#include "ninsar_lidar_calib/params.hpp"

#include <Eigen/Geometry>
#include <array>
#include <vector>

namespace ninsar_lidar_calib
{

/// A single RANSAC plane and the points that voted for it.
struct Face
{
  Eigen::Vector4d model{0.0, 0.0, 1.0, 0.0};  ///< a, b, c, d with |(a,b,c)| = 1
  int inliers = 0;
  double extent = 0.0;                        ///< largest span of the inliers
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal() const { return model.head<3>(); }
};

/// Segments three orthogonal planes inside a region of interest and turns them
/// into a 6-DoF pose for the target.
///
/// The three faces come out of RANSAC in an arbitrary order and with arbitrary
/// normal signs, which would silently corrupt the optimization. `detect`
/// therefore canonicalizes every observation against the first accepted one.
class CornerDetector
{
public:
  /// `base_from_lidar` maps the ROI box (given in base_link) into the cloud's
  /// own frame. `world_from_base` is the RTK pose, used only to compare this
  /// scan's axes against the reference in the world frame - the target does not
  /// move there, however far the robot drives.
  /// `ground` is this scan's ground plane; when valid and
  /// `params.exclude_ground` is set, its points are removed before segmenting,
  /// so a ground patch inside the ROI cannot pose as one of the three faces.
  CornerObservation detect(
    const Cloud & cloud,
    const CornerParams & params,
    const Eigen::Isometry3d & base_from_lidar,
    const Eigen::Isometry3d & world_from_base,
    const GroundObservation & ground,
    Reject & reason);

  void reset();
  size_t accepted() const { return accepted_; }

  /// Corner of the ROI box in base_link, for drawing it in RViz.
  static std::array<Eigen::Vector3d, 8> roiCorners(const CornerParams & params);

private:
  /// Crops to the ROI and pulls out up to three planes.
  bool segmentFaces(
    const Cloud & cloud, const CornerParams & params,
    const Eigen::Isometry3d & base_from_lidar,
    const GroundObservation & ground,
    std::vector<Face> & faces, Reject & reason) const;

  /// Reorders and flips `axes` to the permutation of the reference triad that
  /// best matches it, so face 1 stays face 1 across scans. `world_from_lidar`
  /// takes the comparison into the world frame, where the target is static.
  void canonicalize(Eigen::Matrix3d & axes, std::array<int, 3> & inliers,
                    std::array<double, 3> & extents,
                    const Eigen::Matrix3d & world_from_lidar_rotation);

  bool has_reference_ = false;
  /// Reference triad expressed in the world frame.
  Eigen::Matrix3d reference_axes_ = Eigen::Matrix3d::Identity();
  size_t accepted_ = 0;
};

/// The point where three planes meet. False when they are near-parallel.
bool intersectPlanes(const std::array<Eigen::Vector4d, 3> & planes, Eigen::Vector3d & point);

/// Nearest rotation matrix to `m`, via SVD.
Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & m);

/// The 24 proper rotations that permute and flip a coordinate triad.
const std::vector<Eigen::Matrix3d> & axisPermutations();

}  // namespace ninsar_lidar_calib
