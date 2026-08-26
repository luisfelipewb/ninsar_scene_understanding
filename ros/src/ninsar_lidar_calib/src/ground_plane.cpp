#include "ninsar_lidar_calib/ground_plane.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>

namespace ninsar_lidar_calib
{

GroundObservation GroundPlaneDetector::detect(
  const Cloud & cloud,
  const GroundParams & params,
  const Eigen::Isometry3d & base_from_lidar,
  Reject & reason) const
{
  GroundObservation result;
  reason = Reject::kNone;

  if (cloud.empty()) {
    reason = Reject::kNoCloud;
    return result;
  }

  Cloud::Ptr working(new Cloud(cloud));

  if (params.voxel_leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(working);
    voxel.setLeafSize(params.voxel_leaf, params.voxel_leaf, params.voxel_leaf);
    Cloud::Ptr reduced(new Cloud);
    voxel.filter(*reduced);
    working.swap(reduced);
  }

  // Keep only the height band where the ground is expected. The band is given
  // in base_link, so the box is expressed there and the cloud is brought into
  // base_link by the current extrinsic guess.
  {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(working);
    crop.setMin(Eigen::Vector4f(-1e4f, -1e4f, static_cast<float>(params.z_min), 1.0f));
    crop.setMax(Eigen::Vector4f(1e4f, 1e4f, static_cast<float>(params.z_max), 1.0f));
    // setTransform is applied to the cloud before the box test, so handing it
    // base_from_lidar means the min/max above are read in base_link.
    crop.setTransform(Eigen::Affine3f(base_from_lidar.matrix().cast<float>()));
    Cloud::Ptr band(new Cloud);
    crop.filter(*band);
    working.swap(band);
  }

  if (static_cast<int>(working->size()) < params.min_inliers) {
    reason = Reject::kTooFewInliers;
    return result;
  }

  // The ground normal is world +Z; express it in the lidar frame so RANSAC can
  // be constrained to planes near it. This is the "initial guess" for the tilt.
  const Eigen::Vector3d expected_normal =
    base_from_lidar.linear().transpose() * Eigen::Vector3d::UnitZ();

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params.distance_threshold);
  seg.setAxis(expected_normal.cast<float>());
  seg.setEpsAngle(params.eps_angle_deg * M_PI / 180.0);
  seg.setMaxIterations(1000);
  seg.setInputCloud(working);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  seg.segment(inliers, coefficients);

  if (static_cast<int>(inliers.indices.size()) < params.min_inliers ||
      coefficients.values.size() < 4)
  {
    reason = Reject::kTooFewInliers;
    return result;
  }

  Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1],
                         coefficients.values[2]);
  double offset = coefficients.values[3];
  const double norm = normal.norm();
  if (norm < 1e-9) {
    reason = Reject::kDegenerate;
    return result;
  }
  normal /= norm;
  offset /= norm;

  // Point the normal up, so averaging across scans cannot cancel out.
  if (expected_normal.dot(normal) < 0.0) {
    normal = -normal;
    offset = -offset;
  }

  result.valid = true;
  result.normal = normal;
  result.offset = offset;
  result.inliers = static_cast<int>(inliers.indices.size());
  return result;
}

void GroundPlaneDetector::accumulate(const GroundObservation & observation)
{
  if (!observation.valid) {
    return;
  }
  normal_sum_ += observation.normal;
  offset_sum_ += observation.offset;
  ++count_;
}

bool GroundPlaneDetector::average(GroundObservation & out) const
{
  if (count_ == 0 || normal_sum_.norm() < 1e-9) {
    return false;
  }
  out.valid = true;
  out.normal = normal_sum_.normalized();
  out.offset = offset_sum_ / static_cast<double>(count_);
  out.inliers = static_cast<int>(count_);
  return true;
}

void GroundPlaneDetector::reset()
{
  normal_sum_.setZero();
  offset_sum_ = 0.0;
  count_ = 0;
}

void groundToRollPitchHeight(
  const Eigen::Vector3d & normal, double offset,
  double & roll, double & pitch, double & height)
{
  // The extrinsic rotation R satisfies R * normal = world +Z. Writing R as
  // Rz(yaw) Ry(pitch) Rx(roll), the yaw drops out (it turns about +Z), leaving
  //     normal = (-sin p, sin r cos p, cos r cos p)
  // which inverts directly. Going through Eigen's eulerAngles instead would be
  // free to return the equivalent-but-useless (180, 180-p, 180+r) branch.
  const Eigen::Vector3d n = normal.normalized();
  roll = std::atan2(n.y(), n.z());
  pitch = -std::asin(std::clamp(n.x(), -1.0, 1.0));
  // Plane in the lidar frame: n . p + offset = 0. The signed distance of a
  // point from it is n . p + offset, so for the sensor at the origin that is
  // offset itself - and with n pointing up, that distance is the height.
  height = offset;
}

}  // namespace ninsar_lidar_calib
