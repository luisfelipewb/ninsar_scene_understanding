#include "ninsar_lidar_calib/corner_detector.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace ninsar_lidar_calib
{
namespace
{

/// Largest pairwise spread of a set of points, used as a face's "size".
double spanOf(const Cloud & cloud, const pcl::PointIndices & indices)
{
  if (indices.indices.size() < 2) {
    return 0.0;
  }
  Eigen::Vector3f low = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f high = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  for (const auto index : indices.indices) {
    const auto & p = cloud[index];
    low = low.cwiseMin(p.getVector3fMap());
    high = high.cwiseMax(p.getVector3fMap());
  }
  return (high - low).maxCoeff();
}

Eigen::Vector3d centroidOf(const Cloud & cloud, const pcl::PointIndices & indices)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto index : indices.indices) {
    sum += cloud[index].getVector3fMap().cast<double>();
  }
  return indices.indices.empty() ? sum : (sum / static_cast<double>(indices.indices.size()));
}

}  // namespace

bool intersectPlanes(const std::array<Eigen::Vector4d, 3> & planes, Eigen::Vector3d & point)
{
  Eigen::Matrix3d normals;
  Eigen::Vector3d offsets;
  for (int i = 0; i < 3; ++i) {
    normals.row(i) = planes[i].head<3>().transpose();
    offsets(i) = -planes[i](3);
  }
  // Near-parallel planes make this singular; the orthogonality check upstream
  // normally catches that first, but a solve can still degrade.
  const Eigen::FullPivLU<Eigen::Matrix3d> lu(normals);
  if (!lu.isInvertible() || std::abs(normals.determinant()) < 1e-6) {
    return false;
  }
  point = lu.solve(offsets);
  return point.allFinite();
}

Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & m)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d r = svd.matrixU() * svd.matrixV().transpose();
  if (r.determinant() < 0.0) {
    Eigen::Matrix3d v = svd.matrixV();
    v.col(2) *= -1.0;
    r = svd.matrixU() * v.transpose();
  }
  return r;
}

const std::vector<Eigen::Matrix3d> & axisPermutations()
{
  // Built once: the 24 signed axis permutations with determinant +1.
  static const std::vector<Eigen::Matrix3d> kRotations = [] {
    std::vector<Eigen::Matrix3d> out;
    const std::array<std::array<int, 3>, 6> orders{
      {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}};
    for (const auto & order : orders) {
      for (int signs = 0; signs < 8; ++signs) {
        Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
        for (int col = 0; col < 3; ++col) {
          const double sign = (signs & (1 << col)) ? -1.0 : 1.0;
          m(order[col], col) = sign;
        }
        if (m.determinant() > 0.0) {
          out.push_back(m);
        }
      }
    }
    return out;
  }();
  return kRotations;
}

std::array<Eigen::Vector3d, 8> CornerDetector::roiCorners(const CornerParams & params)
{
  const Eigen::Vector3d half = params.roi_size * 0.5;
  std::array<Eigen::Vector3d, 8> corners;
  int index = 0;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        corners[index++] = params.roi_center +
          Eigen::Vector3d(sx * half.x(), sy * half.y(), sz * half.z());
      }
    }
  }
  return corners;
}

bool CornerDetector::segmentFaces(
  const Cloud & cloud, const CornerParams & params,
  const Eigen::Isometry3d & base_from_lidar,
  const GroundObservation & ground,
  std::vector<Face> & faces, Reject & reason) const
{
  faces.clear();

  Cloud::Ptr working(new Cloud(cloud));

  if (params.voxel_leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(working);
    voxel.setLeafSize(params.voxel_leaf, params.voxel_leaf, params.voxel_leaf);
    Cloud::Ptr reduced(new Cloud);
    voxel.filter(*reduced);
    working.swap(reduced);
  }

  // The ROI is given in base_link and rides along with the robot. CropBox
  // applies setTransform to the cloud first, so the box bounds below are read
  // in base_link.
  {
    const Eigen::Vector3d half = params.roi_size * 0.5;
    const Eigen::Vector3d low = params.roi_center - half;
    const Eigen::Vector3d high = params.roi_center + half;

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(working);
    crop.setMin(Eigen::Vector4f(low.x(), low.y(), low.z(), 1.0f).cast<float>());
    crop.setMax(Eigen::Vector4f(high.x(), high.y(), high.z(), 1.0f).cast<float>());
    crop.setTransform(Eigen::Affine3f(base_from_lidar.matrix().cast<float>()));
    Cloud::Ptr roi(new Cloud);
    crop.filter(*roi);
    working.swap(roi);
  }

  // Drop the ground. A horizontal patch inside the ROI really is perpendicular
  // to two faces of an upright cube, so without this it is accepted as a face
  // and the "corner" lands on the wrong edge entirely.
  if (params.exclude_ground && ground.valid) {
    Cloud::Ptr kept(new Cloud);
    kept->reserve(working->size());
    for (const auto & point : *working) {
      const double distance =
        ground.normal.dot(point.getVector3fMap().cast<double>()) + ground.offset;
      if (std::abs(distance) > params.ground_margin) {
        kept->push_back(point);
      }
    }
    working.swap(kept);
  }

  if (static_cast<int>(working->size()) < 3 * params.min_inliers) {
    reason = Reject::kRoiEmpty;
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params.distance_threshold);
  seg.setMaxIterations(1000);

  for (int pass = 0; pass < 3; ++pass) {
    if (static_cast<int>(working->size()) < params.min_inliers) {
      reason = Reject::kTooFewInliers;
      return false;
    }

    seg.setInputCloud(working);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients coefficients;
    seg.segment(*inliers, coefficients);

    if (static_cast<int>(inliers->indices.size()) < params.min_inliers ||
        coefficients.values.size() < 4)
    {
      reason = Reject::kTooFewInliers;
      return false;
    }

    Eigen::Vector4d model(coefficients.values[0], coefficients.values[1],
                          coefficients.values[2], coefficients.values[3]);
    const double norm = model.head<3>().norm();
    if (norm < 1e-9) {
      reason = Reject::kDegenerate;
      return false;
    }
    model /= norm;

    Face face;
    face.model = model;
    face.inliers = static_cast<int>(inliers->indices.size());
    face.extent = spanOf(*working, *inliers);
    face.centroid = centroidOf(*working, *inliers);
    faces.push_back(face);

    // Remove this face's points so the next pass finds a different plane.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(working);
    extract.setIndices(inliers);
    extract.setNegative(true);
    Cloud::Ptr remainder(new Cloud);
    extract.filter(*remainder);
    working.swap(remainder);
  }

  return true;
}

void CornerDetector::canonicalize(
  Eigen::Matrix3d & axes, std::array<int, 3> & inliers, std::array<double, 3> & extents,
  const Eigen::Matrix3d & world_from_lidar_rotation)
{
  // Compare in the world frame: there the target is static, so the reference
  // stays valid however far the robot drives. Comparing in the lidar frame
  // would drift apart as the robot circles the target.
  const Eigen::Matrix3d world_axes = world_from_lidar_rotation * axes;

  if (!has_reference_) {
    reference_axes_ = world_axes;
    has_reference_ = true;
    return;
  }

  // RANSAC hands back the three faces in an arbitrary order with arbitrary
  // normal signs. Left alone that silently scrambles the orientation residual,
  // so pick the signed permutation that best matches the reference triad.
  double best_score = -std::numeric_limits<double>::max();
  Eigen::Matrix3d best = Eigen::Matrix3d::Identity();
  for (const auto & permutation : axisPermutations()) {
    const Eigen::Matrix3d candidate = world_axes * permutation;
    const double score = (reference_axes_.transpose() * candidate).trace();
    if (score > best_score) {
      best_score = score;
      best = permutation;
    }
  }

  axes = axes * best;

  // The same permutation has to follow the per-face bookkeeping, or the
  // reported inlier counts and extents would describe the wrong faces.
  std::array<int, 3> permuted_inliers{0, 0, 0};
  std::array<double, 3> permuted_extents{0.0, 0.0, 0.0};
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      if (best(row, col) != 0.0) {
        permuted_inliers[col] = inliers[row];
        permuted_extents[col] = extents[row];
      }
    }
  }
  inliers = permuted_inliers;
  extents = permuted_extents;
}

CornerObservation CornerDetector::detect(
  const Cloud & cloud,
  const CornerParams & params,
  const Eigen::Isometry3d & base_from_lidar,
  const Eigen::Isometry3d & world_from_base,
  const GroundObservation & ground,
  Reject & reason)
{
  CornerObservation result;
  reason = Reject::kNone;

  if (cloud.empty()) {
    reason = Reject::kNoCloud;
    return result;
  }

  std::vector<Face> faces;
  if (!segmentFaces(cloud, params, base_from_lidar, ground, faces, reason) ||
      faces.size() != 3)
  {
    return result;
  }

  // The faces of a cube corner are mutually perpendicular; anything else is a
  // wall, the ground, or clutter that happened to fit.
  const double cos_tolerance =
    std::cos((90.0 - params.orthogonality_tolerance_deg) * M_PI / 180.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (std::abs(faces[i].normal().dot(faces[j].normal())) > cos_tolerance) {
        reason = Reject::kNotPerpendicular;
        return result;
      }
    }
  }

  if (params.size_guess > 0.0) {
    const double low = std::max(0.0, params.size_guess - params.size_margin);
    const double high = params.size_guess + params.size_margin;
    for (const auto & face : faces) {
      if (face.extent < low || face.extent > high) {
        reason = Reject::kBadExtent;
        return result;
      }
    }
  }

  std::array<Eigen::Vector4d, 3> planes{faces[0].model, faces[1].model, faces[2].model};
  Eigen::Vector3d point;
  if (!intersectPlanes(planes, point)) {
    reason = Reject::kDegenerate;
    return result;
  }

  // Point every normal back toward the sensor, so the triad is built from a
  // repeatable choice rather than whichever sign RANSAC happened to return.
  Eigen::Matrix3d axes;
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d normal = faces[i].normal();
    if (normal.dot(-point) < 0.0) {
      normal = -normal;
    }
    axes.col(i) = normal;
  }
  axes = orthonormalize(axes);

  result.valid = true;
  result.point = point;
  result.axes = axes;
  for (int i = 0; i < 3; ++i) {
    result.inliers[i] = faces[i].inliers;
    result.extents[i] = faces[i].extent;
  }

  const Eigen::Matrix3d world_from_lidar_rotation =
    (world_from_base * base_from_lidar).linear();
  canonicalize(result.axes, result.inliers, result.extents, world_from_lidar_rotation);
  ++accepted_;
  return result;
}

void CornerDetector::reset()
{
  has_reference_ = false;
  reference_axes_.setIdentity();
  accepted_ = 0;
}

}  // namespace ninsar_lidar_calib
