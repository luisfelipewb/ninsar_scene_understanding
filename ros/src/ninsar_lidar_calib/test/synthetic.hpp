// Synthetic clouds: a ground plane and a cube corner at known poses, so the
// detectors can be checked against an answer we already know.
#pragma once

#include "ninsar_lidar_calib/ground_plane.hpp"

#include <Eigen/Geometry>
#include <random>

namespace ninsar_lidar_calib
{
namespace testing
{

/// Deterministic noise, so a failing test fails the same way twice.
inline std::mt19937 & rng()
{
  static std::mt19937 generator(42);
  return generator;
}

inline double noise(double sigma)
{
  std::normal_distribution<double> distribution(0.0, sigma);
  return sigma > 0.0 ? distribution(rng()) : 0.0;
}

/// A horizontal patch at world height `z`, sampled on a grid, expressed in the
/// lidar frame via `lidar_from_world`.
inline void addGround(
  Cloud & cloud, const Eigen::Isometry3d & lidar_from_world,
  double z, double half_extent = 8.0, double step = 0.1, double sigma = 0.005)
{
  for (double x = -half_extent; x <= half_extent; x += step) {
    for (double y = -half_extent; y <= half_extent; y += step) {
      const Eigen::Vector3d world(x, y, z + noise(sigma));
      const Eigen::Vector3d p = lidar_from_world * world;
      cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()),
                                    static_cast<float>(p.z())));
    }
  }
}

/// Three perpendicular faces meeting at `world_from_corner`'s origin, each
/// spanning `side` metres, expressed in the lidar frame.
inline void addCorner(
  Cloud & cloud, const Eigen::Isometry3d & lidar_from_world,
  const Eigen::Isometry3d & world_from_corner,
  double side = 0.6, double step = 0.01, double sigma = 0.003)
{
  const Eigen::Isometry3d lidar_from_corner = lidar_from_world * world_from_corner;
  // face k is spanned by the two axes that are not k
  for (int face = 0; face < 3; ++face) {
    const int a = (face + 1) % 3;
    const int b = (face + 2) % 3;
    for (double u = 0.0; u <= side; u += step) {
      for (double v = 0.0; v <= side; v += step) {
        Eigen::Vector3d local = Eigen::Vector3d::Zero();
        local(a) = u;
        local(b) = v;
        local(face) = noise(sigma);
        const Eigen::Vector3d p = lidar_from_corner * local;
        cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()),
                                      static_cast<float>(p.y()),
                                      static_cast<float>(p.z())));
      }
    }
  }
}

}  // namespace testing
}  // namespace ninsar_lidar_calib
