#include "ninsar_lidar_calib/ground_plane.hpp"

#include "synthetic.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace ninsar_lidar_calib;
using namespace ninsar_lidar_calib::testing;

namespace
{

GroundParams defaultParams()
{
  GroundParams params;
  params.z_min = -3.0;
  params.z_max = 0.5;
  params.min_inliers = 100;
  params.voxel_leaf = 0.0;  // the synthetic grid is already sparse enough
  return params;
}

}  // namespace

TEST(GroundPlane, RecoversATiltedGround)
{
  // The lidar sits 1.4 m up, pitched 5 degrees and rolled 3 degrees.
  const Eigen::Isometry3d base_from_lidar =
    fromXyzRpy({0.2, 0.0, 1.4}, {3.0 * M_PI / 180.0, 5.0 * M_PI / 180.0, 0.0});
  const Eigen::Isometry3d lidar_from_world = base_from_lidar.inverse();

  Cloud cloud;
  addGround(cloud, lidar_from_world, 0.0);
  ASSERT_GT(cloud.size(), 1000u);

  GroundPlaneDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(cloud, defaultParams(), base_from_lidar, reason);

  ASSERT_TRUE(observation.valid) << describe(reason);

  // The ground normal seen from the lidar is world +Z pulled into the lidar frame.
  const Eigen::Vector3d expected = base_from_lidar.linear().transpose() *
    Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(observation.normal.dot(expected), 1.0, 1e-3);
  // The sensor is 1.4 m above the ground plane.
  EXPECT_NEAR(observation.offset, 1.4, 0.02);
}

TEST(GroundPlane, IgnoresAWallThanksToTheAngleGuess)
{
  const Eigen::Isometry3d base_from_lidar = fromXyzRpy({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});
  const Eigen::Isometry3d lidar_from_world = base_from_lidar.inverse();

  Cloud cloud;
  // A wall with many more points than the ground: without the perpendicular
  // constraint plain RANSAC would happily return the wall.
  for (double y = -6.0; y <= 6.0; y += 0.02) {
    for (double z = -1.0; z <= 4.0; z += 0.02) {
      const Eigen::Vector3d p = lidar_from_world * Eigen::Vector3d(3.0, y, z);
      cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
  }
  addGround(cloud, lidar_from_world, 0.0, 4.0, 0.1);

  GroundPlaneDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(cloud, defaultParams(), base_from_lidar, reason);

  ASSERT_TRUE(observation.valid) << describe(reason);
  EXPECT_NEAR(observation.normal.z(), 1.0, 1e-2);  // up, not sideways
  EXPECT_NEAR(observation.offset, 1.0, 0.02);
}

TEST(GroundPlane, AveragesAcrossScans)
{
  GroundPlaneDetector detector;
  GroundObservation average;
  EXPECT_FALSE(detector.average(average)) << "no observations yet";

  for (int i = 0; i < 5; ++i) {
    GroundObservation observation;
    observation.valid = true;
    // small alternating tilt, which must cancel in the mean
    const double tilt = (i % 2 == 0) ? 0.01 : -0.01;
    observation.normal = Eigen::Vector3d(tilt, 0.0, 1.0).normalized();
    observation.offset = 1.5;  // 1.5 m above the ground
    detector.accumulate(observation);
  }

  ASSERT_TRUE(detector.average(average));
  EXPECT_NEAR(average.normal.norm(), 1.0, 1e-9);
  EXPECT_NEAR(average.normal.x(), 0.002, 1e-3);
  EXPECT_NEAR(average.offset, 1.5, 1e-9);
}

TEST(GroundPlane, ReportsTheTiltThatMatchesTheExtrinsic)
{
  // Roll and pitch read back from the ground normal must be the ones that were
  // baked into the extrinsic - and in the plain branch, not an equivalent
  // 180-degree-flipped one.
  const double true_roll = 0.03;
  const double true_pitch = -0.06;
  const Eigen::Isometry3d base_from_lidar =
    fromXyzRpy({0.0, 0.0, 1.42}, {true_roll, true_pitch, 0.11});

  // the ground normal as the lidar sees it
  const Eigen::Vector3d normal =
    base_from_lidar.linear().transpose() * Eigen::Vector3d::UnitZ();

  double roll = 0.0;
  double pitch = 0.0;
  double height = 0.0;
  groundToRollPitchHeight(normal, 1.42, roll, pitch, height);

  EXPECT_NEAR(roll, true_roll, 1e-9);
  EXPECT_NEAR(pitch, true_pitch, 1e-9);
  EXPECT_NEAR(height, 1.42, 1e-9);
}

TEST(GroundPlane, ReportsTooFewInliers)
{
  Cloud cloud;
  cloud.push_back(pcl::PointXYZ(0.0f, 0.0f, -1.0f));

  GroundPlaneDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, defaultParams(), Eigen::Isometry3d::Identity(), reason);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(reason, Reject::kTooFewInliers);
}

TEST(Transforms, XyzRpyRoundTrips)
{
  // fromXyzRpy and toXyzRpy must be exact inverses, in the plain branch - not
  // an equivalent 180-degree-flipped one.
  const std::vector<Eigen::Vector3d> cases{
    {0.03, -0.06, 0.11}, {0.0, 0.0, 0.0}, {-2.9, 0.05, 1.5}, {0.5, -0.5, 3.0}};
  for (const auto & rpy : cases) {
    const Eigen::Vector3d xyz(1.0, 2.0, 3.0);
    Eigen::Vector3d back_xyz;
    Eigen::Vector3d back_rpy;
    toXyzRpy(fromXyzRpy(xyz, rpy), back_xyz, back_rpy);
    EXPECT_TRUE(back_xyz.isApprox(xyz, 1e-9));
    EXPECT_TRUE(back_rpy.isApprox(rpy, 1e-6))
      << "rpy " << rpy.transpose() << " came back as " << back_rpy.transpose();
  }
}
