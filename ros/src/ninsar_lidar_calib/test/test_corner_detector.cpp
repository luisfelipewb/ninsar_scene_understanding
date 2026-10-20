#include "ninsar_lidar_calib/corner_detector.hpp"

#include "synthetic.hpp"

#include <gtest/gtest.h>

using namespace ninsar_lidar_calib;
using namespace ninsar_lidar_calib::testing;

namespace
{

CornerParams defaultParams()
{
  CornerParams params;
  params.roi_center = Eigen::Vector3d(3.0, 0.0, 0.5);
  params.roi_size = Eigen::Vector3d(2.0, 2.0, 2.0);
  params.min_inliers = 200;
  params.distance_threshold = 0.02;
  params.voxel_leaf = 0.0;
  return params;
}

/// The target sits 3 m ahead of the robot, axis-aligned.
Eigen::Isometry3d targetPose()
{
  return fromXyzRpy({3.0, 0.0, 0.3}, {0.0, 0.0, 0.0});
}

}  // namespace

TEST(Geometry, IntersectsThreePlanes)
{
  const std::array<Eigen::Vector4d, 3> planes{
    Eigen::Vector4d(1.0, 0.0, 0.0, -2.0),
    Eigen::Vector4d(0.0, 1.0, 0.0, -3.0),
    Eigen::Vector4d(0.0, 0.0, 1.0, -4.0)};
  Eigen::Vector3d point;
  ASSERT_TRUE(intersectPlanes(planes, point));
  EXPECT_NEAR(point.x(), 2.0, 1e-9);
  EXPECT_NEAR(point.y(), 3.0, 1e-9);
  EXPECT_NEAR(point.z(), 4.0, 1e-9);
}

TEST(Geometry, RejectsParallelPlanes)
{
  const std::array<Eigen::Vector4d, 3> planes{
    Eigen::Vector4d(1.0, 0.0, 0.0, -2.0),
    Eigen::Vector4d(1.0, 0.0, 0.0, -3.0),
    Eigen::Vector4d(0.0, 0.0, 1.0, -4.0)};
  Eigen::Vector3d point;
  EXPECT_FALSE(intersectPlanes(planes, point));
}

TEST(Geometry, AxisPermutationsAreTwentyFourProperRotations)
{
  const auto & rotations = axisPermutations();
  ASSERT_EQ(rotations.size(), 24u);
  for (const auto & rotation : rotations) {
    EXPECT_NEAR(rotation.determinant(), 1.0, 1e-9);
    EXPECT_TRUE((rotation * rotation.transpose()).isIdentity(1e-9));
  }
}

TEST(CornerDetector, RecoversTheCornerPoint)
{
  const Eigen::Isometry3d base_from_lidar =
    fromXyzRpy({0.1, 0.0, 1.2}, {0.0, 2.0 * M_PI / 180.0, 0.0});
  const Eigen::Isometry3d lidar_from_base = base_from_lidar.inverse();

  Cloud cloud;
  addCorner(cloud, lidar_from_base, targetPose());
  ASSERT_GT(cloud.size(), 1000u);

  CornerDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, defaultParams(), base_from_lidar, Eigen::Isometry3d::Identity(), GroundObservation{}, reason);

  ASSERT_TRUE(observation.valid) << describe(reason);

  // The detected corner, taken back into base_link, must be where we put it.
  const Eigen::Vector3d in_base = base_from_lidar * observation.point;
  EXPECT_NEAR(in_base.x(), 3.0, 0.02);
  EXPECT_NEAR(in_base.y(), 0.0, 0.02);
  EXPECT_NEAR(in_base.z(), 0.3, 0.02);

  // The axes must be a proper orthonormal frame, whatever order they came in.
  EXPECT_NEAR(observation.axes.determinant(), 1.0, 1e-6);
  EXPECT_TRUE((observation.axes * observation.axes.transpose()).isIdentity(1e-6));
}

TEST(CornerDetector, KeepsAxesConsistentAsTheRobotMoves)
{
  // The same fixed target seen from two very different robot poses. Without
  // canonicalization RANSAC's arbitrary face order would make these disagree.
  const Eigen::Isometry3d base_from_lidar = fromXyzRpy({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});
  const Eigen::Isometry3d world_from_target = fromXyzRpy({5.0, 5.0, 0.3}, {0.0, 0.0, 0.4});

  const std::array<Eigen::Isometry3d, 2> robot_poses{
    fromXyzRpy({2.0, 5.0, 0.0}, {0.0, 0.0, 0.0}),
    fromXyzRpy({5.0, 2.0, 0.0}, {0.0, 0.0, M_PI / 2.0})};

  CornerDetector detector;
  auto params = defaultParams();
  params.roi_size = Eigen::Vector3d(3.0, 3.0, 3.0);

  std::array<Eigen::Matrix3d, 2> world_axes;
  for (size_t i = 0; i < robot_poses.size(); ++i) {
    const Eigen::Isometry3d world_from_base = robot_poses[i];
    const Eigen::Isometry3d lidar_from_world = (world_from_base * base_from_lidar).inverse();

    Cloud cloud;
    addCorner(cloud, lidar_from_world, world_from_target);

    Reject reason = Reject::kNone;
    const auto observation =
      detector.detect(cloud, params, base_from_lidar, world_from_base,
                      GroundObservation{}, reason);
    ASSERT_TRUE(observation.valid) << "pose " << i << ": " << describe(reason);

    world_axes[i] = (world_from_base * base_from_lidar).linear() * observation.axes;
  }

  // Same physical target, so the triads must agree in the world frame. The
  // tolerance is loose on purpose: this test is about face labelling, and a
  // permutation or sign flip shows up as 90 degrees or more, far outside it.
  // What remains inside it is ordinary RANSAC plane-fit noise.
  const Eigen::Matrix3d difference = world_axes[0].transpose() * world_axes[1];
  const double angle = Eigen::AngleAxisd(difference).angle();
  EXPECT_LT(angle, 15.0 * M_PI / 180.0)
    << "face labelling changed between viewpoints:\n" << difference;
}

TEST(CornerDetector, RejectsAnEmptyRoi)
{
  Cloud cloud;
  addCorner(cloud, Eigen::Isometry3d::Identity(), targetPose());

  auto params = defaultParams();
  params.roi_center = Eigen::Vector3d(-10.0, 0.0, 0.0);  // looking the wrong way

  CornerDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, params, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
    GroundObservation{}, reason);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(reason, Reject::kRoiEmpty);
}

TEST(CornerDetector, RejectsNonPerpendicularPlanes)
{
  // Two parallel walls and a floor: three planes, but not a corner.
  Cloud cloud;
  for (double u = -0.5; u <= 0.5; u += 0.01) {
    for (double v = 0.0; v <= 1.0; v += 0.01) {
      cloud.push_back(pcl::PointXYZ(2.6f, u, v));           // wall
      cloud.push_back(pcl::PointXYZ(3.4f, u, v));           // parallel wall
      cloud.push_back(pcl::PointXYZ(3.0f + u, v - 0.5f, 0.0f));  // floor
    }
  }

  CornerDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, defaultParams(), Eigen::Isometry3d::Identity(),
    Eigen::Isometry3d::Identity(), GroundObservation{}, reason);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(reason, Reject::kNotPerpendicular);
}

TEST(CornerDetector, ExcludesTheGroundFromTheFaces)
{
  // An upright cube's two vertical faces are perpendicular to the ground, so a
  // ground patch inside the ROI is a perfectly valid third "face" - and gives
  // the wrong corner. Excluding the detected ground is what prevents that.
  const Eigen::Isometry3d target = targetPose();  // corner at (3, 0, 0.3)

  Cloud cloud;
  addCorner(cloud, Eigen::Isometry3d::Identity(), target, 0.6);
  // a dense ground patch at z = -1.0, well inside the ROI
  for (double x = 2.2; x <= 3.8; x += 0.01) {
    for (double y = -0.8; y <= 0.8; y += 0.01) {
      cloud.push_back(pcl::PointXYZ(x, y, -1.0f));
    }
  }

  GroundObservation ground;
  ground.valid = true;
  ground.normal = Eigen::Vector3d::UnitZ();
  ground.offset = 1.0;  // sensor 1 m above the plane at z = -1

  auto params = defaultParams();
  params.roi_center = Eigen::Vector3d(3.0, 0.0, 0.0);
  params.roi_size = Eigen::Vector3d(2.0, 2.0, 3.0);  // deliberately includes the ground

  CornerDetector without;
  Reject reason = Reject::kNone;
  auto params_without = params;
  params_without.exclude_ground = false;
  const auto bad = without.detect(cloud, params_without, Eigen::Isometry3d::Identity(),
                                  Eigen::Isometry3d::Identity(), ground, reason);
  // the ground wins a RANSAC pass, so the "corner" is nowhere near the target
  EXPECT_TRUE(!bad.valid || (bad.point - target.translation()).norm() > 0.1)
    << "expected the ground to spoil the fit, got " << bad.point.transpose();

  CornerDetector with;
  reason = Reject::kNone;
  const auto good = with.detect(cloud, params, Eigen::Isometry3d::Identity(),
                                Eigen::Isometry3d::Identity(), ground, reason);
  ASSERT_TRUE(good.valid) << describe(reason);
  EXPECT_LT((good.point - target.translation()).norm(), 0.03);
}

TEST(CornerDetector, RejectsWrongSizedFaces)
{
  Cloud cloud;
  addCorner(cloud, Eigen::Isometry3d::Identity(), targetPose(), 0.6);

  auto params = defaultParams();
  params.size_guess = 2.0;    // we claim to expect a 2 m cube
  params.size_margin = 0.2;   // the 0.6 m faces fall well outside

  CornerDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, params, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
    GroundObservation{}, reason);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(reason, Reject::kBadExtent);
}

TEST(CornerDetector, AcceptsFacesMatchingTheSizeGuess)
{
  Cloud cloud;
  addCorner(cloud, Eigen::Isometry3d::Identity(), targetPose(), 0.6);

  auto params = defaultParams();
  params.size_guess = 0.6;
  params.size_margin = 0.3;  // generous, as configured in practice

  CornerDetector detector;
  Reject reason = Reject::kNone;
  const auto observation = detector.detect(
    cloud, params, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
    GroundObservation{}, reason);

  EXPECT_TRUE(observation.valid) << describe(reason);
}
