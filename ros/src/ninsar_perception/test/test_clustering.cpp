// Checks the pure side of object_detector: instance table decoding, class split, clustering, boxes, frames.

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ninsar_perception/clustering.hpp"

using namespace ninsar_perception;

namespace
{

// ninsar_bringup config/isaac/semantic_cloud_adapter.yaml categories as the server sees them ("unknown" last), shortened
const std::vector<std::string> kCategories = {"ground", "tree", "grapevine", "lemon", "unknown"};
constexpr int kGround = 0, kTree = 1, kGrapevine = 2;

vision_msgs::msg::Detection2D instance(const std::string & id,
                                       const std::vector<std::pair<std::string, double>> & results)
{
  vision_msgs::msg::Detection2D det;
  det.id = id;
  for (const auto & [cls, score] : results) {
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = cls;
    hyp.hypothesis.score = score;
    det.results.push_back(hyp);
  }
  return det;
}

// semantic_cloud_adapter's ground truth table: 0 = unknown, k + 1 = categories[k]
std::vector<InstanceClass> ground_truth_table()
{
  std::vector<vision_msgs::msg::Detection2D> instances = {instance("0", {{kUnknown, 0.05}})};
  for (size_t k = 0; k + 1 < kCategories.size(); ++k) {
    instances.push_back(instance(std::to_string(k + 1), {{kCategories[k], 1.0}}));
  }
  return decode_instance_table(instances, kCategories);
}

// cloud_in layout: x, y, z, rgb (float32), instance_id (int32)
sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<Eigen::Vector3f> & xyz, const std::vector<int32_t> & ids,
                                         bool with_instance_id = true)
{
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (with_instance_id) {
    modifier.setPointCloud2Fields(5, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32,
                                  "z", 1, PointField::FLOAT32, "rgb", 1, PointField::FLOAT32,
                                  "instance_id", 1, PointField::INT32);
  } else {
    modifier.setPointCloud2Fields(3, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32,
                                  "z", 1, PointField::FLOAT32);
  }
  modifier.resize(xyz.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (size_t i = 0; i < xyz.size(); ++i, ++x, ++y, ++z) {
    *x = xyz[i].x();
    *y = xyz[i].y();
    *z = xyz[i].z();
  }
  if (with_instance_id) {
    sensor_msgs::PointCloud2Iterator<int32_t> id(cloud, "instance_id");
    for (size_t i = 0; i < ids.size(); ++i, ++id) {
      *id = ids[i];
    }
  }
  return cloud;
}

// Points on a regular grid filling an upright box (level frame), rotated by yaw about its centre.
std::vector<Eigen::Vector3f> box_points(const Eigen::Vector3d & center, const Eigen::Vector3d & size, double yaw,
                                        double step = 0.05)
{
  std::vector<Eigen::Vector3f> pts;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3i n = (size / step).array().round().cast<int>();
  for (int i = 0; i <= n.x(); ++i) {
    for (int j = 0; j <= n.y(); ++j) {
      for (int k = 0; k <= n.z(); ++k) {
        const Eigen::Vector3d local = Eigen::Vector3d(i, j, k) * step - size / 2;
        pts.push_back((center + R * local).cast<float>());
      }
    }
  }
  return pts;
}

Cloud::Ptr to_cloud(const std::vector<Eigen::Vector3f> & pts, float score = 1.0f)
{
  auto cloud = std::make_shared<Cloud>();
  for (const auto & p : pts) {
    Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    q.intensity = score;
    cloud->push_back(q);
  }
  return cloud;
}

pcl::Indices all_indices(const Cloud & cloud)
{
  pcl::Indices idx(cloud.size());
  for (size_t i = 0; i < idx.size(); ++i) {
    idx[i] = static_cast<pcl::index_t>(i);
  }
  return idx;
}

// yaw difference modulo pi (a box is symmetric under a half turn)
double yaw_error(double a, double b)
{
  return std::abs(std::remainder(a - b, M_PI));
}

}  // namespace

// --------------------------------------------------------------------------------------
// Instance table
// --------------------------------------------------------------------------------------

TEST(DecodeInstanceTable, GroundTruth)
{
  const auto table = ground_truth_table();
  ASSERT_EQ(table.size(), kCategories.size());
  EXPECT_EQ(table[0].category, -1);  // ("unknown", 0.05)
  EXPECT_EQ(table[1].category, kGround);
  EXPECT_EQ(table[2].category, kTree);
  EXPECT_EQ(table[3].category, kGrapevine);
  EXPECT_FLOAT_EQ(table[2].score, 1.0f);
}

TEST(DecodeInstanceTable, ConfidenceBinsAndOddIds)
{
  const auto table = decode_instance_table(
    {
      instance("0", {{kUnknown, 0.05}}),
      instance("1", {{"tree", 0.25}, {kUnknown, 0.75}}),   // low confidence: still tree, score 0.25
      instance("2", {{kUnknown, 0.1}, {"grapevine", 0.9}}),
      instance("5", {{"cabbage", 1.0}}),                    // not a category -> unknown
    },
    kCategories);
  ASSERT_EQ(table.size(), 6u);
  EXPECT_EQ(table[1].category, kTree);
  EXPECT_FLOAT_EQ(table[1].score, 0.25f);
  EXPECT_EQ(table[2].category, kGrapevine);
  EXPECT_FLOAT_EQ(table[2].score, 0.9f);
  EXPECT_EQ(table[3].category, -1);  // ids not in the table
  EXPECT_EQ(table[5].category, -1);
}

// --------------------------------------------------------------------------------------
// Split
// --------------------------------------------------------------------------------------

TEST(SplitByClass, FiltersClassRangeScoreAndBadPoints)
{
  auto table = ground_truth_table();
  table[3].score = 0.4f;  // grapevine points have a low score
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto cloud = make_cloud(
    {{1, 0, 0}, {2, 0, 0}, {20, 0, 0}, {nan, 0, 0}, {1, 1, 0}, {3, 0, 0}, {4, 0, 0}},
    {1, 2, 2, 2, 99, 3, 0});  // ground, tree, far tree, NaN tree, bad id, grapevine, unknown

  SplitOptions options;
  options.max_range = 10.0;
  options.min_score = 0.5;
  options.keep_category = {false, true, true, false, false};  // tree, grapevine
  SplitStats stats;
  const auto by_class = split_by_class(cloud, table, options, &stats);

  EXPECT_EQ(stats.points, 7u);
  EXPECT_EQ(stats.far, 1u);
  EXPECT_EQ(stats.non_finite, 1u);
  EXPECT_EQ(stats.bad_instance, 1u);
  ASSERT_EQ(by_class.size(), 1u);  // grapevine dropped by min_score, ground/unknown not kept
  const auto & tree = by_class.at(kTree);
  ASSERT_EQ(tree->size(), 1u);
  EXPECT_FLOAT_EQ((*tree)[0].x, 2.0f);
  EXPECT_FLOAT_EQ((*tree)[0].intensity, 1.0f);
}

TEST(SplitByClass, RotatesIntoLevelFrame)
{
  SplitOptions options;
  options.keep_category = {false, true, false, false, false};
  options.rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const auto by_class = split_by_class(make_cloud({{1, 0, 0}}, {2}), ground_truth_table(), options);
  const auto & p = (*by_class.at(kTree))[0];
  EXPECT_NEAR(p.x, 0.0f, 1e-6);
  EXPECT_NEAR(p.y, 1.0f, 1e-6);
}

TEST(SplitByClass, ThrowsWithoutInstanceId)
{
  SplitOptions options;
  options.keep_category = {true, true, true, true, true};
  EXPECT_THROW(split_by_class(make_cloud({{1, 0, 0}}, {}, false), ground_truth_table(), options),
               std::runtime_error);
}

// --------------------------------------------------------------------------------------
// Clustering
// --------------------------------------------------------------------------------------

TEST(DetectClass, SeparatedBlobsAndMinSize)
{
  auto pts = box_points({2, 0, 0}, {0.3, 0.3, 0.3}, 0.0);
  const auto b = box_points({2, 2, 0}, {0.3, 0.3, 0.3}, 0.0);
  pts.insert(pts.end(), b.begin(), b.end());
  pts.push_back({2, -3, 0});  // isolated point: below min_cluster_size
  pts.push_back({2, -3, 0.05});

  ClusterParams params;
  params.voxel_size = 0.05;
  params.cluster_tolerance = 0.15;
  params.min_cluster_size = 5;
  const auto result = detect_class(to_cloud(pts, 0.8f), kTree, params);

  EXPECT_FALSE(result.voxel_overflow);
  ASSERT_EQ(result.detections.size(), 2u);
  for (const auto & d : result.detections) {
    EXPECT_EQ(d.category, kTree);
    EXPECT_NEAR(d.score, 0.8, 1e-5);
    EXPECT_NEAR(d.centroid.x(), 2.0, 0.03);
    EXPECT_NEAR(d.box.size.z(), 0.3, 0.06);
    EXPECT_GT(d.covariance.trace(), 0.0);
  }
}

TEST(DetectClass, MaxDimensionDropsLongClusters)
{
  ClusterParams params;
  params.min_cluster_size = 5;
  params.max_dimension = 1.0;
  const auto result = detect_class(to_cloud(box_points({3, 0, 0}, {2.0, 0.2, 0.2}, 0.0)), kTree, params);
  EXPECT_TRUE(result.detections.empty());
}

TEST(VoxelGridGuard, DetectsOverflow)
{
  const auto cloud = to_cloud({{0, 0, 0}, {100, 100, 100}});
  EXPECT_FALSE(voxel_grid_fits(*cloud, 0.01));  // 10^4 cells per axis
  EXPECT_TRUE(voxel_grid_fits(*cloud, 0.5));
  EXPECT_TRUE(voxel_grid_fits(*cloud, 0.0));    // disabled

  ClusterParams params;
  params.voxel_size = 0.001;
  params.min_cluster_size = 1;
  const auto result = detect_class(cloud, kTree, params);
  EXPECT_TRUE(result.voxel_overflow);
  EXPECT_EQ(result.cloud->size(), 2u);  // not downsampled
  EXPECT_EQ(result.detections.size(), 2u);
}

// --------------------------------------------------------------------------------------
// Boxes and frames
// --------------------------------------------------------------------------------------

TEST(FitYawBox, RecoversRotatedBox)
{
  for (double yaw : {0.0, 0.3, 2.0, -1.2, M_PI / 2}) {
    const auto cloud = to_cloud(box_points({4, -1, 0.5}, {2.0, 0.5, 1.0}, yaw));
    const YawBox box = fit_yaw_box(*cloud, all_indices(*cloud));
    EXPECT_NEAR(box.size.x(), 2.0, 1e-3) << "yaw " << yaw;
    EXPECT_NEAR(box.size.y(), 0.5, 1e-3) << "yaw " << yaw;
    EXPECT_NEAR(box.size.z(), 1.0, 1e-3) << "yaw " << yaw;
    EXPECT_NEAR(box.center.x(), 4.0, 1e-3);
    EXPECT_NEAR(box.center.y(), -1.0, 1e-3);
    EXPECT_NEAR(box.center.z(), 0.5, 1e-3);
    EXPECT_LT(yaw_error(box.yaw, yaw), 1e-3) << "yaw " << yaw << " fitted " << box.yaw;
    EXPECT_GT(box.yaw, -M_PI / 2);
    EXPECT_LE(box.yaw, M_PI / 2);
  }
}

TEST(FitYawBox, SinglePoint)
{
  const auto cloud = to_cloud({{1, 2, 3}});
  const YawBox box = fit_yaw_box(*cloud, all_indices(*cloud));
  EXPECT_NEAR(box.center.x(), 1.0, 1e-6);
  EXPECT_NEAR(box.size.norm(), 0.0, 1e-6);
}

TEST(Frames, PitchedSensorGivesUprightBoxes)
{
  // sensor pitched down 20 deg and yawed 40 deg in world
  const Eigen::Quaterniond world_q_sensor(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()));
  const Eigen::Matrix3d R = level_rotation(world_q_sensor, true);

  // an upright box in the level frame, as the sensor sees it
  const double yaw = 0.4;
  const auto level = box_points({3, 1, -0.5}, {1.5, 0.4, 0.8}, yaw);
  std::vector<Eigen::Vector3f> sensor;
  std::vector<int32_t> ids;
  for (const auto & p : level) {
    sensor.push_back((R.transpose() * p.cast<double>()).cast<float>());
    ids.push_back(2);  // tree
  }

  SplitOptions options;
  options.keep_category = {false, true, false, false, false};
  options.rotation = R.cast<float>();
  const auto by_class = split_by_class(make_cloud(sensor, ids), ground_truth_table(), options);
  ClusterParams params;
  params.min_cluster_size = 5;
  const auto result = detect_class(by_class.at(kTree), kTree, params);
  ASSERT_EQ(result.detections.size(), 1u);
  const auto & d = result.detections[0];
  EXPECT_NEAR(d.box.size.x(), 1.5, 0.06);
  EXPECT_NEAR(d.box.size.y(), 0.4, 0.06);
  EXPECT_NEAR(d.box.size.z(), 0.8, 0.06);
  EXPECT_LT(yaw_error(d.box.yaw, yaw), 0.05);

  // in the sensor frame: centre maps back, box z axis = world up seen from the sensor
  const SensorPose box = box_in_sensor(d.box, R);
  EXPECT_LT((R * box.position - d.box.center).norm(), 1e-9);
  const Eigen::Vector3d up_in_sensor = R.transpose() * Eigen::Vector3d::UnitZ();
  EXPECT_LT((box.orientation * Eigen::Vector3d::UnitZ() - up_in_sensor).norm(), 1e-9);

  const Eigen::Matrix3d cov_s = covariance_in_sensor(d.covariance, R);
  EXPECT_NEAR(cov_s.trace(), d.covariance.trace(), 1e-9);
  EXPECT_LT((R * cov_s * R.transpose() - d.covariance).norm(), 1e-9);
}

TEST(Frames, GravityAlignOff)
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(level_rotation(q, false).isIdentity());
  EXPECT_TRUE(level_rotation(Eigen::Quaterniond(0, 0, 0, 0), true).isIdentity());  // unset pose
}

TEST(ClassColor, FollowsVoxeland)
{
  EXPECT_EQ(class_color(0), 0xBCBCBCu);
  EXPECT_EQ(class_color(1), 0x9DBBE3u);
  EXPECT_EQ(class_color(8), 0xE6194Bu);
  EXPECT_EQ(class_color(22), 0xFAD4E0u);  // wraps
}
