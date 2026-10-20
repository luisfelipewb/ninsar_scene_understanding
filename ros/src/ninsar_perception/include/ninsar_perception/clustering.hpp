// Per-frame semantic object detection on a SemanticPointCloud (cloud_in): split the points by
// class, downsample and Euclidean-cluster each class, fit a gravity-aligned yaw box per cluster.
//
// Pure functions on message structs and PCL clouds, no ROS graph (covered by
// test/test_clustering.cpp). The node is src/object_detector_node.cpp; the input contract is
// docs/specs/CLOUD_IN_SPEC.md and the output format docs/specs/OBJECT_DETECTOR.md.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

namespace ninsar_perception
{

inline const std::string kUnknown = "unknown";

// Points carry their class score in `intensity`, so VoxelGrid averages it with the position.
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

// --------------------------------------------------------------------------------------
// Input decoding
// --------------------------------------------------------------------------------------

struct InstanceClass
{
  int category = -1;  // index into categories; -1 = unknown or not a category
  float score = 0.0f;
};

// Local instance table -> class of each instance index (atoi(id), as the Voxeland server reads it).
// The class is the best-scoring result other than "unknown", which covers both encodings of
// semantic_cloud_adapter: one-hot (class, 1.0) and confidence bins (class, p) + (unknown, 1 - p).
std::vector<InstanceClass> decode_instance_table(
  const std::vector<vision_msgs::msg::Detection2D> & instances,
  const std::vector<std::string> & categories);

struct SplitOptions
{
  double max_range = 10.0;            // in the sensor frame; cloud_in keeps far points for carving
  double min_score = 0.0;             // drop points whose class score is lower
  std::vector<bool> keep_category;    // per category index; false = not detected
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();  // sensor -> level frame
};

struct SplitStats
{
  size_t points = 0;
  size_t non_finite = 0;
  size_t far = 0;
  size_t bad_instance = 0;  // instance_id outside the table
};

// Points of each kept category (by category index), rotated into the level frame, score in intensity.
// Throws std::runtime_error if the cloud lacks x, y, z or instance_id.
std::map<int, Cloud::Ptr> split_by_class(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<InstanceClass> & table,
  const SplitOptions & options, SplitStats * stats = nullptr);

// --------------------------------------------------------------------------------------
// Clustering
// --------------------------------------------------------------------------------------

struct ClusterParams
{
  double voxel_size = 0.05;         // VoxelGrid leaf [m]; <= 0 disables downsampling
  double cluster_tolerance = 0.2;   // EuclideanClusterExtraction distance [m]
  int min_cluster_size = 20;        // in points after downsampling
  int max_cluster_size = 100000;
  double max_dimension = 0.0;       // drop boxes with a longer side [m]; <= 0 disables
};

// False when VoxelGrid would overflow its int32 voxel indices for this cloud's extent. PCL then
// warns and returns the input unfiltered, on every frame.
bool voxel_grid_fits(const Cloud & cloud, double leaf);

// VoxelGrid with `leaf`, or the input itself when leaf <= 0.
Cloud::Ptr downsample(const Cloud::Ptr & cloud, double leaf);

std::vector<pcl::PointIndices> euclidean_clusters(const Cloud::Ptr & cloud, const ClusterParams & params);

// --------------------------------------------------------------------------------------
// Boxes
// --------------------------------------------------------------------------------------

// Box upright in its frame: rotation yaw about z, size.x >= size.y, yaw in (-pi/2, pi/2].
struct YawBox
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

// Minimum-area rectangle of the xy of the points, z extent for the height.
YawBox fit_yaw_box(const Cloud & cloud, const pcl::Indices & indices);

struct Detection
{
  int category = -1;
  YawBox box;                       // level frame
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();      // level frame
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();    // point spread, level frame
  double score = 0.0;               // mean class score of the points
  pcl::Indices indices;             // into the downsampled cloud of the class
};

struct ClassResult
{
  Cloud::Ptr cloud;                 // downsampled points of the class, level frame
  std::vector<Detection> detections;
  bool voxel_overflow = false;      // VoxelGrid would overflow: clustered without downsampling
};

// Downsample, cluster and box one class (points in the level frame).
ClassResult detect_class(const Cloud::Ptr & cloud, int category, const ClusterParams & params);

// --------------------------------------------------------------------------------------
// Frames and colours
// --------------------------------------------------------------------------------------

// Rotation sensor -> level frame: the sensor orientation in world (z up), or identity.
Eigen::Matrix3d level_rotation(const Eigen::Quaterniond & world_q_sensor, bool gravity_align);

// A level-frame box or point expressed in the sensor frame (R = level_rotation).
struct SensorPose
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
SensorPose box_in_sensor(const YawBox & box, const Eigen::Matrix3d & R);
Eigen::Matrix3d covariance_in_sensor(const Eigen::Matrix3d & covariance, const Eigen::Matrix3d & R);

// 0xRRGGBB of category index i, as the Voxeland map draws it (SemanticMap::indexToHexColor;
// same table as VOXELAND_PALETTE / class_colors() in ninsar_mapping/voxel_store.py).
uint32_t class_color(size_t category);

}  // namespace ninsar_perception
