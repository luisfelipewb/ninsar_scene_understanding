#include "ninsar_perception/clustering.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ninsar_perception
{

// SemanticMap::color_palette, voxeland_map/src/semantics.cpp:6 (0xRRGGBB)
static const uint32_t kVoxelandPalette[] = {
  0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF,
  0xE6194B, 0x3CB44B, 0xFFE119, 0x4363D8, 0xF58231, 0x911EB4, 0x42D4F4, 0xF032E6,
  0xBFEF45, 0xFABED4, 0x469990, 0xDCBEFF, 0x9A6324, 0xFFFAC8,
};
static const uint32_t kIndex0Grey = 0xBCBCBC;  // indexToHexColor(0)

// --------------------------------------------------------------------------------------
// Input decoding
// --------------------------------------------------------------------------------------

std::vector<InstanceClass> decode_instance_table(
  const std::vector<vision_msgs::msg::Detection2D> & instances,
  const std::vector<std::string> & categories)
{
  std::map<std::string, int> index;
  for (size_t k = 0; k < categories.size(); ++k) {
    if (categories[k] != kUnknown) {
      index.emplace(categories[k], static_cast<int>(k));
    }
  }

  std::vector<InstanceClass> table;
  for (const auto & det : instances) {
    const int i = std::atoi(det.id.c_str());  // as the server: atoi(instances[i].id)
    if (i < 0) {
      continue;
    }
    if (static_cast<size_t>(i) >= table.size()) {
      table.resize(i + 1);
    }
    InstanceClass best;
    for (const auto & hyp : det.results) {
      const auto it = index.find(hyp.hypothesis.class_id);
      if (it != index.end() && (best.category < 0 || hyp.hypothesis.score > best.score)) {
        best.category = it->second;
        best.score = static_cast<float>(hyp.hypothesis.score);
      }
    }
    table[i] = best;
  }
  return table;
}

static void require_field(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, uint8_t datatype)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      if (f.datatype != datatype) {
        throw std::runtime_error("field '" + name + "' has datatype " + std::to_string(f.datatype) +
                                 ", expected " + std::to_string(datatype));
      }
      return;
    }
  }
  throw std::runtime_error("cloud has no field '" + name + "'");
}

std::map<int, Cloud::Ptr> split_by_class(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<InstanceClass> & table,
  const SplitOptions & options, SplitStats * stats)
{
  using sensor_msgs::msg::PointField;
  require_field(cloud, "x", PointField::FLOAT32);
  require_field(cloud, "y", PointField::FLOAT32);
  require_field(cloud, "z", PointField::FLOAT32);
  require_field(cloud, "instance_id", PointField::INT32);

  SplitStats local;
  SplitStats & s = stats ? *stats : local;
  s = SplitStats();

  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  s.points = n;
  const float max_r2 = static_cast<float>(options.max_range * options.max_range);
  const float min_score = static_cast<float>(options.min_score);

  std::vector<Cloud::Ptr> by_category(options.keep_category.size());
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<int32_t> it_id(cloud, "instance_id");
  for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z, ++it_id) {
    const Eigen::Vector3f p(*it_x, *it_y, *it_z);
    if (!p.allFinite()) {
      ++s.non_finite;
      continue;
    }
    if (p.squaredNorm() > max_r2) {
      ++s.far;
      continue;
    }
    const int32_t id = *it_id;
    if (id < 0 || static_cast<size_t>(id) >= table.size()) {
      ++s.bad_instance;
      continue;
    }
    const InstanceClass & ic = table[id];
    if (ic.category < 0 || static_cast<size_t>(ic.category) >= by_category.size() ||
        !options.keep_category[ic.category] || ic.score < min_score)
    {
      continue;
    }
    Cloud::Ptr & out = by_category[ic.category];
    if (!out) {
      out = std::make_shared<Cloud>();
    }
    const Eigen::Vector3f q = options.rotation * p;
    Point pt;
    pt.x = q.x();
    pt.y = q.y();
    pt.z = q.z();
    pt.intensity = ic.score;
    out->push_back(pt);
  }

  std::map<int, Cloud::Ptr> result;
  for (size_t k = 0; k < by_category.size(); ++k) {
    if (by_category[k]) {
      result.emplace(static_cast<int>(k), std::move(by_category[k]));
    }
  }
  return result;
}

// --------------------------------------------------------------------------------------
// Clustering
// --------------------------------------------------------------------------------------

bool voxel_grid_fits(const Cloud & cloud, double leaf)
{
  if (cloud.empty() || leaf <= 0.0) {
    return true;
  }
  Eigen::Vector4f lo, hi;
  pcl::getMinMax3D(cloud, lo, hi);
  // same arithmetic as pcl::VoxelGrid::applyFilter
  const double inv = 1.0 / leaf;
  int64_t cells = 1;
  for (int a = 0; a < 3; ++a) {
    cells *= static_cast<int64_t>(hi[a] * inv) - static_cast<int64_t>(lo[a] * inv) + 1;
  }
  return cells <= static_cast<int64_t>(std::numeric_limits<int32_t>::max());
}

Cloud::Ptr downsample(const Cloud::Ptr & cloud, double leaf)
{
  if (leaf <= 0.0 || cloud->empty()) {
    return cloud;
  }
  pcl::VoxelGrid<Point> grid;  // averages all fields, so intensity = mean class score of the voxel
  grid.setInputCloud(cloud);
  const float l = static_cast<float>(leaf);
  grid.setLeafSize(l, l, l);
  auto out = std::make_shared<Cloud>();
  grid.filter(*out);
  return out;
}

std::vector<pcl::PointIndices> euclidean_clusters(const Cloud::Ptr & cloud, const ClusterParams & params)
{
  std::vector<pcl::PointIndices> clusters;
  if (cloud->empty()) {
    return clusters;
  }
  auto tree = std::make_shared<pcl::search::KdTree<Point>>();
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(params.cluster_tolerance);
  ec.setMinClusterSize(params.min_cluster_size);
  ec.setMaxClusterSize(params.max_cluster_size);  // larger clusters are dropped, not split
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
  return clusters;
}

// --------------------------------------------------------------------------------------
// Boxes
// --------------------------------------------------------------------------------------

YawBox fit_yaw_box(const Cloud & cloud, const pcl::Indices & indices)
{
  YawBox box;
  if (indices.empty()) {
    return box;
  }
  std::vector<cv::Point2f> xy;
  xy.reserve(indices.size());
  float z_lo = std::numeric_limits<float>::max(), z_hi = std::numeric_limits<float>::lowest();
  for (const auto i : indices) {
    const Point & p = cloud[i];
    xy.emplace_back(p.x, p.y);
    z_lo = std::min(z_lo, p.z);
    z_hi = std::max(z_hi, p.z);
  }

  // Yaw from the rectangle's edges rather than RotatedRect::angle, whose range changed in OpenCV 4.5.
  const cv::RotatedRect rect = cv::minAreaRect(xy);
  cv::Point2f v[4];
  rect.points(v);
  Eigen::Vector2d long_edge(v[1].x - v[0].x, v[1].y - v[0].y);
  Eigen::Vector2d short_edge(v[2].x - v[1].x, v[2].y - v[1].y);
  if (short_edge.norm() > long_edge.norm()) {
    std::swap(long_edge, short_edge);
  }
  double yaw = std::atan2(long_edge.y(), long_edge.x());
  if (yaw <= -M_PI / 2) {
    yaw += M_PI;
  } else if (yaw > M_PI / 2) {
    yaw -= M_PI;
  }

  box.center = Eigen::Vector3d(rect.center.x, rect.center.y, 0.5 * (z_lo + z_hi));
  box.size = Eigen::Vector3d(long_edge.norm(), short_edge.norm(), z_hi - z_lo);
  box.yaw = yaw;
  return box;
}

ClassResult detect_class(const Cloud::Ptr & cloud, int category, const ClusterParams & params)
{
  ClassResult result;
  const bool fits = voxel_grid_fits(*cloud, params.voxel_size);
  result.voxel_overflow = !fits;
  result.cloud = downsample(cloud, fits ? params.voxel_size : 0.0);

  for (auto & cluster : euclidean_clusters(result.cloud, params)) {
    Detection det;
    det.category = category;
    det.box = fit_yaw_box(*result.cloud, cluster.indices);
    if (params.max_dimension > 0.0 && det.box.size.maxCoeff() > params.max_dimension) {
      continue;
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    Eigen::Vector4d mean = Eigen::Vector4d::Zero();
    pcl::computeMeanAndCovarianceMatrix(*result.cloud, cluster.indices, cov, mean);
    det.centroid = mean.head<3>();
    det.covariance = cov;
    double score = 0.0;
    for (const auto i : cluster.indices) {
      score += (*result.cloud)[i].intensity;
    }
    det.score = score / static_cast<double>(cluster.indices.size());
    det.indices = std::move(cluster.indices);
    result.detections.push_back(std::move(det));
  }
  return result;
}

// --------------------------------------------------------------------------------------
// Frames and colours
// --------------------------------------------------------------------------------------

Eigen::Matrix3d level_rotation(const Eigen::Quaterniond & world_q_sensor, bool gravity_align)
{
  if (!gravity_align || world_q_sensor.norm() < 1e-9) {
    return Eigen::Matrix3d::Identity();
  }
  return world_q_sensor.normalized().toRotationMatrix();
}

SensorPose box_in_sensor(const YawBox & box, const Eigen::Matrix3d & R)
{
  SensorPose pose;
  pose.position = R.transpose() * box.center;
  pose.orientation = Eigen::Quaterniond(
    R.transpose() * Eigen::AngleAxisd(box.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix());
  pose.orientation.normalize();
  return pose;
}

Eigen::Matrix3d covariance_in_sensor(const Eigen::Matrix3d & covariance, const Eigen::Matrix3d & R)
{
  return R.transpose() * covariance * R;
}

uint32_t class_color(size_t category)
{
  constexpr size_t n = sizeof(kVoxelandPalette) / sizeof(kVoxelandPalette[0]);
  return category == 0 ? kIndex0Grey : kVoxelandPalette[category % n];
}

}  // namespace ninsar_perception
