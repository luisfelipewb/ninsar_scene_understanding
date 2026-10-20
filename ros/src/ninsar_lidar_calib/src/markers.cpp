#include "ninsar_lidar_calib/markers.hpp"

#include "ninsar_lidar_calib/corner_detector.hpp"

#include <array>

namespace ninsar_lidar_calib
{
namespace
{

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

/// The 12 edges of a box, as index pairs into roiCorners' output ordering
/// (x outer, y middle, z inner).
constexpr std::array<std::pair<int, int>, 12> kBoxEdges{{
  {0, 1}, {2, 3}, {4, 5}, {6, 7},   // along z
  {0, 2}, {1, 3}, {4, 6}, {5, 7},   // along y
  {0, 4}, {1, 5}, {2, 6}, {3, 7},   // along x
}};

}  // namespace

visualization_msgs::msg::Marker roiMarker(
  const CornerParams & params, const std::string & base_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame;
  marker.header.stamp = stamp;
  marker.ns = "roi";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.color = color(1.0f, 0.85f, 0.1f, 0.9f);

  const auto corners = CornerDetector::roiCorners(params);
  for (const auto & edge : kBoxEdges) {
    marker.points.push_back(toPoint(corners[edge.first]));
    marker.points.push_back(toPoint(corners[edge.second]));
  }
  return marker;
}

visualization_msgs::msg::Marker groundMarker(
  const GroundObservation & ground, const std::string & lidar_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = lidar_frame;
  marker.header.stamp = stamp;
  marker.ns = "ground";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = ground.valid ? visualization_msgs::msg::Marker::ADD
                               : visualization_msgs::msg::Marker::DELETE;
  if (!ground.valid) {
    return marker;
  }

  // A thin slab lying on the detected plane, centred under the sensor.
  const Eigen::Vector3d centre = -ground.offset * ground.normal;
  const Eigen::Quaterniond q =
    Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), ground.normal);
  marker.pose.position = toPoint(centre);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
  marker.scale.x = 20.0;
  marker.scale.y = 20.0;
  marker.scale.z = 0.02;
  marker.color = color(0.2f, 0.6f, 1.0f, 0.25f);
  return marker;
}

visualization_msgs::msg::MarkerArray cornerMarkers(
  const CornerObservation & corner, const std::string & lidar_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker point;
  point.header.frame_id = lidar_frame;
  point.header.stamp = stamp;
  point.ns = "corner_point";
  point.id = 0;
  point.type = visualization_msgs::msg::Marker::SPHERE;
  point.action = corner.valid ? visualization_msgs::msg::Marker::ADD
                              : visualization_msgs::msg::Marker::DELETE;
  point.pose.orientation.w = 1.0;
  point.pose.position = toPoint(corner.point);
  point.scale.x = point.scale.y = point.scale.z = 0.12;
  point.color = color(1.0f, 0.2f, 0.2f);
  array.markers.push_back(point);

  // One arrow per face normal, in the usual x/y/z colours so a flipped or
  // permuted triad is obvious at a glance.
  const std::array<std_msgs::msg::ColorRGBA, 3> axis_colors{
    color(1.0f, 0.3f, 0.3f), color(0.3f, 1.0f, 0.3f), color(0.3f, 0.3f, 1.0f)};

  for (int i = 0; i < 3; ++i) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = lidar_frame;
    arrow.header.stamp = stamp;
    arrow.ns = "corner_axes";
    arrow.id = i;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = corner.valid ? visualization_msgs::msg::Marker::ADD
                                : visualization_msgs::msg::Marker::DELETE;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.03;   // shaft diameter
    arrow.scale.y = 0.07;   // head diameter
    arrow.scale.z = 0.10;   // head length
    arrow.color = axis_colors[i];
    if (corner.valid) {
      arrow.points.push_back(toPoint(corner.point));
      arrow.points.push_back(toPoint(corner.point + 0.5 * corner.axes.col(i)));
    }
    array.markers.push_back(arrow);
  }

  return array;
}

visualization_msgs::msg::Marker cornerTrailMarker(
  const std::vector<Eigen::Vector3d> & world_points, const std::string & world_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame;
  marker.header.stamp = stamp;
  marker.ns = "corner_trail";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
  marker.color = color(1.0f, 0.5f, 0.0f, 0.8f);
  marker.points.reserve(world_points.size());
  for (const auto & p : world_points) {
    marker.points.push_back(toPoint(p));
  }
  return marker;
}

}  // namespace ninsar_lidar_calib
