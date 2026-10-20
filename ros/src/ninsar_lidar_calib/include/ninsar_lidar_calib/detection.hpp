// What one scan contributes to the calibration, and how it is logged.
#pragma once

#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <string>

namespace ninsar_lidar_calib
{

/// Why a scan produced no usable detection. Reported in the status log so the
/// operator can tell which parameter to change.
enum class Reject
{
  kNone,
  kNoCloud,          ///< empty or unreadable cloud
  kNoPose,           ///< map -> base_link not available at this stamp
  kRoiEmpty,         ///< the ROI box contains too few points
  kTooFewInliers,    ///< a RANSAC plane did not reach min_inliers
  kNotPerpendicular, ///< the three faces are not mutually orthogonal
  kBadExtent,        ///< face extents disagree with size_guess
  kDegenerate,       ///< the three planes do not intersect in a single point
};

const char * describe(Reject reason);

/// The ground plane as seen in one scan, in the lidar frame.
struct GroundObservation
{
  bool valid = false;
  Eigen::Vector3d normal{0.0, 0.0, 1.0};  ///< unit, pointing up
  /// plane: normal . p + offset = 0. With `normal` pointing up, `offset` is
  /// the sensor height above the ground.
  double offset = 0.0;
  int inliers = 0;
};

/// The corner target as seen in one scan, in the lidar frame.
struct CornerObservation
{
  bool valid = false;
  Eigen::Vector3d point{0.0, 0.0, 0.0};   ///< the corner itself
  Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();  ///< columns: face normals
  std::array<int, 3> inliers{0, 0, 0};
  std::array<double, 3> extents{0.0, 0.0, 0.0};  ///< largest span of each face
};

/// One scan's full contribution.
struct Detection
{
  int64_t stamp_ns = 0;
  Eigen::Isometry3d world_from_base = Eigen::Isometry3d::Identity();  ///< RTK pose
  GroundObservation ground;
  CornerObservation corner;
  Reject reason = Reject::kNone;

  bool usable() const { return ground.valid || corner.valid; }
};

/// Appends detections to a JSONL file, one compact object per line.
///
/// Deliberately not a ROS message: the Python solver reads it directly, and a
/// run's detections survive independently of any bag or node.
class DetectionLog
{
public:
  DetectionLog() = default;
  ~DetectionLog();

  /// Opens `path` for writing, truncating it. Returns false on failure.
  bool open(const std::string & path);
  void write(const Detection & detection);
  void close();

  bool isOpen() const { return file_.is_open(); }
  size_t count() const { return count_; }

private:
  std::ofstream file_;
  size_t count_ = 0;
};

}  // namespace ninsar_lidar_calib
