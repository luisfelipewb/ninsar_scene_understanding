#include "ninsar_lidar_calib/detection.hpp"

#include <iomanip>
#include <sstream>

namespace ninsar_lidar_calib
{
namespace
{

/// JSON array of a fixed-size double range, at full double precision so the
/// solver reads back exactly what was detected.
template <typename Iterator>
std::string jsonArray(Iterator begin, Iterator end)
{
  std::ostringstream out;
  out << std::setprecision(17);
  out << '[';
  for (Iterator it = begin; it != end; ++it) {
    if (it != begin) {
      out << ", ";
    }
    out << *it;
  }
  out << ']';
  return out.str();
}

std::string jsonMatrix(const Eigen::Matrix3d & m)
{
  // row-major, so numpy reshape(3, 3) gives the same matrix back
  std::ostringstream out;
  out << std::setprecision(17) << '[';
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (row || col) {
        out << ", ";
      }
      out << m(row, col);
    }
  }
  out << ']';
  return out.str();
}

std::string jsonIsometry(const Eigen::Isometry3d & tf)
{
  std::ostringstream out;
  out << std::setprecision(17)
      << R"({"t": )" << jsonArray(tf.translation().data(), tf.translation().data() + 3)
      << R"(, "R": )" << jsonMatrix(tf.linear()) << '}';
  return out.str();
}

}  // namespace

const char * describe(Reject reason)
{
  switch (reason) {
    case Reject::kNone:
      return "accepted";
    case Reject::kNoCloud:
      return "empty cloud";
    case Reject::kNoPose:
      return "no map->base_link pose at this stamp";
    case Reject::kRoiEmpty:
      return "ROI box holds too few points (move corner.roi_center/roi_size)";
    case Reject::kTooFewInliers:
      return "a plane did not reach min_inliers";
    case Reject::kNotPerpendicular:
      return "faces are not mutually perpendicular";
    case Reject::kBadExtent:
      return "face extents disagree with corner.size_guess";
    case Reject::kDegenerate:
      return "the three planes do not meet in one point";
  }
  return "unknown";
}

DetectionLog::~DetectionLog()
{
  close();
}

bool DetectionLog::open(const std::string & path)
{
  close();
  file_.open(path, std::ios::out | std::ios::trunc);
  count_ = 0;
  return file_.is_open();
}

void DetectionLog::write(const Detection & detection)
{
  if (!file_.is_open()) {
    return;
  }

  std::ostringstream line;
  line << std::setprecision(17);
  line << R"({"stamp_ns": )" << detection.stamp_ns
       << R"(, "world_from_base": )" << jsonIsometry(detection.world_from_base);

  if (detection.ground.valid) {
    const auto & g = detection.ground;
    line << R"(, "ground": {"normal": )"
         << jsonArray(g.normal.data(), g.normal.data() + 3)
         << R"(, "offset": )" << g.offset
         << R"(, "inliers": )" << g.inliers << '}';
  }

  if (detection.corner.valid) {
    const auto & c = detection.corner;
    line << R"(, "corner": {"point": )"
         << jsonArray(c.point.data(), c.point.data() + 3)
         << R"(, "axes": )" << jsonMatrix(c.axes)
         << R"(, "inliers": )" << jsonArray(c.inliers.begin(), c.inliers.end())
         << R"(, "extents": )" << jsonArray(c.extents.begin(), c.extents.end())
         << '}';
  }

  line << '}';
  file_ << line.str() << '\n';
  file_.flush();  // a killed live session keeps everything detected so far
  ++count_;
}

void DetectionLog::close()
{
  if (file_.is_open()) {
    file_.close();
  }
}

}  // namespace ninsar_lidar_calib
