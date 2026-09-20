// C++ port of scripts/ground_segmentation_node.py
// Node name: ground_segmentation
//
// Depth point-cloud ground segmentation + voxel density filtering +
// temporal consistency filtering -> /obstacle_points, optional
// /depth/clearing_points (all measured endpoints including floor).
//
// Ground classification happens in the target_frame (base_footprint);
// temporal matching happens in the temporal_frame (odom).  Output cloud keeps
// the original sensor frame and (sensor or receive) stamp.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
// ---------------------------------------------------------------------------
// Read XYZ from a PointCloud2, respecting endianness, field offsets and
// organized row padding.  Mirrors read_xyz() in the Python node:
//  - organized (height > 1): sample every `stride` row and column
//  - unorganized (height == 1): sample every stride*stride-th point
bool read_xyz(const sensor_msgs::PointCloud2& msg, int stride,
              std::vector<Eigen::Vector3d>& out, std::vector<double>& ranges_sq)
{
  out.clear();
  ranges_sq.clear();
  if (msg.width == 0 || msg.height == 0)
    return true;

  stride = std::max(1, stride);
  const std::string names[3] = { "x", "y", "z" };
  int offsets[3] = { 0, 0, 0 };
  int datatypes[3] = { 0, 0, 0 };
  for (int k = 0; k < 3; ++k)
  {
    bool found = false;
    for (size_t f = 0; f < msg.fields.size(); ++f)
    {
      if (msg.fields[f].name == names[k])
      {
        if (msg.fields[f].count != 1 ||
            (msg.fields[f].datatype != sensor_msgs::PointField::FLOAT32 &&
             msg.fields[f].datatype != sensor_msgs::PointField::FLOAT64))
          throw std::runtime_error("XYZ fields must be scalar float32/float64");
        offsets[k] = static_cast<int>(msg.fields[f].offset);
        datatypes[k] = static_cast<int>(msg.fields[f].datatype);
        found = true;
        break;
      }
    }
    if (!found)
      throw std::runtime_error("PointCloud2 missing field " + names[k]);
  }

  const bool big = msg.is_bigendian;
  const uint32_t point_step = msg.point_step;
  const uint32_t width = msg.width;
  const uint8_t* data = msg.data.empty() ? NULL : &msg.data[0];
  const uint64_t total = static_cast<uint64_t>(msg.width) * msg.height;
  const size_t eff = (msg.height > 1) ? static_cast<size_t>(stride)
                                      : static_cast<size_t>(stride * stride);

  auto read_value = [&](const uint8_t* p, int offset, int dtype, double& v) {
    if (dtype == sensor_msgs::PointField::FLOAT32)
    {
      float f;
      std::memcpy(&f, p + offset, 4);
      if (big)
      {
        uint32_t u;
        std::memcpy(&u, &f, 4);
        u = ((u & 0xFF) << 24) | ((u & 0xFF00) << 8) | ((u >> 8) & 0xFF00) | ((u >> 24) & 0xFF);
        std::memcpy(&f, &u, 4);
      }
      v = f;
    }
    else  // FLOAT64
    {
      double d;
      std::memcpy(&d, p + offset, 8);
      if (big)
      {
        uint64_t u;
        std::memcpy(&u, &d, 8);
        u = ((u & 0xFFULL) << 56) | ((u & 0xFF00ULL) << 40) | ((u & 0xFF0000ULL) << 24) |
            ((u & 0xFF000000ULL) << 8) | ((u >> 8) & 0xFF000000ULL) |
            ((u >> 24) & 0xFF0000ULL) | ((u >> 40) & 0xFF00ULL) | ((u >> 56) & 0xFFULL);
        std::memcpy(&d, &u, 8);
      }
      v = d;
    }
  };

  for (uint64_t idx = 0; idx < total; ++idx)
  {
    bool keep;
    if (msg.height > 1)
    {
      uint64_t row = idx / width;
      uint64_t col = idx % width;
      keep = (row % stride == 0) && (col % stride == 0);
    }
    else
    {
      keep = (idx % eff == 0);
    }
    if (!keep)
      continue;
    if (data == NULL)
      continue;

    const uint8_t* p = data + idx * point_step;
    double x, y, z;
    read_value(p, offsets[0], datatypes[0], x);
    read_value(p, offsets[1], datatypes[1], y);
    read_value(p, offsets[2], datatypes[2], z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;
    out.emplace_back(x, y, z);
    ranges_sq.push_back(x * x + y * y + z * z);
  }
  return true;
}

Eigen::Matrix3d rotation_from_quat(double x, double y, double z, double w)
{
  double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm < 1e-9)
    throw std::runtime_error("Invalid zero TF quaternion");
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  Eigen::Matrix3d R;
  R << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
      2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
      2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y;
  return R;
}

void transform_xyz(const std::vector<Eigen::Vector3d>& in,
                   const geometry_msgs::TransformStamped& tf,
                   std::vector<Eigen::Vector3d>& out)
{
  out.resize(in.size());
  Eigen::Matrix3d R = rotation_from_quat(
      tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
      tf.transform.rotation.w);
  Eigen::Vector3d t(tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.translation.z);
  for (size_t i = 0; i < in.size(); ++i)
    out[i] = R * in[i] + t;
}

double percentile_z(const std::vector<Eigen::Vector3d>& pts, double q)
{
  std::vector<double> zs;
  zs.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    zs.push_back(pts[i].z());
  std::sort(zs.begin(), zs.end());
  size_t n = zs.size();
  double pos = q * (n - 1);
  size_t lo = static_cast<size_t>(std::floor(pos));
  size_t hi = static_cast<size_t>(std::ceil(pos));
  double frac = pos - lo;
  return zs[lo] + frac * (zs[hi] - zs[lo]);
}

// Constrained robust plane fit (tables/walls cannot become the floor).
std::vector<bool> ground_mask(const std::vector<Eigen::Vector3d>& pts, double search_height,
                              double distance, double max_tilt_deg, double max_offset,
                              int max_fit_points)
{
  std::vector<bool> fallback(pts.size(), false);
  for (size_t i = 0; i < pts.size(); ++i)
    if (std::fabs(pts[i].z()) <= distance)
      fallback[i] = true;

  std::vector<Eigen::Vector3d> cand;
  cand.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    if (pts[i].z() > -search_height && pts[i].z() < search_height)
      cand.push_back(pts[i]);

  if (max_fit_points > 0 && static_cast<int>(cand.size()) > max_fit_points)
  {
    int fit_stride = static_cast<int>(std::ceil(static_cast<double>(cand.size()) / max_fit_points));
    std::vector<Eigen::Vector3d> sub;
    sub.reserve(cand.size() / fit_stride + 1);
    for (size_t i = 0; i < cand.size(); i += fit_stride)
      sub.push_back(cand[i]);
    cand.swap(sub);
  }
  if (cand.size() < 30)
    return fallback;

  double seed_z = percentile_z(cand, 0.2);
  std::vector<Eigen::Vector3d> c2;
  c2.reserve(cand.size());
  for (size_t i = 0; i < cand.size(); ++i)
    if (std::fabs(cand[i].z() - seed_z) < 2 * distance)
      c2.push_back(cand[i]);
  cand.swap(c2);

  Eigen::Vector3d normal;
  double offset = 0.0;
  for (int it = 0; it < 3; ++it)
  {
    if (cand.size() < 20)
      return fallback;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < cand.size(); ++i)
      centroid += cand[i];
    centroid /= static_cast<double>(cand.size());

    Eigen::MatrixXd A(cand.size(), 3);
    for (size_t i = 0; i < cand.size(); ++i)
      A.row(i) = cand[i] - centroid;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    if (svd.singularValues().size() < 2)
      return fallback;
    if (svd.singularValues()[1] < 0.05)
      return fallback;
    normal = svd.matrixV().col(2);
    if (normal[2] < 0)
      normal = -normal;
    offset = -centroid.dot(normal);
    if (normal[2] < std::cos(max_tilt_deg * M_PI / 180.0) || std::fabs(offset) > max_offset)
      return fallback;

    std::vector<Eigen::Vector3d> c3;
    c3.reserve(cand.size());
    for (size_t i = 0; i < cand.size(); ++i)
      if (std::fabs(cand[i].dot(normal) + offset) < distance)
        c3.push_back(cand[i]);
    cand.swap(c3);
  }

  std::vector<bool> mask(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    mask[i] = std::fabs(pts[i].dot(normal) + offset) < distance;
  return mask;
}

// Voxel-density filter on the XY plane: keep points whose voxel holds at
// least min_points points (matches spatial_mask in the Python node).
std::vector<bool> spatial_mask(const std::vector<Eigen::Vector3d>& pts, double voxel_size,
                               int min_points)
{
  std::vector<bool> mask(pts.size(), false);
  if (pts.empty())
    return mask;
  typedef std::pair<long long, long long> Cell;
  std::map<Cell, int> counts;
  std::vector<Cell> cells(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
  {
    cells[i] = Cell(static_cast<long long>(std::floor(pts[i].x() / voxel_size)),
                    static_cast<long long>(std::floor(pts[i].y() / voxel_size)));
    counts[cells[i]]++;
  }
  for (size_t i = 0; i < pts.size(); ++i)
    mask[i] = counts[cells[i]] >= min_points;
  return mask;
}

// ---------------------------------------------------------------------------
class TemporalFilter
{
public:
  typedef std::pair<long long, long long> Cell;

  TemporalFilter() : hits_(1), cell_size_(0.10), history_(1)
  {
  }

  TemporalFilter(int hits, double cell_size)
    : hits_(std::max(1, hits)), cell_size_(cell_size),
      history_(std::max(1, hits - 1))
  {
  }

  int hits() const { return hits_; }
  double cell_size() const { return cell_size_; }
  void clear() { history_.clear(); }

  std::vector<bool> mask(const std::vector<Eigen::Vector3d>& pts)
  {
    std::vector<Cell> cells(pts.size());
    std::set<Cell> current;
    for (size_t i = 0; i < pts.size(); ++i)
    {
      cells[i] = Cell(static_cast<long long>(std::floor(pts[i].x() / cell_size_)),
                      static_cast<long long>(std::floor(pts[i].y() / cell_size_)));
      current.insert(cells[i]);
    }

    std::vector<bool> keep(pts.size(), true);
    if (hits_ > 1)
    {
      if (static_cast<int>(history_.size()) < hits_ - 1)
      {
        std::fill(keep.begin(), keep.end(), false);
      }
      else
      {
        std::set<Cell> allowed = current;
        for (size_t h = 0; h < history_.size(); ++h)
        {
          std::set<Cell> neigh;
          for (std::set<Cell>::const_iterator it = history_[h].begin(); it != history_[h].end(); ++it)
          {
            for (int dx = -1; dx <= 1; ++dx)
              for (int dy = -1; dy <= 1; ++dy)
                neigh.insert(Cell(it->first + dx, it->second + dy));
          }
          std::set<Cell> inter;
          std::set_intersection(allowed.begin(), allowed.end(), neigh.begin(), neigh.end(),
                                std::inserter(inter, inter.end()));
          allowed.swap(inter);
        }
        for (size_t i = 0; i < pts.size(); ++i)
          keep[i] = allowed.count(cells[i]) > 0;
      }
    }
    history_.push_back(current);
    return keep;
  }

private:
  int hits_;
  double cell_size_;
  std::deque<std::set<Cell> > history_;
};

// ---------------------------------------------------------------------------
sensor_msgs::PointCloud2 make_cloud(const std_msgs::Header& header,
                                    const std::vector<Eigen::Vector3d>& pts)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = pts.size();
  cloud.fields.resize(3);
  const char* names[3] = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i)
  {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = 4 * i;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = 12 * pts.size();
  cloud.data.resize(12 * pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
  {
    float* p = reinterpret_cast<float*>(&cloud.data[12 * i]);
    p[0] = static_cast<float>(pts[i].x());
    p[1] = static_cast<float>(pts[i].y());
    p[2] = static_cast<float>(pts[i].z());
  }
  cloud.is_dense = true;
  return cloud;
}
}  // namespace

// ---------------------------------------------------------------------------
class GroundSegmentationNode
{
public:
  GroundSegmentationNode()
    : nh_(), pnh_("~"), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_),
      temporal_(1, 0.10), last_stamp_(ros::Time(0))
  {
    input_topic_ = pnh_.param<std::string>("input_pointcloud_topic",
                                           "/ascamera_hp60c/depth0/points");
    ground_frame_ = pnh_.param<std::string>("target_frame", "base_footprint");
    fixed_frame_ = pnh_.param<std::string>("temporal_frame", "odom");
    height_min_ = pnh_.param("height_min", 0.05);
    height_max_ = pnh_.param("height_max", 1.9);
    max_range_ = pnh_.param("max_range", 2.5);
    clearing_range_ = pnh_.param("clearing_range", 5.0);
    min_range_ = pnh_.param("min_range", 0.15);
    stride_ = std::max(1, pnh_.param("pixel_stride", 4));
    voxel_size_ = pnh_.param("voxel_size", 0.05);
    min_points_ = std::max(1, pnh_.param("min_points_per_voxel", 2));
    rate_ = pnh_.param("max_rate", 10.0);
    tf_timeout_ = pnh_.param("transform_timeout", 0.3);
    max_age_ = pnh_.param("max_cloud_age", 1.5);
    publish_clearing_ = pnh_.param("publish_clearing_points", false);
    clearing_stride_ = std::max(1, pnh_.param("clearing_pixel_stride", 8));
    stamp_mode_ = pnh_.param<std::string>("stamp_mode", "sensor");
    ground_distance_ = pnh_.param("distance_threshold", 0.025);
    plane_fit_max_points_ = std::max(0, pnh_.param("plane_fit_max_points", 5000));
    ground_search_ = pnh_.param("ground_search_height", 0.10);
    int temporal_min_hits = std::max(1, pnh_.param("temporal_min_hits", 1));
    double temporal_cell_size = pnh_.param("temporal_cell_size", 0.10);
    temporal_ = TemporalFilter(temporal_min_hits, temporal_cell_size);

    bool valid = rate_ > 0 && stride_ >= 1 && voxel_size_ > 0 && min_points_ >= 1 &&
                 temporal_.hits() >= 1 && temporal_.cell_size() > 0 &&
                 height_max_ > height_min_ && clearing_range_ >= max_range_ &&
                 max_range_ > min_range_;
    if (!valid || (stamp_mode_ != "sensor" && stamp_mode_ != "receive"))
    {
      ROS_FATAL("Invalid depth obstacle filter parameters");
      ros::shutdown();
      return;
    }

    last_success_ = std::chrono::steady_clock::now();

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1);
    if (publish_clearing_)
      clear_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/depth/clearing_points", 1);

    ros::SubscribeOptions so;
    so.template init<sensor_msgs::PointCloud2>(
        input_topic_, 1, boost::bind(&GroundSegmentationNode::input_callback, this, _1));
    sub_ = nh_.subscribe(so);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate_, 0.01)),
                             &GroundSegmentationNode::process_latest, this);
    watchdog_ = nh_.createTimer(ros::Duration(2.0),
                                &GroundSegmentationNode::check_input, this);

    ROS_INFO("Depth fusion: ground=%s temporal=%s output=sensor frame; stamp=%s",
             ground_frame_.c_str(), fixed_frame_.c_str(), stamp_mode_.c_str());
  }

private:
  void input_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lk(latest_lock_);
    latest_cloud_ = msg;
  }

  void check_input(const ros::TimerEvent&)
  {
    double since = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - last_success_)
                       .count();
    if (since > 2.0)
      ROS_WARN_THROTTLE(10.0,
                        "No valid depth frame for >2s (%.0fs total, %zu dropped): "
                        "check camera, timestamps and TF", since, tf_drops_);
  }

  // Try ground classification in a cascade of world frames.  map->base is the
  // preferred target because it is globally consistent, but it depends on the
  // localization TF; when that briefly stalls we fall back to a robot-/odom-
  // anchored frame so the depth safety gate does not halt the robot for the
  // whole outage.  Every lookup stays pinned to the sensor `stamp` (strict,
  // never Time(0)) to keep ground/obstacle geometry lock-step with the scan.
  bool lookup_ground_tf(const std::string& cam, const ros::Time& stamp,
                        geometry_msgs::TransformStamped& out)
  {
    const char* cands[3] = { NULL, "base_footprint", "odom" };
    cands[0] = ground_frame_.c_str();
    for (int i = 0; i < 3; ++i)
    {
      try
      {
        out = tf_buffer_.lookupTransform(cands[i], cam, stamp,
                                         ros::Duration(tf_timeout_));
        bool fb = (cands[i] != ground_frame_);
        if (fb && !ground_fallback_)
          ROS_WARN_THROTTLE(30.0,
                            "Map TF unavailable; classifying ground in %s "
                            "(strict stamps, sensor frame preserved)",
                            cands[i]);
        ground_fallback_ = fb;
        frame_desc_ = cands[i];
        return true;
      }
      catch (const tf2::TransformException&)
      {
        // Try the next candidate frame.
      }
    }
    // No candidate published at the strict sensor stamp -> frame is a real loss.
    ground_fallback_ = false;
    frame_desc_ = ground_frame_;
    return false;
  }

  void process_latest(const ros::TimerEvent&)
  {
    sensor_msgs::PointCloud2ConstPtr msg;
    {
      std::lock_guard<std::mutex> lk(latest_lock_);
      msg = latest_cloud_;
      latest_cloud_.reset();
    }
    if (!msg)
      return;

    ros::Time arrival = ros::Time::now();
    ros::Time stamp = msg->header.stamp;
    if (stamp_mode_ == "receive")
    {
      stamp = arrival;
      ROS_WARN_THROTTLE(30.0, "Using receive timestamps; fix camera clock for motion accuracy");
    }
    double age = (arrival - stamp).toSec();
    if (msg->header.frame_id.empty() || stamp == ros::Time(0) || age > max_age_ || age < -0.1)
    {
      temporal_.clear();
      ROS_WARN_THROTTLE(5.0, "Dropped depth frame: invalid frame/stamp, age=%.3fs", age);
      return;
    }
    if (last_stamp_ != ros::Time(0))
    {
      double dt = (stamp - last_stamp_).toSec();
      if (dt <= 0 || dt > max_age_)
        temporal_.clear();
      if (dt <= 0)
      {
        last_stamp_ = stamp;
        return;
      }
    }
    last_stamp_ = stamp;

    try
    {
      std::vector<Eigen::Vector3d> xyz;
      std::vector<double> ranges_sq;
      read_xyz(*msg, stride_, xyz, ranges_sq);

      // Range filter (measured endpoints only).
      double min_r2 = min_range_ * min_range_;
      double clear_r2 = clearing_range_ * clearing_range_;
      std::vector<Eigen::Vector3d> pts;
      std::vector<double> dist_sq;
      pts.reserve(xyz.size());
      dist_sq.reserve(xyz.size());
      for (size_t i = 0; i < xyz.size(); ++i)
      {
        if (ranges_sq[i] >= min_r2 && ranges_sq[i] <= clear_r2)
        {
          pts.push_back(xyz[i]);
          dist_sq.push_back(ranges_sq[i]);
        }
      }
      if (pts.empty())
      {
        temporal_.clear();
        ROS_WARN_THROTTLE(5.0,
                          "Depth contains no valid measured endpoints; not declaring free space");
        return;
      }

      geometry_msgs::TransformStamped ground_tf;
      if (!lookup_ground_tf(msg->header.frame_id, stamp, ground_tf))
      {
        ++tf_drops_;
        temporal_.clear();
        ROS_WARN_THROTTLE(5.0,
                          "Dropped depth frame: no TF at strict stamp %.3fs "
                          "from %s/system (%s), consecutive drops=%zu", stamp.toSec(),
                          msg->header.frame_id.c_str(), frame_desc_.c_str(), tf_drops_);
        return;
      }

      geometry_msgs::TransformStamped fixed_tf = ground_tf;
      if (temporal_.hits() > 1)
      {
        try
        {
          fixed_tf = tf_buffer_.lookupTransform(fixed_frame_, msg->header.frame_id, stamp,
                                                ros::Duration(tf_timeout_));
        }
        catch (const tf2::TransformException&)
        {
          ROS_WARN_THROTTLE(10.0, "Temporal TF %s unavailable; matching in %s",
                            fixed_frame_.c_str(), frame_desc_.c_str());
        }
      }

      std::vector<Eigen::Vector3d> base;
      transform_xyz(pts, ground_tf, base);
      std::vector<bool> is_ground =
          ground_mask(base, ground_search_, ground_distance_, 15.0, 0.04, plane_fit_max_points_);

      std::vector<size_t> indices;
      indices.reserve(pts.size());
      double max_r2 = max_range_ * max_range_;
      for (size_t i = 0; i < pts.size(); ++i)
      {
        if (!is_ground[i] && base[i].z() >= height_min_ && base[i].z() <= height_max_ &&
            dist_sq[i] <= max_r2)
          indices.push_back(i);
      }
      int n_candidates = static_cast<int>(indices.size());

      // Voxel density filtering (over the base-frame XY cells).
      std::vector<Eigen::Vector3d> base_sel;
      base_sel.reserve(indices.size());
      for (size_t k = 0; k < indices.size(); ++k)
        base_sel.push_back(base[indices[k]]);
      std::vector<bool> sp = spatial_mask(base_sel, voxel_size_, min_points_);
      std::vector<size_t> keep;
      keep.reserve(indices.size());
      for (size_t k = 0; k < indices.size(); ++k)
        if (sp[k])
          keep.push_back(indices[k]);
      int n_voxel = static_cast<int>(keep.size());

      // Keep a single representative per voxel (first occurrence).
      if (!keep.empty())
      {
        typedef std::pair<long long, long long> Cell;
        std::map<Cell, bool> seen;
        std::vector<size_t> sel;
        sel.reserve(keep.size());
        for (size_t k = 0; k < keep.size(); ++k)
        {
          Cell c(static_cast<long long>(std::floor(base[keep[k]].x() / voxel_size_)),
                 static_cast<long long>(std::floor(base[keep[k]].y() / voxel_size_)));
          if (seen.count(c) == 0)
          {
            seen[c] = true;
            sel.push_back(keep[k]);
          }
        }
        keep.swap(sel);
      }

      // Temporal consistency on the representatives.
      std::vector<Eigen::Vector3d> fixed;
      {
        std::vector<Eigen::Vector3d> sel_pts;
        sel_pts.reserve(keep.size());
        for (size_t k = 0; k < keep.size(); ++k)
          sel_pts.push_back(pts[keep[k]]);
        transform_xyz(sel_pts, fixed_tf, fixed);
      }
      std::vector<bool> tmp = temporal_.mask(fixed);
      std::vector<size_t> final_idx;
      final_idx.reserve(keep.size());
      for (size_t k = 0; k < keep.size(); ++k)
        if (tmp[k])
          final_idx.push_back(keep[k]);

      std_msgs::Header header;
      header.stamp = stamp;
      header.frame_id = msg->header.frame_id;

      if (clear_pub_)
      {
        // All measured endpoints (including floor) for 3-D clearing rays.
        std::vector<Eigen::Vector3d> clear_pts;
        clear_pts.reserve(pts.size() / clearing_stride_ + 1);
        for (size_t i = 0; i < pts.size(); i += clearing_stride_)
          clear_pts.push_back(pts[i]);
        clear_pub_.publish(make_cloud(header, clear_pts));
      }

      std::vector<Eigen::Vector3d> obstacle_pts;
      obstacle_pts.reserve(final_idx.size());
      for (size_t k = 0; k < final_idx.size(); ++k)
        obstacle_pts.push_back(pts[final_idx[k]]);
      pub_.publish(make_cloud(header, obstacle_pts));

      last_success_ = std::chrono::steady_clock::now();
      tf_drops_ = 0;
      ROS_INFO_THROTTLE(5.0,
                        "Depth valid=%zu ground=%d candidates=%d voxel=%d marked=%zu age=%.3fs (%s)",
                        pts.size(),
                        static_cast<int>(std::count(is_ground.begin(), is_ground.end(), true)),
                        n_candidates, n_voxel, obstacle_pts.size(), age, frame_desc_.c_str());
    }
    catch (const tf2::TransformException& ex)
    {
      ++tf_drops_;
      temporal_.clear();
      ROS_WARN_THROTTLE(5.0,
                        "Dropped depth frame: TF at acquisition time unavailable "
                        "(consecutive drops=%zu): %s", tf_drops_, ex.what());
    }
    catch (const std::exception& ex)
    {
      temporal_.clear();
      ROS_WARN_THROTTLE(10.0, "Dropped malformed depth cloud: %s", ex.what());
    }
  }

  // ---- params ----
  std::string input_topic_, ground_frame_, fixed_frame_, stamp_mode_;
  double height_min_, height_max_, max_range_, clearing_range_, min_range_;
  int stride_;
  double voxel_size_;
  int min_points_;
  double rate_, tf_timeout_, max_age_;
  bool publish_clearing_;
  int clearing_stride_;
  double ground_distance_;
  int plane_fit_max_points_;
  double ground_search_;
  TemporalFilter temporal_;

  // ---- state ----
  ros::Time last_stamp_;
  std::chrono::steady_clock::time_point last_success_;
  std::mutex latest_lock_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  // Consecutive frames dropped because no usable TF was available.  Reset to 0
  // on the first successful classification so the counter reflects the current
  // drop streak instead of an ever-growing total.
  std::size_t tf_drops_ = 0;
  // Frame actually used for ground classification (== ground_frame_ normally,
  // falls back to base_footprint/odom during a map->base TF outage, always at
  // the same strict sensor stamp).
  std::string frame_desc_;
  // True while classifying in a fallback frame instead of the configured target.
  bool ground_fallback_ = false;

  // ---- ros handles ----
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber sub_;
  ros::Publisher pub_, clear_pub_;
  ros::Timer timer_, watchdog_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_segmentation");
  GroundSegmentationNode node;
  ros::spin();
  return 0;
}
