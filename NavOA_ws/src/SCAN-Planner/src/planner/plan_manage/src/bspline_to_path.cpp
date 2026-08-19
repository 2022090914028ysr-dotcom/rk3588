#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
class BsplineToPath
{
public:
  BsplineToPath()
    : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/planning/bspline");
    pnh_.param<std::string>("output_topic", output_topic_, "/planning/path");
    pnh_.param<std::string>("frame_id", frame_id_, "odom");
    pnh_.param("sample_dt", sample_dt_, 0.05);
    pnh_.param("min_point_distance", min_point_distance_, 0.03);
    pnh_.param("flatten_z", flatten_z_, true);

    sample_dt_ = std::max(0.01, sample_dt_);
    min_point_distance_ = std::max(0.0, min_point_distance_);

    path_pub_ = nh_.advertise<nav_msgs::Path>(output_topic_, 1);
    bspline_sub_ = nh_.subscribe(input_topic_, 10, &BsplineToPath::bsplineCallback, this);

    ROS_INFO("[bspline_to_path] %s -> %s (frame=%s, dt=%.3fs)",
             input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), sample_dt_);
  }

private:
  void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
  {
    if (msg->order < 1 ||
        msg->pos_pts.size() < static_cast<size_t>(msg->order + 1) ||
        msg->knots.size() != msg->pos_pts.size() + static_cast<size_t>(msg->order) + 1)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[bspline_to_path] Invalid B-spline: order=%d, control_points=%zu, knots=%zu.",
                        msg->order, msg->pos_pts.size(), msg->knots.size());
      return;
    }

    Eigen::MatrixXd control_points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      control_points(0, i) = msg->pos_pts[i].x;
      control_points(1, i) = msg->pos_pts[i].y;
      control_points(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
      knots(i) = msg->knots[i];

    scan_planner::UniformBspline spline(control_points, msg->order, 0.1);
    spline.setKnot(knots);
    const double duration = spline.getTimeSum();
    if (!std::isfinite(duration) || duration < 0.0)
    {
      ROS_WARN_THROTTLE(1.0, "[bspline_to_path] Invalid trajectory duration: %.3f.", duration);
      return;
    }

    std::vector<Eigen::Vector3d> points;
    const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt_)));
    points.reserve(sample_count + 1);
    for (int i = 0; i <= sample_count; ++i)
    {
      const double t = (i == sample_count) ? duration : std::min(duration, i * sample_dt_);
      const Eigen::Vector3d point = spline.evaluateDeBoorT(t);
      if (!point.allFinite())
        continue;
      if (points.empty() || (point - points.back()).head<2>().norm() >= min_point_distance_ ||
          i == sample_count)
        points.push_back(point);
    }

    if (points.empty())
      return;

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
      Eigen::Vector2d direction = Eigen::Vector2d::Zero();
      if (i + 1 < points.size())
        direction = (points[i + 1] - points[i]).head<2>();
      else if (i > 0)
        direction = (points[i] - points[i - 1]).head<2>();

      const double yaw = direction.squaredNorm() > 1e-8 ?
                           std::atan2(direction.y(), direction.x()) : 0.0;
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x();
      pose.pose.position.y = points[i].y();
      pose.pose.position.z = flatten_z_ ? 0.0 : points[i].z();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.poses.push_back(pose);
    }

    path_pub_.publish(path);
    ROS_DEBUG("[bspline_to_path] Published traj_id=%ld as a path with %zu poses.",
              static_cast<long>(msg->traj_id), path.poses.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber bspline_sub_;
  ros::Publisher path_pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double sample_dt_;
  double min_point_distance_;
  bool flatten_z_;
};
}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bspline_to_path");
  BsplineToPath node;
  ros::spin();
  return 0;
}
