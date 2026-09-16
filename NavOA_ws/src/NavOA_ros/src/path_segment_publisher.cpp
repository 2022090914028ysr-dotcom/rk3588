// C++ port of scripts/path_segment_publisher.py
// Node name: path_segment_publisher (anonymous)
//
// Sits between move_base and NavOA:
//  - samples the global plan into waypoints (segment_length spacing)
//  - publishes the current waypoint to /split_goal (consumed by NavOA)
//  - loose tolerance for mid waypoints, strict tolerance for the final one
//  - navigation progress monitor: when the robot makes no progress it asks
//    move_base for a fresh global route (replan), without clearing costmaps
//    so live obstacle observations survive.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <cmath>
#include <string>
#include <vector>

class PathToGoalFollower
{
public:
  PathToGoalFollower() : nh_(), pnh_("~"), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_)
  {
    segment_length_ = pnh_.param("segment_length", 1.5);
    reach_tolerance_ = pnh_.param("reach_tolerance", 0.5);
    final_tolerance_ = pnh_.param("final_tolerance", 0.15);
    base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
    map_frame_ = pnh_.param<std::string>("map_frame", "map");

    progress_timeout_ = std::max(0.0, pnh_.param("progress_timeout", 25.0));
    progress_distance_ = std::max(0.01, pnh_.param("progress_distance", 0.20));
    replan_cooldown_ = std::max(0.0, pnh_.param("replan_cooldown", 5.0));
    max_replan_attempts_ = std::max(0, pnh_.param("max_replan_attempts", 3));
    empty_plan_threshold_ = std::max(1, pnh_.param("empty_plan_threshold", 3));
    clear_costmaps_service_ = pnh_.param<std::string>(
        "clear_costmaps_service", "/move_base/clear_costmaps");
    clear_costmaps_timeout_ = std::max(0.1, pnh_.param("clear_costmaps_timeout", 2.0));

    current_goal_idx_ = 0;
    active_path_ = false;
    has_requested_goal_ = false;
    empty_plan_count_ = 0;
    replan_attempts_ = 0;
    has_best_goal_distance_ = false;
    best_goal_distance_ = 0.0;
    last_progress_time_ = ros::Time::now();
    last_replan_time_ = ros::Time(0);
    recovery_in_progress_ = false;
    has_pending_internal_ = false;
    pending_internal_goal_stamp_ = ros::Time(0);

    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10,
                              &PathToGoalFollower::goal_reset_callback, this);
    cancel_sub_ = nh_.subscribe("/move_base/cancel", 10,
                                &PathToGoalFollower::cancel_callback, this);
    plan_sub_ = nh_.subscribe("/move_base/GlobalPlanner/plan", 10,
                              &PathToGoalFollower::path_callback, this);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/split_goal", 1);
    goal_replan_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    recovery_status_pub_ = nh_.advertise<std_msgs::String>(
        "/navoa/recovery_status", 1, true /* latched */);

    clear_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmaps_service_);

    control_timer_ = nh_.createTimer(ros::Duration(0.1), &PathToGoalFollower::control_loop, this);

    ROS_INFO("Path Segment Publisher Initialized.");
    ROS_INFO("Logic: Loose tolerance for mid-points (%.2fm), Strict for final (%.2fm).",
             reach_tolerance_, final_tolerance_);
  }

private:
  // -------------------------------------------------------------------------
  bool lookup_base_pose(double& x, double& y, double& z,
                        double& ox, double& oy, double& oz, double& ow)
  {
    try
    {
      geometry_msgs::TransformStamped ts =
          tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
      x = ts.transform.translation.x;
      y = ts.transform.translation.y;
      z = ts.transform.translation.z;
      ox = ts.transform.rotation.x;
      oy = ts.transform.rotation.y;
      oz = ts.transform.rotation.z;
      ow = ts.transform.rotation.w;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      return false;
    }
  }

  // -------------------------------------------------------------------------
  void goal_reset_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    bool is_internal = has_pending_internal_ &&
                       msg->header.stamp == pending_internal_goal_stamp_;
    has_pending_internal_ = false;
    requested_goal_ = *msg;
    has_requested_goal_ = true;
    last_progress_time_ = ros::Time::now();
    empty_plan_count_ = 0;
    if (is_internal)
    {
      ROS_INFO("Internally republished goal accepted for replanning.");
      return;
    }

    replan_attempts_ = 0;
    has_best_goal_distance_ = false;
    ROS_INFO("New goal received; stopping the old split path.");
    active_path_ = false;
    waypoints_.clear();
    current_goal_idx_ = 0;
    publish_hold_goal("new-goal");
  }

  void cancel_callback(const actionlib_msgs::GoalIDConstPtr&)
  {
    active_path_ = false;
    waypoints_.clear();
    current_goal_idx_ = 0;
    publish_hold_goal("move-base-cancel");
    publish_recovery_status("cancelled");
  }

  void path_callback(const nav_msgs::PathConstPtr& msg)
  {
    if (msg->poses.empty())
    {
      empty_plan_count_++;
      active_path_ = false;
      waypoints_.clear();
      publish_hold_goal("empty-global-plan");
      ROS_WARN_THROTTLE(2.0, "Global planner returned an empty path (%d/%d).",
                        empty_plan_count_, empty_plan_threshold_);
      if (empty_plan_count_ >= empty_plan_threshold_)
        trigger_replan("empty-global-plan", false);
      return;
    }

    empty_plan_count_ = 0;
    ROS_INFO("Received new global plan with %zu poses.", msg->poses.size());

    waypoints_ = sample_path(msg->poses, segment_length_);
    current_goal_idx_ = 0;
    active_path_ = true;

    ROS_INFO("Path sampled to %zu waypoints. Starting execution.", waypoints_.size());
    publish_current_goal();
  }

  std::vector<geometry_msgs::PoseStamped> sample_path(
      const std::vector<geometry_msgs::PoseStamped>& poses, double interval)
  {
    std::vector<geometry_msgs::PoseStamped> sampled;
    if (poses.empty())
      return sampled;

    const geometry_msgs::PoseStamped* last_added = &poses[0];
    for (size_t i = 1; i < poses.size(); ++i)
    {
      const geometry_msgs::PoseStamped& pose = poses[i];
      if (calculate_distance(pose.pose.position, last_added->pose.position) >= interval)
      {
        sampled.push_back(pose);
        last_added = &pose;
      }
    }

    const geometry_msgs::PoseStamped& final_pose = poses.back();
    if (sampled.empty())
      sampled.push_back(final_pose);
    else if (calculate_distance(sampled.back().pose.position, final_pose.pose.position) > 0.1)
      sampled.push_back(final_pose);

    return sampled;
  }

  // -------------------------------------------------------------------------
  void control_loop(const ros::TimerEvent&)
  {
    if (!active_path_ || waypoints_.empty())
      return;

    double rx, ry, rz, ox, oy, oz, ow;
    if (!lookup_base_pose(rx, ry, rz, ox, oy, oz, ow))
      return;

    monitor_navigation_progress(rx, ry);
    if (!active_path_)
      return;

    if (current_goal_idx_ >= waypoints_.size())
      return;

    const geometry_msgs::PoseStamped& current_goal = waypoints_[current_goal_idx_];
    double gx = current_goal.pose.position.x;
    double gy = current_goal.pose.position.y;

    double dist = std::hypot(gx - rx, gy - ry);

    bool is_final_goal = (current_goal_idx_ == waypoints_.size() - 1);
    double current_threshold = is_final_goal ? final_tolerance_ : reach_tolerance_;

    if (dist < current_threshold)
    {
      if (!is_final_goal)
      {
        ROS_INFO("Waypoint %zu reached (Dist: %.2f). Moving to next.", current_goal_idx_, dist);
        current_goal_idx_++;
        publish_current_goal();
      }
      else
      {
        ROS_INFO("FINAL goal reached precisely (Dist: %.2f)! Mission Complete.", dist);
        active_path_ = false;
        waypoints_.clear();
      }
    }
  }

  void publish_current_goal()
  {
    if (current_goal_idx_ < waypoints_.size())
    {
      geometry_msgs::PoseStamped goal = waypoints_[current_goal_idx_];
      goal.header.stamp = ros::Time::now();
      goal_pub_.publish(goal);
      ROS_INFO(">> Publishing goal %zu/%zu to /split_goal: x=%.2f, y=%.2f",
               current_goal_idx_ + 1, waypoints_.size(),
               goal.pose.position.x, goal.pose.position.y);
    }
  }

  // -------------------------------------------------------------------------
  void monitor_navigation_progress(double robot_x, double robot_y)
  {
    if (progress_timeout_ <= 0.0 || !has_requested_goal_ || recovery_in_progress_)
      return;

    std::string goal_frame = requested_goal_.header.frame_id.empty()
                                 ? map_frame_
                                 : requested_goal_.header.frame_id;
    if (!goal_frame.empty() && goal_frame[0] == '/')
      goal_frame.erase(goal_frame.begin());
    std::string map_frame = map_frame_;
    if (!map_frame.empty() && map_frame[0] == '/')
      map_frame.erase(map_frame.begin());
    if (goal_frame != map_frame)
    {
      ROS_WARN_THROTTLE(10.0, "Cannot monitor goal progress in frame '%s'", goal_frame.c_str());
      return;
    }

    const geometry_msgs::Point& gp = requested_goal_.pose.position;
    double distance = std::hypot(gp.x - robot_x, gp.y - robot_y);
    ros::Time now = ros::Time::now();

    if (!has_best_goal_distance_)
    {
      has_best_goal_distance_ = true;
      best_goal_distance_ = distance;
      last_progress_time_ = now;
      return;
    }

    if (distance <= best_goal_distance_ - progress_distance_)
    {
      best_goal_distance_ = distance;
      last_progress_time_ = now;
      replan_attempts_ = 0;
      return;
    }

    double stalled_for = (now - last_progress_time_).toSec();
    if (stalled_for >= progress_timeout_)
    {
      ROS_WARN("No goal progress for %.1fs (distance=%.2fm); requesting a different global route.",
               stalled_for, distance);
      // Do not clear live obstacle observations here: the global obstacle
      // layer must retain them so GlobalPlanner routes around the blockage.
      trigger_replan("no-navigation-progress", false);
    }
  }

  bool trigger_replan(const std::string& reason, bool clear_costmaps)
  {
    if (!has_requested_goal_ || recovery_in_progress_)
      return false;

    ros::Time now = ros::Time::now();
    if ((now - last_replan_time_).toSec() < replan_cooldown_)
      return false;

    if (replan_attempts_ >= max_replan_attempts_)
    {
      ROS_ERROR("Navigation recovery exhausted after %d attempts; stopping the current mission.",
                replan_attempts_);
      active_path_ = false;
      waypoints_.clear();
      publish_hold_goal("recovery-exhausted");
      publish_recovery_status("failed:" + reason);
      cancel_pub_.publish(actionlib_msgs::GoalID());
      return false;
    }

    recovery_in_progress_ = true;
    replan_attempts_++;
    last_replan_time_ = now;
    last_progress_time_ = now;
    active_path_ = false;
    waypoints_.clear();
    publish_hold_goal(reason);

    if (clear_costmaps)
    {
      if (ros::service::waitForService(clear_costmaps_service_,
                                       ros::Duration(clear_costmaps_timeout_)))
      {
        std_srvs::Empty srv;
        if (clear_client_.call(srv))
          ROS_WARN("Cleared move_base costmaps before replanning.");
        else
          ROS_WARN("Failed to clear move_base costmaps: service call failed.");
      }
      else
      {
        ROS_WARN("Failed to clear move_base costmaps: service unavailable.");
      }
    }

    geometry_msgs::PoseStamped goal = requested_goal_;
    goal.header.stamp = ros::Time::now();
    pending_internal_goal_stamp_ = goal.header.stamp;
    has_pending_internal_ = true;
    goal_replan_pub_.publish(goal);
    empty_plan_count_ = 0;
    publish_recovery_status("replan:" + reason + ":" + std::to_string(replan_attempts_) +
                            "/" + std::to_string(max_replan_attempts_));
    ROS_WARN("Republished navigation goal for replanning (%s, attempt %d/%d).",
             reason.c_str(), replan_attempts_, max_replan_attempts_);
    recovery_in_progress_ = false;
    return true;
  }

  bool publish_hold_goal(const std::string& reason)
  {
    double rx, ry, rz, ox, oy, oz, ow;
    if (!lookup_base_pose(rx, ry, rz, ox, oy, oz, ow))
    {
      ROS_WARN_THROTTLE(5.0, "Cannot publish NavOA hold goal: TF unavailable");
      return false;
    }

    geometry_msgs::PoseStamped hold;
    hold.header.frame_id = map_frame_;
    hold.header.stamp = ros::Time::now();
    hold.pose.position.x = rx;
    hold.pose.position.y = ry;
    hold.pose.position.z = rz;
    hold.pose.orientation.x = ox;
    hold.pose.orientation.y = oy;
    hold.pose.orientation.z = oz;
    hold.pose.orientation.w = ow;
    goal_pub_.publish(hold);
    ROS_WARN_THROTTLE(2.0, "Holding NavOA at current pose (%s).", reason.c_str());
    return true;
  }

  void publish_recovery_status(const std::string& status)
  {
    std_msgs::String msg;
    msg.data = status;
    recovery_status_pub_.publish(msg);
  }

  static double calculate_distance(const geometry_msgs::Point& p1,
                                   const geometry_msgs::Point& p2)
  {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
  }

  // ---- params ----
  double segment_length_, reach_tolerance_, final_tolerance_;
  std::string base_frame_, map_frame_;
  double progress_timeout_, progress_distance_, replan_cooldown_;
  int max_replan_attempts_, empty_plan_threshold_;
  std::string clear_costmaps_service_;
  double clear_costmaps_timeout_;

  // ---- state ----
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  size_t current_goal_idx_;
  bool active_path_;
  geometry_msgs::PoseStamped requested_goal_;
  bool has_requested_goal_;
  int empty_plan_count_;
  int replan_attempts_;
  bool has_best_goal_distance_;
  double best_goal_distance_;
  ros::Time last_progress_time_, last_replan_time_;
  bool recovery_in_progress_;
  bool has_pending_internal_;
  ros::Time pending_internal_goal_stamp_;

  // ---- ros handles ----
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber goal_sub_, cancel_sub_, plan_sub_;
  ros::Publisher goal_pub_, goal_replan_pub_, cancel_pub_, recovery_status_pub_;
  ros::ServiceClient clear_client_;
  ros::Timer control_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_segment_publisher", ros::init_options::AnonymousName);
  PathToGoalFollower follower;
  ros::spin();
  return 0;
}
