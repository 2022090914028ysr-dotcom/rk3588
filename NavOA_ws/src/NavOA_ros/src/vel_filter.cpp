// C++ port of scripts/vel_filter.py
// Node name: cmd_vel_modifier_node (anonymous)
//
// Safety velocity filter sitting between the upstream controller and /cmd_vel:
//  - laser-scan forward safety with hysteresis latching
//  - ultrasonic virtual bumper (front/left/right) with active evasion
//  - optional camscan bias / virtual bumper (glass / transparent obstacles)
//  - output clamps, max-angular pivot, deadband
//  - command timeout: publishes a single zero command once the input expires
//
// Behavior (thresholds, mode names, debug/status strings) matches vel_filter.py.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <cmath>
#include <limits>
#include <map>
#include <string>

class VelocityFilter
{
public:
  VelocityFilter() : nh_("~"), pnh_("~")
  {
    scan_topic_ = pnh_.param<std::string>("scan_topic", "/scan");
    cmd_in_topic_ = pnh_.param<std::string>("cmd_in_topic", "/cmd_vel_tem");
    cmd_out_topic_ = pnh_.param<std::string>("cmd_out_topic", "/cmd_vel");

    cmd_timeout_ = pnh_.param("cmd_timeout", 0.5);
    publish_rate_ = pnh_.param("publish_rate", 20.0);
    debug_topic_ = pnh_.param<std::string>("debug_topic", "/vel_filter/debug");

    deadband_linear_x_ = pnh_.param("deadband_linear_x", 0.0);
    deadband_angular_z_ = pnh_.param("deadband_angular_z", 0.0);

    allow_reverse_ = pnh_.param("allow_reverse", true);
    reverse_requires_obstacle_ = pnh_.param("reverse_requires_obstacle", true);
    reverse_obstacle_distance_ = pnh_.param("reverse_obstacle_distance", 0.35);
    max_reverse_speed_ = pnh_.param("max_reverse_speed", 0.25);

    enable_forward_safety_ = pnh_.param("enable_forward_safety", true);
    front_center_angle_ = pnh_.param("front_center_angle", 0.0);
    forward_stop_distance_ = pnh_.param("forward_stop_distance", 0.30);
    forward_slow_distance_ = pnh_.param("forward_slow_distance", 0.60);
    forward_stop_release_distance_ =
        pnh_.param("forward_stop_release_distance", forward_slow_distance_);
    min_forward_speed_ = pnh_.param("min_forward_speed", 0.05);
    front_half_angle_ = pnh_.param("front_half_angle", 0.52);

    max_forward_speed_ = pnh_.param("max_forward_speed", 0.0);
    max_angular_speed_ = pnh_.param("max_angular_speed", 0.0);
    pivot_at_max_angular_speed_ = pnh_.param("pivot_at_max_angular_speed", true);

    wz_scale_ = pnh_.param("wz_scale", 1.0);

    enable_auto_reverse_ = pnh_.param("enable_auto_reverse", false);
    auto_reverse_speed_ = pnh_.param("auto_reverse_speed", -0.1);

    stop_turn_when_forward_blocked_ = pnh_.param("stop_turn_when_forward_blocked", true);

    enable_free_space_steer_ = pnh_.param("enable_free_space_steer", false);
    steer_scan_half_angle_ = pnh_.param("steer_scan_half_angle", 1.57);
    steer_deadzone_ = pnh_.param("steer_deadzone", 0.10);
    steer_gain_ = pnh_.param("steer_gain", 1.2);
    steer_max_angular_ = pnh_.param("steer_max_angular", 0.6);
    steer_activation_distance_ =
        pnh_.param("steer_activation_distance", forward_slow_distance_);
    steer_release_distance_ =
        pnh_.param("steer_release_distance", steer_activation_distance_ + 0.20);
    steer_forward_speed_ = pnh_.param("steer_forward_speed", 0.05);
    enable_turn_search_when_blocked_ = pnh_.param("enable_turn_search_when_blocked", true);
    turn_search_angular_ = pnh_.param("turn_search_angular", 0.35);

    min_scan_range_ = pnh_.param("min_scan_range", 0.0);

    enable_ultrasonic_bumper_ = pnh_.param("enable_ultrasonic_bumper", true);
    ultrasonic_stop_distance_ = pnh_.param("ultrasonic_stop_distance", 0.25);
    ultrasonic_slow_distance_ = pnh_.param("ultrasonic_slow_distance", 0.45);
    ultrasonic_side_stop_distance_ = pnh_.param("ultrasonic_side_stop_distance", 0.20);
    ultrasonic_blend_distance_ = pnh_.param("ultrasonic_blend_distance", 0.13);
    ultrasonic_evade_forward_speed_ = pnh_.param("ultrasonic_evade_forward_speed", 0.05);
    ultrasonic_evade_lateral_speed_ = pnh_.param("ultrasonic_evade_lateral_speed", 0.10);
    ultrasonic_linear_evade_ = pnh_.param("ultrasonic_linear_evade", true);
    ultrasonic_timeout_ = ros::Duration(pnh_.param("ultrasonic_timeout", 1.0));

    reverse_side_stop_distance_ = pnh_.param("reverse_side_stop_distance", 0.35);
    reverse_side_hold_duration_ = pnh_.param("reverse_side_hold_duration", 0.5);

    enable_camscan_bias_ = pnh_.param("enable_camscan_bias", true);
    camscan_topic_ = pnh_.param<std::string>("camscan_topic", "/camscan");
    camscan_front_threshold_ = pnh_.param("camscan_front_threshold", 0.4);
    camscan_bias_gain_ = pnh_.param("camscan_bias_gain", 0.5);

    enable_camscan_bumper_ = pnh_.param("enable_camscan_bumper", false);
    camscan_stop_distance_ = pnh_.param("camscan_stop_distance", 0.30);
    camscan_slow_distance_ = pnh_.param("camscan_slow_distance", 0.50);
    raw_scan_topic_ = pnh_.param<std::string>("raw_scan_topic", "/scan");
    camscan_timeout_ = ros::Duration(pnh_.param("camscan_timeout", 1.0));

    us_dist_["front"] = std::numeric_limits<double>::infinity();
    us_dist_["left"] = std::numeric_limits<double>::infinity();
    us_dist_["right"] = std::numeric_limits<double>::infinity();
    us_last_time_["front"] = ros::Time(0);
    us_last_time_["left"] = ros::Time(0);
    us_last_time_["right"] = ros::Time(0);

    _camscan_front_dist_ = std::numeric_limits<double>::infinity();
    _raw_scan_front_dist_ = std::numeric_limits<double>::infinity();

    front_min_range_ = std::numeric_limits<double>::infinity();
    obstacle_close_ = false;
    left_clearance_ = std::numeric_limits<double>::infinity();
    right_clearance_ = std::numeric_limits<double>::infinity();

    _last_cmd_time_ = ros::Time(0);
    _has_valid_cmd_ = false;
    _timeout_published_ = false;
    _steer_block_latched_ = false;
    _forward_stop_latched_ = false;
    _last_turn_sign_ = 1.0;
    _last_mode_ = "idle";
    _reverse_last_vy_ = 0.0;
    _reverse_hold_until_ = ros::Time(0);
    _reverse_evade_vy_ = 0.0;

    cmd_sub_ = nh_.subscribe(cmd_in_topic_, 10, &VelocityFilter::callback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &VelocityFilter::scan_callback, this);

    if (enable_ultrasonic_bumper_)
    {
      us_front_sub_ = nh_.subscribe("/ultrasonic/front", 10, &VelocityFilter::us_front_cb, this);
      us_left_sub_ = nh_.subscribe("/ultrasonic/left", 10, &VelocityFilter::us_left_cb, this);
      us_right_sub_ = nh_.subscribe("/ultrasonic/right", 10, &VelocityFilter::us_right_cb, this);
    }

    if (enable_camscan_bias_ || enable_camscan_bumper_)
    {
      camscan_sub_ = nh_.subscribe(camscan_topic_, 10, &VelocityFilter::camscan_callback, this);
    }
    if (enable_camscan_bias_)
    {
      raw_scan_sub_ = nh_.subscribe(raw_scan_topic_, 10, &VelocityFilter::raw_scan_callback, this);
    }

    pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_out_topic_, 10);
    debug_pub_ = nh_.advertise<std_msgs::String>(debug_topic_, 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/vel_filter/status", 10);
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(publish_rate_, 1.0)), &VelocityFilter::publish_timer, this);

    ROS_INFO("vel_filter running: %s -> %s (timeout=%.2fs, rate=%.1fHz, front_center=%.2frad)",
             cmd_in_topic_.c_str(), cmd_out_topic_.c_str(), cmd_timeout_, publish_rate_,
             front_center_angle_);
  }

private:
  // -------------------------------------------------------------------------
  static double angle_diff(double angle, double center)
  {
    return std::atan2(std::sin(angle - center), std::cos(angle - center));
  }

  void apply_deadband(geometry_msgs::Twist& twist)
  {
    if (deadband_linear_x_ > 0.0 && std::fabs(twist.linear.x) < deadband_linear_x_)
      twist.linear.x = 0.0;
    if (deadband_angular_z_ > 0.0 && std::fabs(twist.angular.z) < deadband_angular_z_)
      twist.angular.z = 0.0;
  }

  // Zero translation when the final yaw rate reaches its limit.
  bool apply_max_angular_pivot(geometry_msgs::Twist& twist)
  {
    if (!pivot_at_max_angular_speed_ || max_angular_speed_ <= 0.0)
      return false;
    double tolerance = std::max(1e-6, max_angular_speed_ * 1e-6);
    if (std::fabs(twist.angular.z) < max_angular_speed_ - tolerance)
      return false;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    return true;
  }

  void publish_debug(const geometry_msgs::Twist& in, const geometry_msgs::Twist& out)
  {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "mode=%s front=%.3f left=%.3f right=%.3f stop_latched=%s steer_latched=%s "
             "in_vx=%.3f in_w=%.3f out_vx=%.3f out_w=%.3f",
             _last_mode_.c_str(), front_min_range_, left_clearance_, right_clearance_,
             _forward_stop_latched_ ? "True" : "False",
             _steer_block_latched_ ? "True" : "False",
             in.linear.x, in.angular.z, out.linear.x, out.angular.z);
    std_msgs::String msg;
    msg.data = buf;
    debug_pub_.publish(msg);
  }

  void publish_status(const geometry_msgs::Twist& twist, const std::string& source)
  {
    double vx = twist.linear.x;
    double vy = twist.linear.y;
    double wz = twist.angular.z;

    std::string trend;
    if (vx > 0.01)
    {
      if (wz > 0.05)
        trend = "forward_left";
      else if (wz < -0.05)
        trend = "forward_right";
      else if (vy > 0.005)
        trend = "strafe_left";
      else if (vy < -0.005)
        trend = "strafe_right";
      else
        trend = "forward";
    }
    else if (vx < -0.01)
      trend = "reverse";
    else if (std::fabs(vy) > 0.005)
      trend = (vy > 0) ? "strafe_left" : "strafe_right";
    else if (std::fabs(wz) > 0.01)
      trend = (wz > 0) ? "turn_left" : "turn_right";
    else
      trend = "stop";

    std::string src = source.rfind("us_", 0) == 0 ? "ultrasonic" : "navoa";
    char buf[128];
    snprintf(buf, sizeof(buf), "source=%s mode=%s trend=%s vx=%.3f vy=%.3f wz=%.3f",
             src.c_str(), source.c_str(), trend.c_str(), vx, vy, wz);
    std_msgs::String msg;
    msg.data = buf;
    status_pub_.publish(msg);
  }

  void publish_timer(const ros::TimerEvent&)
  {
    ros::Time now = ros::Time::now();
    if (_last_cmd_time_ != ros::Time(0) &&
        (now - _last_cmd_time_).toSec() <= cmd_timeout_)
    {
      pub_.publish(_latest_output_);
      _timeout_published_ = false;
    }
    else if (_has_valid_cmd_ && !_timeout_published_)
    {
      ROS_INFO_ONCE("[vel_filter] cmd_vel timeout expired; publishing single zero command.");
      pub_.publish(geometry_msgs::Twist());
      _timeout_published_ = true;
      _has_valid_cmd_ = false;
    }
  }

  // -------------------------------------------------------------------------
  void scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if (msg->ranges.empty() || msg->angle_increment == 0.0)
    {
      front_min_range_ = std::numeric_limits<double>::infinity();
      obstacle_close_ = false;
      left_clearance_ = std::numeric_limits<double>::infinity();
      right_clearance_ = std::numeric_limits<double>::infinity();
      return;
    }

    double min_valid = std::max((msg->range_min > 0.0 ? msg->range_min : 0.0), min_scan_range_);
    double min_r = std::numeric_limits<double>::infinity();
    double range_max = (msg->range_max > 0.0) ? msg->range_max : 30.0;

    double left_sum = 0.0;
    size_t left_count = 0;
    double right_sum = 0.0;
    size_t right_count = 0;

    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      double r = msg->ranges[i];
      bool valid = std::isfinite(r) && r >= min_valid;
      double rel_angle = angle_diff(angle, front_center_angle_);

      if (-front_half_angle_ <= rel_angle && rel_angle <= front_half_angle_)
      {
        if (valid && r < min_r)
          min_r = r;
      }
      if (front_half_angle_ < rel_angle && rel_angle <= steer_scan_half_angle_ && valid)
      {
        left_sum += std::min(r, range_max);
        left_count++;
      }
      if (-steer_scan_half_angle_ <= rel_angle && rel_angle < -front_half_angle_ && valid)
      {
        right_sum += std::min(r, range_max);
        right_count++;
      }
      angle += msg->angle_increment;
    }

    front_min_range_ = min_r;
    obstacle_close_ = std::isfinite(min_r) && (min_r < reverse_obstacle_distance_);
    left_clearance_ = (left_count > 0) ? left_sum / left_count
                                       : std::numeric_limits<double>::infinity();
    right_clearance_ = (right_count > 0) ? right_sum / right_count
                                         : std::numeric_limits<double>::infinity();

    if (std::isfinite(min_r))
    {
      if (min_r < forward_stop_distance_)
        _forward_stop_latched_ = true;
      else if (min_r > forward_stop_release_distance_)
        _forward_stop_latched_ = false;
    }
  }

  // ---- ultrasonic callbacks (virtual bumper) -------------------------------
  void us_front_cb(const std_msgs::Float32ConstPtr& msg)
  {
    us_dist_["front"] = msg->data;
    us_last_time_["front"] = ros::Time::now();
  }
  void us_left_cb(const std_msgs::Float32ConstPtr& msg)
  {
    us_dist_["left"] = msg->data;
    us_last_time_["left"] = ros::Time::now();
  }
  void us_right_cb(const std_msgs::Float32ConstPtr& msg)
  {
    us_dist_["right"] = msg->data;
    us_last_time_["right"] = ros::Time::now();
  }

  double us_get_valid(const std::string& name)
  {
    ros::Time now = ros::Time::now();
    if ((now - us_last_time_[name]) < ultrasonic_timeout_)
    {
      double d = us_dist_[name];
      if (std::isfinite(d) && d > 0.0)
        return d;
    }
    return std::numeric_limits<double>::infinity();
  }

  // ---- camscan callback (glass / transparent obstacle detection) ----------
  void camscan_callback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if (msg->ranges.empty() || msg->angle_increment == 0.0)
    {
      _camscan_front_dist_ = std::numeric_limits<double>::infinity();
      return;
    }
    double min_valid = std::max((msg->range_min > 0.0 ? msg->range_min : 0.0), min_scan_range_);
    double min_r = std::numeric_limits<double>::infinity();
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      double r = msg->ranges[i];
      if (std::isfinite(r) && r >= min_valid)
      {
        double rel = angle_diff(angle, front_center_angle_);
        if (-front_half_angle_ <= rel && rel <= front_half_angle_ && r < min_r)
          min_r = r;
      }
      angle += msg->angle_increment;
    }
    _camscan_front_dist_ = min_r;
    _camscan_last_time_ = ros::Time::now();
  }

  double camscan_front_valid()
  {
    if ((ros::Time::now() - _camscan_last_time_) < camscan_timeout_)
    {
      double d = _camscan_front_dist_;
      if (std::isfinite(d) && d > 0.0)
        return d;
    }
    return std::numeric_limits<double>::infinity();
  }

  void raw_scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if (msg->ranges.empty() || msg->angle_increment == 0.0)
    {
      _raw_scan_front_dist_ = std::numeric_limits<double>::infinity();
      return;
    }
    double min_valid = std::max((msg->range_min > 0.0 ? msg->range_min : 0.0), min_scan_range_);
    double min_r = std::numeric_limits<double>::infinity();
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      double r = msg->ranges[i];
      if (std::isfinite(r) && r >= min_valid)
      {
        double rel = angle_diff(angle, front_center_angle_);
        if (-front_half_angle_ <= rel && rel <= front_half_angle_ && r < min_r)
          min_r = r;
      }
      angle += msg->angle_increment;
    }
    _raw_scan_front_dist_ = min_r;
    _raw_scan_last_time_ = ros::Time::now();
  }

  double raw_scan_front_valid()
  {
    // Matches the Python: validity window reuses camscan_timeout.
    if ((ros::Time::now() - _raw_scan_last_time_) < camscan_timeout_)
    {
      double d = _raw_scan_front_dist_;
      if (std::isfinite(d) && d > 0.0)
        return d;
    }
    return std::numeric_limits<double>::infinity();
  }

  void apply_camscan_bias(geometry_msgs::Twist& twist)
  {
    if (!enable_camscan_bias_)
      return;

    double cam_front = camscan_front_valid();
    if (!std::isfinite(cam_front) || cam_front >= camscan_front_threshold_)
      return;

    double raw_front = raw_scan_front_valid();
    if (std::isfinite(raw_front) && raw_front < camscan_front_threshold_)
      return;

    double rot_bias = 0.0;
    if (twist.angular.z > 0.03)
      rot_bias = 1.0;
    else if (twist.angular.z < -0.03)
      rot_bias = -1.0;

    double us_bias = 0.0;
    if (enable_ultrasonic_bumper_)
    {
      double d_left = us_get_valid("left");
      double d_right = us_get_valid("right");
      bool left_ok = std::isfinite(d_left);
      bool right_ok = std::isfinite(d_right);
      if (left_ok && right_ok)
      {
        double diff = d_right - d_left;
        us_bias = (std::fabs(diff) > 0.03) ? ((diff > 0) ? 1.0 : -1.0) : 0.0;
      }
      else if (left_ok)
        us_bias = -1.0;
      else if (right_ok)
        us_bias = 1.0;
    }

    double bias = rot_bias + us_bias;
    if (std::fabs(bias) < 0.01)
      return;

    double vy_add = std::copysign(camscan_bias_gain_, bias);
    twist.linear.y += vy_add;
    twist.angular.z = 0.0;
  }

  double compute_linear_evade_speed(double distance)
  {
    if (!ultrasonic_linear_evade_)
      return std::fabs(ultrasonic_evade_lateral_speed_);
    double threshold = ultrasonic_side_stop_distance_;
    if (threshold <= 0.0 || distance >= threshold)
      return 0.0;
    double scale = (threshold - distance) / threshold;
    return std::fabs(ultrasonic_evade_lateral_speed_) * std::max(0.0, std::min(1.0, scale));
  }

  // Ultrasonic + camscan virtual bumper with active evasion.
  void apply_ultrasonic_bumper(geometry_msgs::Twist& twist, std::string& mode)
  {
    mode = "pass_through";
    if (!enable_ultrasonic_bumper_ && !enable_camscan_bumper_)
      return;

    double d_front = us_get_valid("front");
    double d_left = us_get_valid("left");
    double d_right = us_get_valid("right");

    bool cam_front_only = false;
    if (enable_camscan_bumper_)
    {
      double d_cam_front = camscan_front_valid();
      if (std::isfinite(d_cam_front))
      {
        if (std::isfinite(d_front))
          d_front = std::min(d_front, d_cam_front);
        else
          d_front = d_cam_front;
        if (d_cam_front < camscan_stop_distance_ &&
            (!std::isfinite(us_get_valid("front")) ||
             us_get_valid("front") >= ultrasonic_stop_distance_))
          cam_front_only = true;
      }
    }

    double eff_stop_dist = ultrasonic_stop_distance_;
    double eff_slow_dist = ultrasonic_slow_distance_;
    if (enable_camscan_bumper_)
    {
      eff_stop_dist = std::min(eff_stop_dist, camscan_stop_distance_);
      eff_slow_dist = std::max(eff_slow_dist, camscan_slow_distance_);
    }

    bool front_blocked = std::isfinite(d_front) && d_front < eff_stop_dist;
    bool left_blocked = std::isfinite(d_left) && d_left < ultrasonic_side_stop_distance_;
    bool right_blocked = std::isfinite(d_right) && d_right < ultrasonic_side_stop_distance_;

    // ---- no trigger: maybe just slow-down ----
    if (!(front_blocked || left_blocked || right_blocked))
    {
      _reverse_evade_vy_ = 0.0;
      ros::Time now = ros::Time::now();
      if (reverse_side_stop_distance_ > 0.0 && now < _reverse_hold_until_ &&
          std::fabs(_reverse_last_vy_) > 0.001)
      {
        twist.linear.y = _reverse_last_vy_;
        mode = "us_reverse_hold";
        return;
      }

      if (std::isfinite(d_front) && d_front < eff_slow_dist)
      {
        if (twist.linear.x > 0.0)
        {
          double denom = eff_slow_dist - eff_stop_dist;
          double scale = (denom > 0.0) ? (d_front - eff_stop_dist) / denom : 0.0;
          scale = std::max(0.0, std::min(1.0, scale));
          twist.linear.x *= scale;
          mode = "us_slow_forward";
          return;
        }
      }
      mode = "pass_through";
      return;
    }

    // ---- determine evasive action ----
    if (front_blocked)
    {
      if (enable_auto_reverse_ && allow_reverse_)
      {
        double orig_wz = twist.angular.z;
        twist.linear.x = -std::fabs(auto_reverse_speed_);
        twist.angular.z = 0.0;
        mode = cam_front_only ? "cam_reverse" : "us_reverse";

        if (reverse_side_stop_distance_ > 0.0)
        {
          if (std::fabs(_reverse_evade_vy_) < 0.001)
          {
            bool rev_left = std::isfinite(d_left) && d_left < reverse_side_stop_distance_;
            bool rev_right = std::isfinite(d_right) && d_right < reverse_side_stop_distance_;
            double lat = std::fabs(ultrasonic_evade_lateral_speed_);
            if (rev_left && rev_right)
              _reverse_evade_vy_ = (d_right > d_left) ? lat : -lat;
            else if (rev_left)
              _reverse_evade_vy_ = lat;
            else if (rev_right)
              _reverse_evade_vy_ = -lat;
            else
            {
              if (orig_wz > 0.03)
                _reverse_evade_vy_ = lat;
              else if (orig_wz < -0.03)
                _reverse_evade_vy_ = -lat;
            }
          }
          twist.linear.y = _reverse_evade_vy_;

          if (_reverse_evade_vy_ > 0.001 && std::isfinite(d_right) &&
              d_right < ultrasonic_side_stop_distance_)
            twist.linear.y = 0.0;
          else if (_reverse_evade_vy_ < -0.001 && std::isfinite(d_left) &&
                   d_left < ultrasonic_side_stop_distance_)
            twist.linear.y = 0.0;
        }

        _reverse_last_vy_ = twist.linear.y;
        _reverse_hold_until_ = ros::Time::now() + ros::Duration(reverse_side_hold_duration_);
      }
      else
      {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        mode = cam_front_only ? "cam_front_stop" : "us_front_stop";
      }
    }
    else if (left_blocked && right_blocked)
    {
      double speed_left = compute_linear_evade_speed(d_left);
      double speed_right = compute_linear_evade_speed(d_right);
      twist.linear.y = speed_left - speed_right;
      double closest_side = std::min(d_left, d_right);
      if (closest_side < ultrasonic_blend_distance_)
      {
        double scale = closest_side / std::max(ultrasonic_blend_distance_, 0.01);
        twist.linear.x *= std::max(0.0, scale);
      }
      mode = "us_narrow_pass";
    }
    else if (left_blocked)
    {
      double speed = compute_linear_evade_speed(d_left);
      twist.linear.y = speed;
      if (d_left < ultrasonic_blend_distance_)
      {
        double scale = d_left / std::max(ultrasonic_blend_distance_, 0.01);
        twist.linear.x *= std::max(0.0, scale);
      }
      mode = "us_evade_left";
    }
    else if (right_blocked)
    {
      double speed = compute_linear_evade_speed(d_right);
      twist.linear.y = -speed;
      if (d_right < ultrasonic_blend_distance_)
      {
        double scale = d_right / std::max(ultrasonic_blend_distance_, 0.01);
        twist.linear.x *= std::max(0.0, scale);
      }
      mode = "us_evade_right";
    }
  }

  // Reactive steering toward the more open side (kept for parity with the
  // Python class; not invoked by the main callback there either).
  void apply_free_space_steer(geometry_msgs::Twist& twist, bool forward_blocked)
  {
    if (!enable_free_space_steer_)
      return;
    if (!forward_blocked)
      return;
    if (!std::isfinite(front_min_range_))
      return;

    double balance = left_clearance_ - right_clearance_;
    double target_angular;
    if (std::isfinite(balance) && std::fabs(balance) >= steer_deadzone_)
    {
      target_angular = steer_gain_ * balance;
      _last_turn_sign_ = (target_angular >= 0.0) ? 1.0 : -1.0;
    }
    else if (enable_turn_search_when_blocked_)
      target_angular = _last_turn_sign_ * std::fabs(turn_search_angular_);
    else
      return;

    target_angular = std::max(-steer_max_angular_, std::min(steer_max_angular_, target_angular));
    if (twist.linear.x > 0.0)
      twist.linear.x = std::min(twist.linear.x, steer_forward_speed_);
    twist.angular.z = target_angular;
  }

  // -------------------------------------------------------------------------
  void callback(const geometry_msgs::TwistConstPtr& msg)
  {
    _last_cmd_time_ = ros::Time::now();
    _has_valid_cmd_ = true;

    geometry_msgs::Twist modified = *msg;
    _last_mode_ = "pass_through";

    // ---- ultrasonic virtual bumper (highest priority) ----
    if (enable_ultrasonic_bumper_)
    {
      std::string us_mode;
      apply_ultrasonic_bumper(modified, us_mode);
      if (us_mode != "pass_through")
      {
        modified.angular.z *= wz_scale_;
        if (max_forward_speed_ > 0.0 && modified.linear.x > max_forward_speed_)
          modified.linear.x = max_forward_speed_;
        if (max_angular_speed_ > 0.0)
        {
          double az = modified.angular.z;
          if (std::fabs(az) > max_angular_speed_)
            modified.angular.z = std::copysign(max_angular_speed_, az);
        }
        bool pivoting = apply_max_angular_pivot(modified);
        apply_deadband(modified);
        _latest_output_ = modified;
        _last_mode_ = pivoting ? (us_mode + "_pivot") : us_mode;
        publish_debug(*msg, modified);
        publish_status(modified, _last_mode_);
        return;
      }
    }

    // ---- camscan bias: camscan sees front obstacle but lidar doesn't ----
    apply_camscan_bias(modified);

    modified.angular.z *= wz_scale_;

    if (max_forward_speed_ > 0.0 && modified.linear.x > max_forward_speed_)
      modified.linear.x = max_forward_speed_;
    if (max_angular_speed_ > 0.0)
    {
      double az = modified.angular.z;
      if (std::fabs(az) > max_angular_speed_)
        modified.angular.z = std::copysign(max_angular_speed_, az);
    }

    bool pivoting = apply_max_angular_pivot(modified);
    apply_deadband(modified);
    _latest_output_ = modified;
    _last_mode_ = pivoting ? "max_angular_pivot" : "pass_through";
    publish_debug(*msg, modified);
    publish_status(modified, _last_mode_);
  }

  // ---- config ----
  ros::NodeHandle nh_, pnh_;
  std::string scan_topic_, cmd_in_topic_, cmd_out_topic_, debug_topic_;
  double cmd_timeout_, publish_rate_;
  double deadband_linear_x_, deadband_angular_z_;
  bool allow_reverse_, reverse_requires_obstacle_;
  double reverse_obstacle_distance_, max_reverse_speed_;
  bool enable_forward_safety_;
  double front_center_angle_, forward_stop_distance_, forward_slow_distance_;
  double forward_stop_release_distance_, min_forward_speed_, front_half_angle_;
  double max_forward_speed_, max_angular_speed_;
  bool pivot_at_max_angular_speed_;
  double wz_scale_;
  bool enable_auto_reverse_;
  double auto_reverse_speed_;
  bool stop_turn_when_forward_blocked_;
  bool enable_free_space_steer_;
  double steer_scan_half_angle_, steer_deadzone_, steer_gain_, steer_max_angular_;
  double steer_activation_distance_, steer_release_distance_, steer_forward_speed_;
  bool enable_turn_search_when_blocked_;
  double turn_search_angular_;
  double min_scan_range_;
  bool enable_ultrasonic_bumper_;
  double ultrasonic_stop_distance_, ultrasonic_slow_distance_, ultrasonic_side_stop_distance_;
  double ultrasonic_blend_distance_, ultrasonic_evade_forward_speed_, ultrasonic_evade_lateral_speed_;
  bool ultrasonic_linear_evade_;
  ros::Duration ultrasonic_timeout_;
  double reverse_side_stop_distance_, reverse_side_hold_duration_;
  bool enable_camscan_bias_, enable_camscan_bumper_;
  std::string camscan_topic_, raw_scan_topic_;
  double camscan_front_threshold_, camscan_bias_gain_, camscan_stop_distance_, camscan_slow_distance_;
  ros::Duration camscan_timeout_;

  // ---- state ----
  std::map<std::string, double> us_dist_;
  std::map<std::string, ros::Time> us_last_time_;
  double _camscan_front_dist_, _raw_scan_front_dist_;
  ros::Time _camscan_last_time_, _raw_scan_last_time_;
  double front_min_range_, left_clearance_, right_clearance_;
  bool obstacle_close_;
  ros::Time _last_cmd_time_;
  geometry_msgs::Twist _latest_output_;
  bool _has_valid_cmd_, _timeout_published_, _steer_block_latched_, _forward_stop_latched_;
  double _last_turn_sign_;
  std::string _last_mode_;
  double _reverse_last_vy_, _reverse_evade_vy_;
  ros::Time _reverse_hold_until_;

  // ---- ros handles ----
  ros::Subscriber cmd_sub_, scan_sub_, us_front_sub_, us_left_sub_, us_right_sub_;
  ros::Subscriber camscan_sub_, raw_scan_sub_;
  ros::Publisher pub_, debug_pub_, status_pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_modifier_node", ros::init_options::AnonymousName);
  VelocityFilter filter;
  ros::spin();
  return 0;
}
