/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <cmath>
#include <thread>

#include <costmap_2d/cost_values.h>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("goal_search_radius", goal_search_radius_, 0.5);
  private_nh_.param("blacklist_timeout", blacklist_timeout_, 60.0);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    // No frontiers right now. Cancel any active goal but keep the exploration
    // timer alive: the map may still grow and produce new frontiers, which we
    // must pick up automatically instead of permanently stopping.
    move_base_client_.cancelAllGoals();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find a non blacklisted frontier whose goal move_base can plan to
  // (frontier centroids usually lie in unknown space or inside the obstacle
  //  inflation region, where GlobalPlanner cannot produce a plan)
  auto frontier = frontiers.end();
  geometry_msgs::Point target_position;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    if (goalOnBlacklist(it->centroid)) {
      continue;
    }
    geometry_msgs::Point goal = it->centroid;
    if (!relocateGoal(goal)) {
      frontier_blacklist_.push_back({it->centroid, ros::Time::now()});
      ROS_WARN("Frontier at (%.2f, %.2f) has no plannable goal nearby, "
               "blacklisting",
               it->centroid.x, it->centroid.y);
      continue;
    }
    frontier = it;
    target_position = goal;
    break;
  }
  if (frontier == frontiers.end()) {
    // Every candidate frontier is non-plannable or blacklisted. Do not stop
    // permanently: blacklisted goals expire and new frontiers may form, so we
    // simply cancel the active goal and wait for the next planning cycle.
    move_base_client_.cancelAllGoals();
    return;
  }
  // blacklist by the frontier centroid, not by the (relocated) goal, so the
  // blacklist keeps matching future frontier searches
  const geometry_msgs::Point target_centroid = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back({target_centroid, ros::Time::now()});
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position, target_centroid](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position, target_centroid);
      });
}

bool Explore::relocateGoal(geometry_msgs::Point& goal)
{
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  unsigned int gx, gy;
  if (!costmap2d->worldToMap(goal.x, goal.y, gx, gy)) {
    ROS_DEBUG("Frontier goal (%.2f, %.2f) is outside the costmap bounds",
              goal.x, goal.y);
    return false;
  }

  // GlobalPlanner treats every cell with cost >= INSCRIBED_INFLATED_OBSTACLE
  // (lethal, inscribed inflation and unknown) as an obstacle, so a goal is
  // only plannable on a cell below that value.
  auto plannable = [costmap2d](int x, int y) {
    return costmap2d->getCost(x, y) <
           costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  if (plannable(gx, gy)) {
    return true;  // already on a plannable cell
  }

  const int size_x = static_cast<int>(costmap2d->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap2d->getSizeInCellsY());
  const double resolution = costmap2d->getResolution();
  const int max_radius = std::max(
      1, static_cast<int>(std::ceil(goal_search_radius_ / resolution)));

  // Expand square rings around the goal. Prefer the nearest completely free
  // cell (cost 0, i.e. also outside the inflation region); fall back to the
  // cheapest traversable cell. The choice must stay deterministic between
  // planning cycles, otherwise goals would alternate forever and the
  // progress timeout could never blacklist a stuck frontier.
  bool found_free = false;
  int free_ring = 0;
  double free_dist = 0.;
  double free_wx = 0., free_wy = 0.;

  bool found_any = false;
  unsigned int any_cost = 0;
  double any_dist = 0.;
  double any_wx = 0., any_wy = 0.;

  for (int r = 1; r <= max_radius && !found_free; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        // ring cells only
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int x = static_cast<int>(gx) + dx;
        const int y = static_cast<int>(gy) + dy;
        if (x < 0 || y < 0 || x >= size_x || y >= size_y) {
          continue;
        }
        if (!plannable(x, y)) {
          continue;
        }
        double wx, wy;
        costmap2d->mapToWorld(x, y, wx, wy);
        const double dist = std::hypot(wx - goal.x, wy - goal.y);
        const unsigned int cost = costmap2d->getCost(x, y);
        if (cost == costmap_2d::FREE_SPACE &&
            (!found_free || dist < free_dist)) {
          found_free = true;
          free_ring = r;
          free_dist = dist;
          free_wx = wx;
          free_wy = wy;
        } else if (!found_free &&
                   (!found_any || cost < any_cost ||
                    (cost == any_cost && dist < any_dist))) {
          found_any = true;
          any_cost = cost;
          any_dist = dist;
          any_wx = wx;
          any_wy = wy;
        }
      }
    }
  }

  if (found_free) {
    ROS_DEBUG("Relocated goal %d cells to free space: (%.2f, %.2f) -> "
              "(%.2f, %.2f)",
              free_ring, goal.x, goal.y, free_wx, free_wy);
    goal.x = free_wx;
    goal.y = free_wy;
    return true;
  }
  if (found_any) {
    ROS_DEBUG("Relocated goal to traversable cell (cost %u): (%.2f, %.2f) -> "
              "(%.2f, %.2f)",
              any_cost, goal.x, goal.y, any_wx, any_wy);
    goal.x = any_wx;
    goal.y = any_wy;
    return true;
  }
  return false;
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  const ros::Time now = ros::Time::now();
  // drop blacklist entries that have expired, so a region that was unreachable
  // earlier (blocked obstacle, narrow passage) can be re-considered once the
  // robot gets around to it later
  frontier_blacklist_.erase(
      std::remove_if(
          frontier_blacklist_.begin(), frontier_blacklist_.end(),
          [this, now](const BlacklistEntry& entry) {
            return (now - entry.added) > ros::Duration(blacklist_timeout_);
          }),
      frontier_blacklist_.end());

  // check if a goal is on the blacklist for goals that we're pursuing
  for (const auto& entry : frontier_blacklist_) {
    double x_diff = fabs(goal.x - entry.point.x);
    double y_diff = fabs(goal.y - entry.point.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal,
                          const geometry_msgs::Point& frontier_centroid)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back({frontier_centroid, ros::Time::now()});
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
