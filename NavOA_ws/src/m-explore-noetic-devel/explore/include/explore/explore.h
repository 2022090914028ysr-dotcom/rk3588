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

#ifndef EXPLORE_H_
#define EXPLORE_H_

#include <cstdint>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "explore/costmap_client.h"
#include "explore/frontier_search.h"

namespace explore
{
class Explore
{
public:
  Explore();
  ~Explore();

private:
  void makePlan();
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);
  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& frontier_goal,
                   const geometry_msgs::Point& frontier_centroid);
  bool goalOnBlacklist(const geometry_msgs::Point& goal);
  bool relocateGoal(geometry_msgs::Point& goal);
  void start();
  void stop();

  ros::NodeHandle private_nh_, relative_nh_;
  tf::TransformListener tf_listener_;
  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_, oneshot_;
  ros::Publisher marker_array_publisher_;
  struct BlacklistEntry {
    geometry_msgs::Point point;
    ros::Time added;
  };
  std::vector<BlacklistEntry> frontier_blacklist_;
  double blacklist_timeout_;
  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  std::size_t last_markers_count_;
  ros::Time last_progress_;
  ros::Duration progress_timeout_;
  bool visualize_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double goal_search_radius_;
  double planner_frequency_;
};

}  // namespace explore

#endif  // EXPLORE_H_
