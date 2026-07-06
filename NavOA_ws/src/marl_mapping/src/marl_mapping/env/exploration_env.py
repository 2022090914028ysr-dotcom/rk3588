#!/usr/bin/python3
"""
Real-World Exploration Environment (Single Robot)
Adapted from NeuralCoMapping's Exploration_Env_Real_World for single robot.
Handles map building, frontier detection, and robot state management.
"""

import math
import os
import pickle
import gzip
import numpy as np
import random
import cv2
import copy
import time
from collections import deque

import rospy
import tf
from scipy.spatial.transform import Rotation as R

from marl_mapping.env.robot_interface import SingleRobot
from marl_mapping.utils.map_builder import MapBuilder
from marl_mapping.utils.fmm_planner import FMMPlanner


class RealWorldExplorationEnv:
    """Real-world exploration environment for single robot autonomous mapping."""

    def __init__(self, args, snp_data=None):
        self.args = args
        self.num_robots = 1  # Single robot
        self.snp = snp_data

        # Core parameters
        self.map_size_cm = args.map_size_cm
        self.map_resolution = args.map_resolution  # cm/pixel
        self.map_size = self.map_size_cm // self.map_resolution
        self.unit_size_cm = args.unit_size_cm

        # Episode tracking
        self.timestep = 0
        self.current_episode = 0
        self.close = False
        self.collision_tolerance = args.max_collisions_allowed

        # Robot state
        self.straight = 0
        self.rotate = 0
        self.bump = False
        self.cont_bump = 0
        self.collision_dir = 0
        self.escape = 0
        self.last_stg = (-1.0, -1.0)
        self.last_stg_dist = 0.0
        self.l_reward = 0.0

        # Initialize robot interface
        rospy.loginfo("[ExplorationEnv] Connecting to real robot...")
        self.robot = SingleRobot(
            rb_name=args.robot_name,
            check_topic=True
        )
        rospy.loginfo("[ExplorationEnv] Robot interface ready.")

        # Map builder for occupancy grid processing
        self.mapper = MapBuilder(
            map_size_cm=self.map_size_cm,
            map_resolution=self.map_resolution,
            unit_size_cm=self.unit_size_cm
        )

        # Data directories
        self.dump_dir = "{}/dump/{}".format(args.dump_location, args.exp_name)
        self.npy_dir = os.path.join(self.dump_dir, "npy_data")
        self.action_dir = os.path.join(self.dump_dir, "action_data")
        for d in [self.dump_dir, self.npy_dir, self.action_dir]:
            os.makedirs(d, exist_ok=True)

        # Map state
        self.obstacle_map = None
        self.explored_map = None
        self.curr_frontier = None
        self.explorable_map = None
        self.prev_explored_area = 0.0
        self.frontier_times = None

        # Pose tracking
        self.curr_loc = [[self.map_size_cm / 100.0 / 2.0, self.map_size_cm / 100.0 / 2.0, 0]]
        self.last_loc = copy.deepcopy(self.curr_loc)

        # Frontier from map (direct method)
        self._frontier = None

    def reset(self):
        """Reset the environment for a new episode."""
        args = self.args
        self.timestep = 0
        self.straight = 0
        self.rotate = 0
        self.bump = False
        self.cont_bump = 0
        self.collision_dir = 0
        self.escape = 0
        self.close = False
        self.collision_tolerance = args.max_collisions_allowed
        self.last_stg = (-1.0, -1.0)
        self.last_stg_dist = 0.0
        self.l_reward = 0.0
        self.prev_explored_area = 0.0

        # Get maps from real robot
        self.obstacle_map, self.explored_map = self._get_robot_maps()
        self.curr_frontier = self._compute_frontier(self.obstacle_map, self.explored_map)
        self.explorable_map = np.ones_like(self.obstacle_map).astype(np.uint8)

        # Initialize frontier times counter
        self.frontier_times = np.zeros(self.obstacle_map.shape)

        # Save initial robot data
        gz_path = os.path.join(self.npy_dir, "{}.pkl.gz".format(self.timestep))
        robot_data = self.robot.get_all_data()
        robot_data.append(5)  # Append action type (stop)
        with gzip.open(gz_path, 'wb') as f:
            pickle.dump([robot_data], f)

        rospy.loginfo("[ExplorationEnv] Environment reset complete.")

    def step(self, action):
        """
        Execute one step with the given action.
        Action mapping:
            0: Left, 1: Right, 2: Forward, 3: Backward
        Returns: (done, info_dict)
        """
        args = self.args
        self.timestep += 1

        # Map RL action to robot movement type
        action_map = {0: 4, 1: 6, 2: 8, 3: 2}  # Left, Right, Forward, Backward
        move_type = action_map.get(action, 5)  # Default: stop

        # Execute movement on real robot
        for _ in range(args.action_repeat):
            self.robot.move(n=1, move_type=move_type,
                           linear=args.linear_speed, angular=args.angular_speed)

        # Wait for new sensor data
        rospy.sleep(args.step_delay)

        # Save action for replay
        action_path = os.path.join(self.action_dir, "{}.pkl.gz".format(self.timestep))
        with gzip.open(action_path, 'wb') as f:
            pickle.dump(action, f)

        # Save robot data
        gz_path = os.path.join(self.npy_dir, "{}.pkl.gz".format(self.timestep))
        robot_data = self.robot.get_all_data()
        robot_data.append(move_type)
        with gzip.open(gz_path, 'wb') as f:
            pickle.dump([robot_data], f)

        # Check episode termination
        if self.timestep >= args.max_episode_length:
            rospy.loginfo("[ExplorationEnv] Max episode length reached.")
            self.robot.stop()
            return True, self._get_info()

        if self.close:
            return True, self._get_info()

        # Update maps
        self._update_maps()
        self.last_loc = copy.deepcopy(self.curr_loc)

        return False, self._get_info()

    def _get_robot_maps(self):
        """Get and process maps from the real robot."""
        robot_map = self.robot.get_map()
        if robot_map is None:
            rospy.logwarn("[ExplorationEnv] No map received, using empty map.")
            empty_map = np.full((self.map_size, self.map_size), -1, dtype=np.int8)
            obstacle_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
            explored_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
            return obstacle_map, explored_map

        # Process occupancy grid: -1=unknown, 0=free, 100=occupied
        processed = self.mapper.process_map(robot_map, self.robot.get_map_info())
        obstacle_map = processed['obstacle']
        explored_map = processed['explored']

        # Update robot pose from EKF
        ekf_pos = self.robot.get_ekf_pos()
        self.curr_loc = [[
            ekf_pos[0] * 100.0 / self.unit_size_cm + self.map_size // 2,
            ekf_pos[1] * 100.0 / self.unit_size_cm + self.map_size // 2,
            0.0
        ]]

        return obstacle_map, explored_map

    def _compute_frontier(self, obstacle_map, explored_map):
        """Compute frontiers: boundaries between explored free space and unknown space."""
        # Dilate obstacles slightly for safety
        kernel = np.ones((3, 3), dtype=np.uint8)
        safe_obstacle = cv2.dilate(obstacle_map.astype(np.uint8), kernel, iterations=2)

        # Frontier = explored free cells adjacent to unknown cells, excluding obstacles
        unknown = (explored_map == 0).astype(np.uint8)
        dilated_unknown = cv2.dilate(unknown, kernel, iterations=1)

        frontier = np.logical_and(
            np.logical_and(explored_map > 0, safe_obstacle == 0),
            dilated_unknown > 0
        ).astype(np.uint8)

        return frontier

    def _update_maps(self):
        """Update maps with latest robot data."""
        obstacle_map, explored_map = self._get_robot_maps()
        if obstacle_map is not None:
            self.obstacle_map = obstacle_map
            self.explored_map = explored_map
            self.curr_frontier = self._compute_frontier(obstacle_map, explored_map)

    def get_current_state(self):
        """Get current state dict for the RL policy."""
        self._update_maps()
        return {
            'obstacle': self.obstacle_map,
            'frontier': self.curr_frontier,
            'explored': self.explored_map,
            'explorable': self.explorable_map if self.explorable_map is not None
            else np.ones_like(self.obstacle_map).astype(np.uint8),
            'sensor_pose': np.array(self.curr_loc[0] if self.curr_loc else [0, 0, 0]),
            'timestep': self.timestep,
        }

    def get_global_reward(self):
        """Compute exploration reward based on explored area ratio."""
        if self.explored_map is None:
            return 0.0, 0.0, 0.0, 0.0, 0.0

        total_cells = self.obstacle_map.size
        obstacle_cells = np.sum(self.obstacle_map > 0)
        explorable_cells = total_cells - obstacle_cells
        explored_cells = np.sum(self.explored_map > 0) - obstacle_cells

        explored_area = float(explored_cells)
        ratio = explored_cells / max(explorable_cells, 1)
        unexp_area = explorable_cells - explored_cells

        return explored_area, ratio, ratio, unexp_area, explored_area

    def _get_info(self):
        """Build info dict with all state data."""
        explored_area, ratio, ratio_all, unexp_area, exp_area_all = self.get_global_reward()
        return {
            'exp_reward': explored_area,
            'exp_ratio': ratio,
            'exp_area_all': exp_area_all,
            'exp_ratio_all': ratio_all,
            'unexp_area': unexp_area,
            'g_reward': 0.0,
            'bump': self.bump,
            'obstacle': self.obstacle_map,
            'frontier': self.curr_frontier,
            'explored': self.explored_map,
            'sensor_pose': np.array(self.curr_loc[0] if self.curr_loc else [0, 0, 0]),
        }

    def close_env(self):
        """Clean shutdown."""
        self.robot.stop()
        rospy.loginfo("[ExplorationEnv] Environment closed.")
