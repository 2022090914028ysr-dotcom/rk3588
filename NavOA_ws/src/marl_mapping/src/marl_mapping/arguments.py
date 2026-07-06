#!/usr/bin/python3
"""
Configuration arguments for autonomous mapping module.
Simplified from NeuralCoMapping's arguments.py for real-world single robot use.
"""

import argparse


def get_exploration_args():
    """Get argument parser for exploration module."""

    parser = argparse.ArgumentParser(description='Autonomous Mapping Exploration')

    # ---- General ----
    parser.add_argument('--exp_name', type=str, default='exploration',
                        help='experiment name')
    parser.add_argument('--dump_location', type=str, default='./exploration_output',
                        help='path to dump logs and data')
    parser.add_argument('--seed', type=int, default=1,
                        help='random seed')

    # ---- Robot ----
    parser.add_argument('--robot_name', type=str, default='car1',
                        help='ROS robot namespace')
    parser.add_argument('--num_robots', type=int, default=1,
                        help='number of robots (fixed to 1 for single robot)')

    # ---- Map ----
    parser.add_argument('--map_size_cm', type=int, default=2500,
                        help='map size in cm (2500 = 25m)')
    parser.add_argument('--map_resolution', type=int, default=5,
                        help='map resolution in cm/pixel')
    parser.add_argument('--unit_size_cm', type=int, default=5,
                        help='unit cell size in cm')

    # ---- Exploration ----
    parser.add_argument('--max_episode_length', type=int, default=500,
                        help='maximum steps per episode')
    parser.add_argument('--max_collisions_allowed', type=int, default=1000,
                        help='max collisions before termination')
    parser.add_argument('--action_repeat', type=int, default=1,
                        help='number of times to repeat each action')
    parser.add_argument('--step_delay', type=float, default=0.5,
                        help='delay between steps in seconds')
    parser.add_argument('--linear_speed', type=float, default=0.2,
                        help='linear speed for exploration movements')
    parser.add_argument('--angular_speed', type=float, default=0.6,
                        help='angular speed for exploration movements')

    # ---- Frontier ----
    parser.add_argument('--min_frontier_size', type=int, default=5,
                        help='minimum frontier cluster size')
    parser.add_argument('--exploration_method', type=str, default='nearest',
                        choices=['nearest', 'largest', 'combined'],
                        help='frontier selection method')
    parser.add_argument('--threshold_near', type=float, default=0.5,
                        help='distance threshold for "near" frontiers')

    # ---- RL (optional) ----
    parser.add_argument('--use_rl', type=int, default=0,
                        help='whether to use RL-based goal selection')
    parser.add_argument('--load_global', type=str, default='0',
                        help='path to RL model checkpoint')
    parser.add_argument('--centralized', type=int, default=0,
                        help='whether to use centralized RL policy')

    # ---- Visualization ----
    parser.add_argument('--vis_type', type=int, default=1,
                        help='0: no vis, 1: show maps, 2: dump images')
    parser.add_argument('--save_data', type=int, default=1,
                        help='whether to save exploration data')

    # ---- Voronoi / Method ----
    parser.add_argument('--method', type=int, default=0,
                        help='0: nearest frontier, 1: voronoi-based')

    return parser


def get_default_args():
    """Get default args for exploration (no CLI parsing, for ROS integration)."""
    parser = get_exploration_args()
    args = parser.parse_args([])
    return args
