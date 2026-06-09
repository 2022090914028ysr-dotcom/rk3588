#!/usr/bin/python3
"""
Frontier Detection and Voronoi-based Goal Selection
Adapted from NeuralCoMapping's frontier.py for single robot.
"""

import numpy as np
import cv2
import copy
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import KMeans


def detect_frontiers(obstacle_map, explored_map, min_frontier_size=5):
    """
    Detect frontier clusters from occupancy and exploration maps.

    Args:
        obstacle_map: binary map (1=obstacle, 0=free)
        explored_map: binary map (1=explored, 0=unknown)
        min_frontier_size: minimum cluster size to consider

    Returns:
        list of (cx, cy, size, white_border) for each frontier cluster.
        white_border = number of explored-free cells adjacent to this
        frontier cluster (higher = more "mature" / better known area).
    """
    h, w = obstacle_map.shape

    # Compute raw frontiers
    kernel = np.ones((3, 3), dtype=np.uint8)
    safe_obstacle = cv2.dilate(obstacle_map.astype(np.uint8), kernel, iterations=2)
    unknown = (explored_map == 0).astype(np.uint8)
    dilated_unknown = cv2.dilate(unknown, kernel, iterations=1)

    frontier = np.logical_and(
        np.logical_and(explored_map > 0, safe_obstacle == 0),
        dilated_unknown > 0
    ).astype(np.uint8)

    # White (explored-free) map for border counting
    white = ((explored_map > 0) & (obstacle_map == 0)).astype(np.uint8)

    # Find connected components
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        frontier, connectivity=8
    )

    frontiers = []
    for i in range(1, num_labels):  # Skip background (label 0)
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_frontier_size:
            continue
        cx, cy = int(centroids[i][0]), int(centroids[i][1])

        # ---- Count adjacent white (explored-free) cells ----
        # Operate on the cluster's bounding box + 1px padding for speed
        x1 = max(0, stats[i, cv2.CC_STAT_LEFT] - 1)
        y1 = max(0, stats[i, cv2.CC_STAT_TOP] - 1)
        x2 = min(w, stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] + 1)
        y2 = min(h, stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] + 1)

        local_labels = labels[y1:y2, x1:x2]
        cluster_mask = (local_labels == i).astype(np.uint8)
        cluster_dilated = cv2.dilate(cluster_mask, kernel, iterations=1)

        local_white = white[y1:y2, x1:x2]
        # White cells in the dilated ring that are NOT the cluster itself
        border = local_white & cluster_dilated & (1 - cluster_mask)
        white_border = int(np.sum(border))

        frontiers.append((cx, cy, area, white_border))

    # Sort by size (largest first)
    frontiers.sort(key=lambda x: x[2], reverse=True)
    return frontiers


def select_best_frontier(robot_pos, frontiers, obstacle_map, method='nearest'):
    """
    Select the best frontier for the robot to explore.

    Args:
        robot_pos: (x, y) robot position in grid coordinates
        frontiers: list of (cx, cy, size) frontier clusters
        obstacle_map: binary obstacle map
        method: 'nearest', 'largest', 'combined'

    Returns:
        (cx, cy) of selected frontier, or None
    """
    if not frontiers:
        return None

    if method == 'nearest':
        # Select nearest frontier
        best = min(frontiers, key=lambda f: np.sqrt(
            (f[0] - robot_pos[0])**2 + (f[1] - robot_pos[1])**2))
        return (best[0], best[1])

    elif method == 'largest':
        # Select largest frontier
        best = max(frontiers, key=lambda f: f[2])
        return (best[0], best[1])

    elif method == 'combined':
        # Score = size / distance
        best = max(frontiers, key=lambda f: f[2] / max(np.sqrt(
            (f[0] - robot_pos[0])**2 + (f[1] - robot_pos[1])**2), 1.0))
        return (best[0], best[1])

    return None


def euclidean_voronoi_planning(exp_map, robot_locations, frontier_map, vis_flag=False):
    """
    Simple Euclidean distance-based Voronoi partitioning and goal selection.

    Args:
        exp_map: exploration map (0=free, 1=obstacle, 2=frontier, 3=unknown)
        robot_locations: list of (x, y) robot positions
        frontier_map: binary frontier map
        vis_flag: whether to save visualization

    Returns:
        global_goals: list of (x, y, radius) goals for each robot
    """
    num_robots = len(robot_locations)
    h, w = exp_map.shape

    # Get all frontier points
    frontier_points = np.argwhere(frontier_map > 0)  # (y, x) format
    if len(frontier_points) == 0:
        return [None] * num_robots

    frontier_points_xy = frontier_points[:, ::-1]  # Convert to (x, y)

    # For single robot: just pick the nearest frontier
    if num_robots == 1:
        rx, ry = robot_locations[0]
        distances = np.sqrt(
            (frontier_points_xy[:, 0] - rx)**2 +
            (frontier_points_xy[:, 1] - ry)**2
        )
        best_idx = np.argmin(distances)
        return [(int(frontier_points_xy[best_idx, 0]),
                 int(frontier_points_xy[best_idx, 1]), 10)]

    # For multiple robots: assign using Voronoi + Hungarian
    # Compute Voronoi regions based on Euclidean distance
    voronoi_map = np.full((h, w), -1, dtype=np.int32)
    for y in range(h):
        for x in range(w):
            dists = [np.sqrt((x - rx)**2 + (y - ry)**2)
                     for rx, ry in robot_locations]
            voronoi_map[y, x] = np.argmin(dists)

    # Assign frontiers to robots within their Voronoi regions
    goals = []
    for i in range(num_robots):
        rx, ry = robot_locations[i]
        region_frontiers = frontier_points_xy[voronoi_map[
            frontier_points[:, 0], frontier_points[:, 1]] == i]

        if len(region_frontiers) > 0:
            # Pick nearest frontier in region
            dists = np.sqrt(
                (region_frontiers[:, 0] - rx)**2 +
                (region_frontiers[:, 1] - ry)**2)
            best_idx = np.argmin(dists)
            goals.append((int(region_frontiers[best_idx, 0]),
                         int(region_frontiers[best_idx, 1]), 10))
        else:
            # Fallback: nearest frontier overall
            dists = np.sqrt(
                (frontier_points_xy[:, 0] - rx)**2 +
                (frontier_points_xy[:, 1] - ry)**2)
            best_idx = np.argmin(dists)
            goals.append((int(frontier_points_xy[best_idx, 0]),
                         int(frontier_points_xy[best_idx, 1]), 10))

    return goals


def greedy_frontier_planning(robot_pos, frontier_map, obstacle_map,
                              threshold_near=0.5, method='nearest'):
    """
    Greedy frontier-based goal planning for single robot.

    Args:
        robot_pos: (x, y) robot position
        frontier_map: binary frontier map
        obstacle_map: binary obstacle map
        threshold_near: distance threshold to consider as "near"
        method: 'nearest' or 'largest'

    Returns:
        (gx, gy, radius) goal
    """
    frontiers = detect_frontiers(obstacle_map,
                                  (frontier_map > 0).astype(np.uint8) | (obstacle_map == 0).astype(np.uint8),
                                  min_frontier_size=3)

    if not frontiers:
        return None

    best = select_best_frontier(robot_pos, frontiers, obstacle_map, method=method)
    if best is None:
        return None

    return (best[0], best[1], 10)


def grid_to_world(grid_pos, map_origin, resolution):
    """Convert grid coordinates to world coordinates."""
    wx = map_origin[0] + grid_pos[0] * resolution
    wy = map_origin[1] + grid_pos[1] * resolution
    return (wx, wy)


def world_to_grid(world_pos, map_origin, resolution):
    """Convert world coordinates to grid coordinates."""
    gx = int((world_pos[0] - map_origin[0]) / resolution)
    gy = int((world_pos[1] - map_origin[1]) / resolution)
    return (gx, gy)
