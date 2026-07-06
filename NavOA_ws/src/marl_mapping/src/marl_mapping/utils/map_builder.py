#!/usr/bin/python3
"""
Map Builder utilities for processing ROS OccupancyGrid into exploration maps.
"""

import numpy as np
import cv2


class MapBuilder:
    """Process occupancy grid maps for autonomous exploration."""

    def __init__(self, map_size_cm=2500, map_resolution=5, unit_size_cm=5):
        self.map_size_cm = map_size_cm
        self.map_resolution = map_resolution
        self.unit_size_cm = unit_size_cm
        self.map_size = map_size_cm // map_resolution

    def process_map(self, occupancy_grid, map_info=None):
        """
        Process ROS OccupancyGrid data into exploration maps.

        Args:
            occupancy_grid: 2D numpy array of occupancy values (-1=unknown, 0=free, 100=occupied)
            map_info: Optional OccupancyGrid info message

        Returns:
            dict with keys: 'obstacle', 'explored', 'free'
        """
        result = {}

        # Initialize maps at target size
        target_size = (self.map_size, self.map_size)
        obstacle = np.zeros(target_size, dtype=np.uint8)
        explored = np.zeros(target_size, dtype=np.uint8)
        free = np.zeros(target_size, dtype=np.uint8)

        if occupancy_grid is None:
            return {'obstacle': obstacle, 'explored': explored, 'free': free}

        h, w = occupancy_grid.shape

        # Compute scaling
        if map_info is not None:
            scale = map_info.resolution * 100.0 / self.map_resolution  # m to cm, then / resolution
        else:
            scale = 5.0 / self.map_resolution  # Default: 5cm res

        # Resize if needed
        if h != self.map_size or w != self.map_size:
            new_h = min(int(h * scale), self.map_size)
            new_w = min(int(w * scale), self.map_size)
            if new_h > 0 and new_w > 0:
                resized = cv2.resize(
                    occupancy_grid.astype(np.float32),
                    (new_w, new_h),
                    interpolation=cv2.INTER_NEAREST
                )
                # Center in target map
                y_start = (self.map_size - new_h) // 2
                x_start = (self.map_size - new_w) // 2
                grid = np.full(target_size, -1, dtype=np.float32)
                grid[y_start:y_start + new_h, x_start:x_start + new_w] = resized
            else:
                grid = np.full(target_size, -1, dtype=np.float32)
        else:
            grid = occupancy_grid.astype(np.float32)

        # Build maps
        obstacle[:] = (grid > 50).astype(np.uint8)  # Occupied
        explored[:] = (grid >= 0).astype(np.uint8)   # Known (free + occupied)
        free[:] = (grid == 0).astype(np.uint8)        # Free space

        # Erode obstacles slightly for safety margin
        kernel = np.ones((3, 3), dtype=np.uint8)
        obstacle = cv2.dilate(obstacle, kernel, iterations=1)

        return {
            'obstacle': obstacle,
            'explored': explored,
            'free': free
        }

    def compute_frontier(self, obstacle_map, explored_map):
        """Detect frontiers: explored free cells adjacent to unknown cells."""
        kernel = np.ones((3, 3), dtype=np.uint8)
        safe_obstacle = cv2.dilate(obstacle_map.astype(np.uint8), kernel, iterations=2)

        unknown = (explored_map == 0).astype(np.uint8)
        dilated_unknown = cv2.dilate(unknown, kernel, iterations=1)

        frontier = np.logical_and(
            np.logical_and(explored_map > 0, safe_obstacle == 0),
            dilated_unknown > 0
        ).astype(np.uint8)

        return frontier

    def get_explorable_mask(self, obstacle_map):
        """Get mask of cells that are explorable (not obstacles)."""
        return (obstacle_map == 0).astype(np.uint8)
