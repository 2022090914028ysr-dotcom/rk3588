#!/usr/bin/python3
"""
FMM Planner - Fast Marching Method for path planning on occupancy grids.
Used for computing distances and planning paths to frontier goals.
"""

import numpy as np
import cv2
try:
    import skfmm
    HAS_SKFMM = True
except ImportError:
    HAS_SKFMM = False


class FMMPlanner:
    """Fast Marching Method planner for grid-based path planning."""

    def __init__(self, obstacle_map, unit_size_cm=5):
        self.unit_size_cm = unit_size_cm
        self.obstacle_map = obstacle_map
        self.map_size = obstacle_map.shape[0]

    def set_obstacles(self, obstacle_map):
        """Update the obstacle map."""
        self.obstacle_map = obstacle_map

    def plan(self, start, goal, vis_flag=False):
        """
        Plan a path from start to goal using FMM.

        Args:
            start: (x, y) tuple in grid coordinates
            goal: (x, y) tuple in grid coordinates
            vis_flag: whether to visualize

        Returns:
            path: list of (x, y) waypoints, or None if no path found
        """
        if not HAS_SKFMM:
            return self._dijkstra_plan(start, goal)

        try:
            # Build traversable mask
            traversable = (self.obstacle_map == 0).astype(np.float64)
            traversable[traversable == 0] = np.inf
            traversable[traversable > 0] = 0

            # Set goal as source
            gx, gy = int(goal[0]), int(goal[1])
            if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                traversable[gy, gx] = -1

            # Run FMM
            distance = skfmm.distance(traversed := traversable.copy())
            if hasattr(distance, 'data'):
                distance = distance.data
            if distance is None:
                return None

            # Gradient descent from start to goal
            sx, sy = int(start[0]), int(start[1])
            if not (0 <= sx < self.map_size and 0 <= sy < self.map_size):
                return None

            path = [(sx, sy)]
            current = np.array([sx, sy], dtype=np.float64)
            max_steps = self.map_size * 2

            for _ in range(max_steps):
                cx, cy = int(current[0]), int(current[1])
                if abs(cx - gx) <= 1 and abs(cy - gy) <= 1:
                    path.append((gx, gy))
                    break

                # Find neighbor with minimum distance
                min_dist = float('inf')
                best = (cx, cy)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                               (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                        d = distance[ny, nx]
                        if d < min_dist:
                            min_dist = d
                            best = (nx, ny)

                if best == (cx, cy):
                    break  # Stuck
                path.append(best)
                current = np.array(best, dtype=np.float64)
                if min_dist == 0:
                    break

            if len(path) < 2:
                return None

            return self._smooth_path(path)

        except Exception as e:
            print(f"[FMM] skfmm failed: {e}, falling back to Dijkstra")
            return self._dijkstra_plan(start, goal)

    def _dijkstra_plan(self, start, goal):
        """Fallback Dijkstra-based planner."""
        import heapq

        sx, sy = int(start[0]), int(start[1])
        gx, gy = int(goal[0]), int(goal[1])

        h, w = self.obstacle_map.shape
        visited = np.full((h, w), False)
        dist = np.full((h, w), np.inf)
        parent = np.full((h, w, 2), -1, dtype=np.int32)

        dist[sy, sx] = 0
        pq = [(0, sx, sy)]

        while pq:
            d, x, y = heapq.heappop(pq)
            if visited[y, x]:
                continue
            visited[y, x] = True
            if x == gx and y == gy:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx]:
                    if self.obstacle_map[ny, nx] > 0:
                        continue
                    nd = d + (1.414 if dx != 0 and dy != 0 else 1.0)
                    if nd < dist[ny, nx]:
                        dist[ny, nx] = nd
                        parent[ny, nx] = [x, y]
                        heapq.heappush(pq, (nd, nx, ny))

        if dist[gy, gx] == np.inf:
            return None

        # Reconstruct path
        path = [(gx, gy)]
        x, y = gx, gy
        while (x, y) != (sx, sy):
            px, py = parent[y, x]
            if px < 0 or py < 0:
                break
            path.append((px, py))
            x, y = px, py
        path.reverse()
        return self._smooth_path(path)

    def _smooth_path(self, path, max_points=50):
        """Simplify path by removing unnecessary waypoints."""
        if len(path) <= 2:
            return path

        # Simple line-of-sight simplification
        simplified = [path[0]]
        last_kept = 0

        for i in range(1, len(path)):
            if i == len(path) - 1:
                simplified.append(path[i])
                break
            if not self._line_of_sight(path[last_kept], path[i + 1]):
                simplified.append(path[i])
                last_kept = i

        # Downsample if too long
        if len(simplified) > max_points:
            indices = np.linspace(0, len(simplified) - 1, max_points, dtype=int)
            simplified = [simplified[i] for i in indices]

        return simplified

    def _line_of_sight(self, p1, p2):
        """Check if there's a clear line of sight between two points."""
        x1, y1 = int(p1[0]), int(p1[1])
        x2, y2 = int(p2[0]), int(p2[1])

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x2:
                if 0 <= x < self.map_size and 0 <= y < self.map_size:
                    if self.obstacle_map[y, x] > 0:
                        return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                if 0 <= x < self.map_size and 0 <= y < self.map_size:
                    if self.obstacle_map[y, x] > 0:
                        return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        return True

    def get_short_term_goal(self, start, global_goal, max_dist=50):
        """
        Get a short-term goal on the path towards the global goal.

        Returns:
            (x, y) short-term goal in grid coordinates
        """
        path = self.plan(start, (global_goal[0], global_goal[1]))
        if path is None or len(path) < 2:
            return start

        # Find point at max_dist along path
        dist_so_far = 0
        prev = path[0]
        for i in range(1, len(path)):
            curr = path[i]
            seg_dist = np.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            if dist_so_far + seg_dist >= max_dist:
                # Interpolate
                if seg_dist > 0:
                    t = (max_dist - dist_so_far) / seg_dist
                    x = prev[0] + t * (curr[0] - prev[0])
                    y = prev[1] + t * (curr[1] - prev[1])
                    return (x, y)
                return prev
            dist_so_far += seg_dist
            prev = curr

        return path[-1]
