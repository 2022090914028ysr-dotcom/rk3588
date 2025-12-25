#!/usr/bin/env python3
import rospy
import math
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData
from collections import deque

class AutoExplorer:
    def __init__(self):
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_cb, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 5)
        self.unknown_complete_threshold = rospy.get_param('~unknown_complete_threshold', 0.05)  # 5%
        self.goal_interval = rospy.get_param('~goal_interval', 5.0)
        self.last_goal_time = rospy.Time(0)
        self.current_map = None
        self.sent_goals = 0
        self.max_goals = rospy.get_param('~max_goals', 0)  # 0 means unlimited

    def map_cb(self, msg: OccupancyGrid):
        self.current_map = msg
        now = rospy.Time.now()
        if (now - self.last_goal_time).to_sec() < self.goal_interval:
            return
        # Check completion
        unknown_ratio = self.compute_unknown_ratio(msg)
        if unknown_ratio <= self.unknown_complete_threshold:
            rospy.loginfo('[AutoExplorer] Exploration complete. Unknown ratio: %.3f', unknown_ratio)
            return
        if self.max_goals and self.sent_goals >= self.max_goals:
            rospy.loginfo('[AutoExplorer] Max goals reached (%d).', self.max_goals)
            return
        # Compute frontier goals
        goal = self.pick_frontier_goal(msg)
        if goal:
            self.publish_goal(goal)
            self.last_goal_time = now
            self.sent_goals += 1
        else:
            rospy.loginfo('[AutoExplorer] No frontier found. Unknown ratio: %.3f', unknown_ratio)

    def compute_unknown_ratio(self, grid: OccupancyGrid):
        data = grid.data
        total = len(data)
        unknown = sum(1 for v in data if v == -1)
        return unknown / float(total) if total else 1.0

    def index_to_xy(self, idx, info: MapMetaData):
        w = info.width
        x = idx % w
        y = idx // w
        return x, y

    def xy_to_world(self, x, y, info: MapMetaData):
        origin = info.origin
        wx = origin.position.x + (x + 0.5) * info.resolution
        wy = origin.position.y + (y + 0.5) * info.resolution
        return wx, wy

    def neighbors4(self, x, y, info: MapMetaData):
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < info.width and 0 <= ny < info.height:
                yield nx, ny

    def pick_frontier_goal(self, grid: OccupancyGrid):
        info = grid.info
        data = grid.data
        width, height = info.width, info.height
        # Frontier cell: unknown (-1) with at least one free neighbor (0)
        frontier = []
        for idx, v in enumerate(data):
            if v != -1:
                continue
            x, y = self.index_to_xy(idx, info)
            has_free_neighbor = any(data[ny*width+nx] == 0 for nx, ny in self.neighbors4(x, y, info))
            if has_free_neighbor:
                frontier.append((x, y))
        if not frontier:
            return None
        # Cluster frontier cells to avoid tiny/noisy points
        visited = set()
        clusters = []
        for fx, fy in frontier:
            if (fx, fy) in visited:
                continue
            q = deque([(fx, fy)])
            visited.add((fx, fy))
            cluster = []
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for nx, ny in self.neighbors4(cx, cy, info):
                    if (nx, ny) in visited:
                        continue
                    idx = ny*width + nx
                    if data[idx] == -1:
                        # still unknown; use 8-neighbors for tighter cluster growth
                        for ddx, ddy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                            mx, my = cx+ddx, cy+ddy
                            if 0 <= mx < width and 0 <= my < height and data[my*width+mx] == -1:
                                pass
                        visited.add((nx, ny))
                        q.append((nx, ny))
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)
        if not clusters:
            # fallback: pick any frontier
            fx, fy = frontier[0]
            wx, wy = self.xy_to_world(fx, fy, info)
            return Point(wx, wy, 0.0)
        # Choose cluster center closest to robot (assume robot at map origin or use tf for base pose)
        # Try to get robot pose via TF (map -> base_link)
        base_pose = None
        try:
            listener = tf.TransformListener()
            listener.waitForTransform(self.frame_id, 'base_link', rospy.Time(0), rospy.Duration(0.2))
            (trans, rot) = listener.lookupTransform(self.frame_id, 'base_link', rospy.Time(0))
            base_pose = (trans[0], trans[1])
        except Exception:
            base_pose = (0.0, 0.0)
        best_dist = float('inf')
        best_point = None
        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / float(len(cluster))
            cy = sum(p[1] for p in cluster) / float(len(cluster))
            wx, wy = self.xy_to_world(int(cx), int(cy), info)
            d = (wx - base_pose[0])**2 + (wy - base_pose[1])**2
            if d < best_dist:
                best_dist = d
                best_point = Point(wx, wy, 0.0)
        return best_point

    def publish_goal(self, point: Point):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = point.x
        msg.pose.position.y = point.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        rospy.loginfo('[AutoExplorer] Publishing goal: (%.2f, %.2f)', point.x, point.y)
        self.goal_pub.publish(msg)


def main():
    rospy.init_node('auto_explorer')
    AutoExplorer()
    rospy.loginfo('[AutoExplorer] Started. Waiting for map...')
    rospy.spin()

if __name__ == '__main__':
    main()
