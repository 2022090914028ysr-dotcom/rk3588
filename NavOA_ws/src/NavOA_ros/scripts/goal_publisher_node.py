#!/usr/bin/env python3
import os
import math
import yaml
import random
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import TransformException


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_goals(raw):
    goals = []
    if raw is None:
        return goals
    # Accept list of [x,y,yaw] or dicts with keys
    for item in raw:
        if isinstance(item, dict):
            x = float(item.get('x', 0.0))
            y = float(item.get('y', 0.0))
            yaw = float(item.get('yaw', 0.0))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            x = float(item[0])
            y = float(item[1])
            yaw = float(item[2]) if len(item) >= 3 else 0.0
        else:
            rospy.logwarn(f"[goal_publisher] Unsupported goal format: {item}")
            continue
        goals.append((x, y, yaw))
    return goals


def load_goals_from_yaml(path):
    if not path:
        return []
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        # YAML may be {'goals': [[x,y,yaw], ...]} or just list
        if isinstance(data, dict) and 'goals' in data:
            return normalize_goals(data['goals'])
        return normalize_goals(data)
    except Exception as e:
        rospy.logerr(f"[goal_publisher] Failed to read YAML '{path}': {e}")
        return []


def make_pose(frame_id: str, x: float, y: float, yaw: float):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class MapCache:
    def __init__(self):
        self.map = None

    def cb(self, msg: OccupancyGrid):
        self.map = msg

    def ready(self):
        return self.map is not None

    def cell_world_xy(self, cx: int, cy: int):
        info = self.map.info
        x = info.origin.position.x + (cx + 0.5) * info.resolution
        y = info.origin.position.y + (cy + 0.5) * info.resolution
        return x, y

    def frontier_free_cells(self):
        m = self.map
        info = m.info
        w, h = info.width, info.height
        data = m.data

        def val(ix, iy):
            if ix < 0 or iy < 0 or ix >= w or iy >= h:
                return 100
            return data[iy * w + ix]

        cells = []
        for cy in range(h):
            row_off = cy * w
            for cx in range(w):
                v = data[row_off + cx]
                if v != 0:
                    continue
                if (val(cx - 1, cy) == -1 or
                    val(cx + 1, cy) == -1 or
                    val(cx, cy - 1) == -1 or
                    val(cx, cy + 1) == -1):
                    cells.append((cx, cy))
        return cells

    def filter_bbox(self, cells, bbox):
        if not cells:
            return []
        if not bbox or len(bbox) < 4:
            return cells
        xmin, xmax, ymin, ymax = bbox
        out = []
        for cx, cy in cells:
            x, y = self.cell_world_xy(cx, cy)
            if x >= xmin and x <= xmax and y >= ymin and y <= ymax:
                out.append((cx, cy))
        return out


def main():
    rospy.init_node('goal_publisher')

    target_topic = rospy.get_param('~target_topic', '/move_base_simple/goal')
    frame_id = rospy.get_param('~frame_id', 'map')
    interval_sec = float(rospy.get_param('~interval_sec', 5.0))
    repeat = bool(rospy.get_param('~repeat', False))
    once = bool(rospy.get_param('~once', False))

    # Sampling & arrival monitoring params
    mode = str(rospy.get_param('~mode', 'list')).lower()  # 'list' or 'sample_unknown'
    map_topic = rospy.get_param('~map_topic', '/map')
    area_bbox = rospy.get_param('~area_bbox', [])  # [xmin, xmax, ymin, ymax]
    sample_count = int(rospy.get_param('~sample_count', 5))
    arrival_tolerance = float(rospy.get_param('~arrival_tolerance', 0.3))
    timeout_sec = float(rospy.get_param('~timeout_sec', 30.0))
    republish_on_timeout = bool(rospy.get_param('~republish_on_timeout', True))
    base_frame = rospy.get_param('~base_frame', 'base_link')

    goals_param = rospy.get_param('~goals', None)
    yaml_path = rospy.get_param('~goal_list_yaml', '')

    goals = []
    # Prefer explicit goals param; fall back to YAML, or sample from frontier
    if mode == 'list':
        goals.extend(normalize_goals(goals_param) or [])
        if not goals:
            goals.extend(load_goals_from_yaml(yaml_path) or [])
    else:
        cache = MapCache()
        rospy.Subscriber(map_topic, OccupancyGrid, cache.cb, queue_size=1)
        rospy.loginfo(f"[goal_publisher] Waiting for map on '{map_topic}' to sample goals...")
        rate_wait = rospy.Rate(10)
        while not rospy.is_shutdown() and not cache.ready():
            rate_wait.sleep()
        if not cache.ready():
            rospy.logerr('[goal_publisher] Map not received; cannot sample goals.')
            return
        frontier_cells = cache.frontier_free_cells()
        frontier_cells = cache.filter_bbox(frontier_cells, area_bbox)
        if not frontier_cells:
            rospy.logwarn('[goal_publisher] No frontier cells found (check map and bbox).')
        else:
            random.shuffle(frontier_cells)
            pick = frontier_cells[:max(1, sample_count)]
            for cx, cy in pick:
                x, y = cache.cell_world_xy(cx, cy)
                goals.append((x, y, 0.0))

    if not goals:
        rospy.logerr('[goal_publisher] No goals provided via ~goals or ~goal_list_yaml')
        return

    pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
    rospy.sleep(0.5)  # allow publisher setup

    idx = 0
    rate = rospy.Rate(10)
    last_sent = rospy.Time(0)
    sent_once = False

    rospy.loginfo(f"[goal_publisher] Publishing to '{target_topic}' with frame_id '{frame_id}'")
    rospy.loginfo(f"[goal_publisher] Loaded {len(goals)} goals (mode={mode})")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # Maintain current goal, arrival monitoring, and timeout handling
        if (now - last_sent).to_sec() >= interval_sec:
            x, y, yaw = goals[idx]
            pose = make_pose(frame_id, x, y, yaw)
            pub.publish(pose)
            rospy.loginfo(f"[goal_publisher] Sent goal #{idx+1}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
            last_sent = now
            goal_sent_time = now
            current_goal = (x, y, yaw)
            if once:
                sent_once = True

        # TF-based arrival monitoring
        try:
            tf_buffer = getattr(rospy, '_goal_pub_tf_buffer', None)
            if tf_buffer is None:
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                rospy._goal_pub_tf_buffer = tf_buffer
                rospy._goal_pub_tf_listener = tf_listener
            t = tf_buffer.lookup_transform(frame_id, base_frame, rospy.Time(0), rospy.Duration(0.2))
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            gx, gy, _ = goals[idx]
            dist = math.hypot(rx - gx, ry - gy)
            if dist <= arrival_tolerance:
                rospy.loginfo(f"[goal_publisher] Arrived at goal #{idx+1} (dist={dist:.2f} <= {arrival_tolerance})")
                if sent_once:
                    break
                idx += 1
                if idx >= len(goals):
                    if repeat:
                        idx = 0
                    else:
                        rospy.loginfo('[goal_publisher] All goals completed; exiting.')
                        break
            else:
                # Timeout handling
                if 'goal_sent_time' in locals() and (now - goal_sent_time).to_sec() >= timeout_sec:
                    if republish_on_timeout:
                        pose = make_pose(frame_id, gx, gy, 0.0)
                        pub.publish(pose)
                        goal_sent_time = now
                        rospy.logwarn(f"[goal_publisher] Timeout at goal #{idx+1}; re-published.")
                    else:
                        rospy.logwarn(f"[goal_publisher] Timeout at goal #{idx+1}; skipping to next.")
                        idx += 1
                        if idx >= len(goals):
                            if repeat:
                                idx = 0
                            else:
                                rospy.loginfo('[goal_publisher] All goals attempted; exiting.')
                                break
        except TransformException:
            # TF not ready; skip arrival check this cycle
            pass
        rate.sleep()

    if sent_once:
        rospy.loginfo('[goal_publisher] Single goal sent; exiting.')


if __name__ == '__main__':
    main()
