#!/usr/bin/env python3
"""Isolated ROS integration test: synthetic depth -> voxel maps -> make_plan.

Run ONLY on the private test master (port 11321). No hardware or NavOA controller
is launched. All move_base velocity output is remapped to /fusion_test/cmd_vel.
"""
import os
import sys
import time
import subprocess
import pathlib
import yaml
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from ground_segmentation_node import make_cloud, read_xyz


def main():
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11321':
        raise RuntimeError('This test requires the isolated master on 127.0.0.1:11321')
    root = pathlib.Path(sys.argv[1])
    config = root/'Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/config'
    script = pathlib.Path(__file__).parent/'ground_segmentation_node.py'
    rospy.init_node('fusion_integration_test')
    for scope in ('global_costmap', 'local_costmap'):
        for name in ('costmap_common_params.yaml', 'depth_obstacle_layers.yaml'):
            for key, value in yaml.safe_load((config/name).read_text()).items():
                rospy.set_param('/move_base/'+scope+'/'+key, value)
        for key, value in yaml.safe_load((config/(scope+'_params.yaml')).read_text()).items():
            existing = rospy.get_param('/move_base/'+key, {})
            existing.update(value)
            rospy.set_param('/move_base/'+key, existing)
    for key, value in {'base_global_planner': 'global_planner/GlobalPlanner',
                       'base_local_planner': 'base_local_planner/TrajectoryPlannerROS',
                       'controller_frequency': 1.0, 'planner_frequency': 0.0,
                       'recovery_behavior_enabled': False}.items():
        rospy.set_param('/move_base/'+key, value)
    for key, value in yaml.safe_load((config/'global_planner_params.yaml').read_text()).items():
        rospy.set_param('/move_base/'+key, value)
    # Camera looks along +base_x; same optical rotation as the deployed launch.
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []
    for parent, child, xyz, quat in [
            ('map', 'odom', (0, 0, 0), (0, 0, 0, 1)),
            ('odom', 'base_footprint', (5, -3, 0), (0, 0, 0, 1)),
            ('base_footprint', 'base_link', (0, 0, .69), (0, 0, 0, 1)),
            ('base_link', 'test_camera', (.1, 0, -.09), (-.5, .5, -.5, .5))]:
        t = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id=parent), child_frame_id=child)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        transforms.append(t)
    broadcaster.sendTransform(transforms)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    raw_pub = rospy.Publisher('/fusion_test/raw', PointCloud2, queue_size=1)
    maps, outputs = {}, {}
    def apply_update(update, scope):
        if scope not in maps:
            return
        m = maps[scope]
        a = np.asarray(m.data).reshape(m.info.height, m.info.width).copy()
        a[update.y:update.y+update.height, update.x:update.x+update.width] = np.asarray(update.data).reshape(update.height, update.width)
        m.data = a.ravel().tolist()
    subscribers = [rospy.Subscriber('/move_base/'+scope+'/costmap', OccupancyGrid,
                                    lambda m, s=scope: maps.__setitem__(s, m))
                   for scope in ('global_costmap', 'local_costmap')]
    subscribers += [rospy.Subscriber('/move_base/'+scope+'/costmap_updates', OccupancyGridUpdate,
                                     lambda m, s=scope: apply_update(m, s))
                    for scope in ('global_costmap', 'local_costmap')]
    subscribers += [rospy.Subscriber(topic, PointCloud2, lambda m, t=topic: outputs.__setitem__(t, m))
                    for topic in ('/obstacle_points', '/depth/clearing_points')]
    grid = OccupancyGrid(header=Header(stamp=rospy.Time.now(), frame_id='map'))
    grid.info.resolution = .05
    grid.info.width = grid.info.height = 240
    grid.info.origin.position.y = -6
    grid.info.origin.orientation.w = 1
    data = np.zeros((240, 240), dtype=np.int8)
    data[100:120, 170] = 100  # static wall at x=8.5, y=-1..0
    grid.data = data.ravel().tolist()
    map_pub.publish(grid)
    rospy.set_param('/ground_segmentation/input_pointcloud_topic', '/fusion_test/raw')
    rospy.set_param('/ground_segmentation/pixel_stride', 1)
    # Match the shipped launch defaults, keeping CPU math single-threaded.
    children = []
    try:
        children.append(subprocess.Popen(['/usr/bin/python3', str(script)]))
        children.append(subprocess.Popen(['/opt/ros/noetic/lib/move_base/move_base',
                                          '__name:=move_base', '/cmd_vel:=/fusion_test/cmd_vel']))
        deadline = time.monotonic()+20
        while len(maps) < 2 and time.monotonic() < deadline:
            map_pub.publish(grid)
            rospy.sleep(.2)
        assert len(maps) == 2, 'costmaps did not initialize'
        rospy.wait_for_service('/move_base/make_plan', timeout=15)

        # camera coords (optical x=-base_y, optical y=.6-base_z, optical z=base_x-.1)
        y, z = np.meshgrid(np.linspace(-.25, .25, 31), np.linspace(.1, 1.1, 41))
        box = np.c_[-y.ravel(), .6-z.ravel(), np.full(y.size, 1.2)]
        # Same rays, true background behind removed box, not artificial max-range fills.
        background = box*(4.0/1.2)
        xg, yg = np.meshgrid(np.linspace(.4, 2.6, 45), np.linspace(-.7, .7, 29))
        floor = np.c_[-yg.ravel(), np.full(xg.size, .6), xg.ravel()]

        def publish(points, duration=1.5):
            end = time.monotonic()+duration
            last = None
            while time.monotonic() < end:
                last = rospy.Time.now()
                raw_pub.publish(make_cloud(Header(stamp=last, frame_id='test_camera'), points))
                rospy.sleep(.13)
            rospy.sleep(.35)
            return last

        def occupied(scope):
            m = maps[scope]
            a = np.asarray(m.data).reshape(m.info.height, m.info.width)
            xx = m.info.origin.position.x+(np.arange(m.info.width)+.5)*m.info.resolution
            yy = m.info.origin.position.y+(np.arange(m.info.height)+.5)*m.info.resolution
            roi = a[np.ix_((yy > -3.35) & (yy < -2.65), (xx > 6.15) & (xx < 6.45))]
            return int(np.count_nonzero(roi == 100))

        publish(np.vstack([floor, box]), 2.5)
        counts = {s: occupied(s) for s in maps}
        assert all(n > 0 for n in counts.values()), 'depth missing: '+str(counts)
        assert outputs['/obstacle_points'].header.frame_id == 'test_camera'
        # Outputs come from one measurement; no transform+restamp misregistration.
        assert abs((outputs['/obstacle_points'].header.stamp-outputs['/depth/clearing_points'].header.stamp).to_sec()) < .2
        print('PASS: camera obstacle marked far from map origin:', counts, flush=True)

        request = GetPlanRequest()
        for pose, x in ((request.start, 5.0), (request.goal, 7.2)):
            pose.header = Header(frame_id='map', stamp=rospy.Time.now())
            pose.pose.position.x, pose.pose.position.y = x, -3
            pose.pose.orientation.w = 1
        request.tolerance = .1
        plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)(request).plan
        assert plan.poses and max(abs(p.pose.position.y+3) for p in plan.poses) > .3
        print('PASS: global planner detours around depth-only box; poses=', len(plan.poses), flush=True)

        publish(np.empty((0, 3)), 1.0)
        assert occupied('global_costmap') > 0, 'empty measurement wrongly erased obstacles'
        print('PASS: empty/unknown depth is not interpreted as free space', flush=True)

        publish(np.vstack([floor, background]), 3.0)
        cleared = {s: occupied(s) for s in maps}
        assert all(n == 0 for n in cleared.values()), 'ghost voxels remain: '+str(cleared)
        print('PASS: observed background rays clear removed box:', cleared, flush=True)
        global_grid = maps['global_costmap']
        static = np.asarray(global_grid.data).reshape(240, 240)
        assert np.all(static[100:120, 170] == 100), 'static map wall was erased'
        plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)(request).plan
        assert plan.poses and max(abs(p.pose.position.y+3) for p in plan.poses) < .15
        print('PASS: static wall retained and direct route restored after clearing', flush=True)
        print('ALL INTEGRATION CHECKS PASSED', flush=True)
    finally:
        for child in children:
            child.terminate()
        for child in children:
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
