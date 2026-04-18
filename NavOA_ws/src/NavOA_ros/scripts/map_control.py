#!/usr/bin/env python3
import os
import signal
import subprocess
import time

import rospy
import rospkg
from cartographer_ros_msgs.srv import WriteState, WriteStateRequest
from std_msgs.msg import String


class MapControl:
    def __init__(self):
        self.mapping_process = None
        self.save_path = rospy.get_param('~save_path', self.default_save_path())
        self.map_name = rospy.get_param('~map_name', 'limo_map')
        self.mapping_launch = rospy.get_param('~mapping_launch', self.default_launch_path())
        self.save_ros_map = rospy.get_param('~save_ros_map', False)
        self.status_pub = rospy.Publisher('/app/map_status', String, queue_size=1, latch=True)
        self.command_sub = rospy.Subscriber('/app/map_command', String, self.command_callback)
        self.publish_status('idle')

    def default_save_path(self):
        root = rospkg.RosPack().get_path('NavOA_ros')
        return os.path.join(root, 'example', 'gazebo_limo', 'maps')

    def default_launch_path(self):
        root = rospkg.RosPack().get_path('NavOA_ros')
        return os.path.join(root, 'example', 'gazebo_limo', 'launch', 'carto_slam.launch')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo('[map_control] status=%s', status)

    def command_callback(self, msg):
        if not msg or not msg.data:
            return
        command = msg.data.strip()
        normalized = command.lower().replace('_', ' ').replace(':', ' ').strip()
        tokens = normalized.split()
        if not tokens:
            return

        if tokens[0] == 'start':
            self.start_mapping()
            return
        if tokens[0] == 'stop':
            self.stop_mapping()
            return
        if tokens[0] == 'save':
            if len(tokens) == 1:
                self.save_map()
                return
            if tokens[1] in ('map', 'full'):
                filename = ' '.join(command.split()[2:]).strip() if len(command.split()) > 2 else None
                self.save_map(filename, save_ros_map=True)
                return
            if tokens[1] in ('pb', 'pbstream', 'stream'):
                filename = ' '.join(command.split()[2:]).strip() if len(command.split()) > 2 else None
                self.save_map(filename, save_ros_map=False)
                return
            filename = command.split(None, 1)[1].strip()
            self.save_map(filename, save_ros_map=True)
            return

        filename = command.replace(' ', '_').strip()
        if filename:
            self.save_map(filename, save_ros_map=True)
        else:
            self.publish_status('error: unsupported command')
    def start_mapping(self):
        if self.mapping_process is not None and self.mapping_process.poll() is None:
            self.publish_status('mapping already running')
            return

        if not os.path.exists(self.mapping_launch):
            self.publish_status('error: mapping launch not found')
            rospy.logerr('Mapping launch file does not exist: %s', self.mapping_launch)
            return

        self.publish_status('starting mapping')
        cmd = ['roslaunch', self.mapping_launch]
        try:
            self.mapping_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.publish_status('mapping started')
            rospy.loginfo('[map_control] started mapping process pid=%s', self.mapping_process.pid)
        except Exception as e:
            self.publish_status('error: cannot start mapping')
            rospy.logerr('[map_control] failed to start mapping: %s', e)

    def stop_mapping(self):
        if self.mapping_process is None or self.mapping_process.poll() is not None:
            self.publish_status('mapping not running')
            return

        self.publish_status('stopping mapping')
        try:
            self.mapping_process.send_signal(signal.SIGINT)
            timeout = 10
            start = time.time()
            while time.time() - start < timeout and self.mapping_process.poll() is None:
                time.sleep(0.1)
            if self.mapping_process.poll() is None:
                self.mapping_process.terminate()
            self.mapping_process = None
            self.publish_status('mapping stopped')
            rospy.loginfo('[map_control] mapping stopped')
        except Exception as e:
            self.publish_status('error: cannot stop mapping')
            rospy.logerr('[map_control] failed to stop mapping: %s', e)

    def save_map(self, filename=None, save_ros_map=None):
        if save_ros_map is None:
            save_ros_map = self.save_ros_map
        if filename is None:
            filename = self.default_map_filename()
        filename = filename.strip()
        if not filename.endswith('.pbstream'):
            filename = filename + '.pbstream'

        full_path = os.path.join(self.save_path, filename)
        os.makedirs(self.save_path, exist_ok=True)

        self.publish_status('saving map')
        rospy.loginfo('[map_control] saving map to %s', full_path)
        try:
            rospy.wait_for_service('/write_state', timeout=30)
            write_state = rospy.ServiceProxy('/write_state', WriteState)
            req = WriteStateRequest()
            req.filename = full_path
            req.include_unfinished_submaps = True
            result = write_state(req)
            if result and hasattr(result, 'status') and hasattr(result.status, 'code') and result.status.code == 0:
                if save_ros_map:
                    msg = 'saved full map package:' + full_path
                else:
                    msg = 'saved pbstream:' + full_path
                self.publish_status(msg)
                rospy.loginfo('[map_control] write_state result: %s', result.status.message)
                if save_ros_map:
                    self.save_ros_occupancy(full_path)
                return True
            else:
                error_msg = 'error: save failed'
                if result and hasattr(result, 'status'):
                    error_msg = 'error: ' + result.status.message
                self.publish_status(error_msg)
                rospy.logerr('[map_control] write_state failed: %s', getattr(result.status, 'message', 'unknown'))
                return False
        except Exception as e:
            self.publish_status('error: save failed')
            rospy.logerr('[map_control] save_map exception: %s', e)
            return False

    def default_map_filename(self):
        now = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        return f'{self.map_name}_{now}.pbstream'

    def save_ros_occupancy(self, pbstream_path):
        file_base = os.path.splitext(os.path.basename(pbstream_path))[0]
        output_path = os.path.join(self.save_path, file_base)
        cmd = ['rosrun', 'map_server', 'map_saver', '-f', output_path]
        rospy.loginfo('[map_control] saving ROS occupancy grid: %s', ' '.join(cmd))
        try:
            subprocess.check_call(cmd)
            self.publish_status('saved map_server:' + output_path)
            rospy.loginfo('[map_control] map_server saved: %s', output_path)
            self.update_current_map_symlinks(pbstream_path, output_path)
        except Exception as e:
            self.publish_status('error: map_server save failed')
            rospy.logerr('[map_control] map_server save failed: %s', e)

    def update_current_map_symlinks(self, pbstream_path, output_path):
        current_base = os.path.join(self.save_path, 'current_map')
        symlink_pairs = [
            (pbstream_path, current_base + '.pbstream'),
            (output_path + '.pgm', current_base + '.pgm'),
            (output_path + '.yaml', current_base + '.yaml'),
        ]
        for target, link in symlink_pairs:
            try:
                if os.path.islink(link) or os.path.exists(link):
                    os.remove(link)
                os.symlink(target, link)
                rospy.loginfo('[map_control] updated symlink %s -> %s', link, target)
            except Exception as e:
                rospy.logwarn('[map_control] failed to update symlink %s -> %s: %s', link, target, e)


if __name__ == '__main__':
    rospy.init_node('map_control', anonymous=False)
    MapControl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
