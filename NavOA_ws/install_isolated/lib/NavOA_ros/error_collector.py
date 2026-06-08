#!/usr/bin/env python3
import os
import rospy
import rospkg
from rosgraph_msgs.msg import Log
from datetime import datetime


class ErrorCollector(object):
    def __init__(self):
        rospy.init_node('error_collector')

        self.pkg_root = rospkg.RosPack().get_path('NavOA_ros')
        self.log_path = rospy.get_param('~log_path', os.path.join(os.path.dirname(self.pkg_root), 'error.md'))
        self.filter_nodes = rospy.get_param('~filter_nodes', [])
        self.filter_text = rospy.get_param('~filter_text', ['Cartographer', 'cartographer', 'reset', 'relocaliz', 'error'])
        self.ignore_move_base_errors = rospy.get_param('~ignore_move_base_errors', True)
        self.ignore_nodes = rospy.get_param('~ignore_nodes', [])
        if self.ignore_move_base_errors and '/move_base' not in self.ignore_nodes:
            self.ignore_nodes.append('/move_base')
        self.severity_levels = rospy.get_param('~severity_levels', [4, 8, 16])
        self.max_entries = rospy.get_param('~max_entries', 1000)

        self.ensure_log_file()
        self.entry_count = 0

        rospy.loginfo('ErrorCollector writing to %s', self.log_path)
        rospy.Subscriber('/rosout', Log, self.on_rosout, queue_size=100)
        rospy.spin()

    def ensure_log_file(self):
        directory = os.path.dirname(self.log_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w') as f:
                f.write('# Error Log\n\n')
                f.write('Generated: %s\n\n' % datetime.now().isoformat())

    def on_rosout(self, msg):
        if msg.level not in self.severity_levels:
            return

        if self.ignore_nodes and msg.name in self.ignore_nodes:
            return

        if self.filter_nodes and msg.name not in self.filter_nodes:
            return

        if self.filter_text and not any(term.lower() in msg.msg.lower() for term in self.filter_text):
            return

        entry = self.format_entry(msg)
        self.write_entry(entry)

    def format_entry(self, msg):
        time_str = datetime.fromtimestamp(msg.header.stamp.to_sec()).strftime('%Y-%m-%d %H:%M:%S.%f')
        level = self.level_name(msg.level)
        entry = [
            '---',
            f'**Time:** {time_str}',
            f'**Level:** {level}',
            f'**Node:** {msg.name}',
            f'**File:** {msg.file}:{msg.line}',
            f'**Message:**',
            '```',
            msg.msg,
            '```',
        ]
        return '\n'.join(entry)

    def write_entry(self, entry):
        with open(self.log_path, 'a') as f:
            f.write(entry + '\n')
        self.entry_count += 1
        if self.entry_count > self.max_entries:
            rospy.logwarn('ErrorCollector has logged more than %d entries', self.max_entries)
            self.entry_count = 0

    @staticmethod
    def level_name(level):
        if level == 1:
            return 'DEBUG'
        if level == 2:
            return 'INFO'
        if level == 4:
            return 'WARN'
        if level == 8:
            return 'ERROR'
        if level == 16:
            return 'FATAL'
        return str(level)


if __name__ == '__main__':
    try:
        ErrorCollector()
    except rospy.ROSInterruptException:
        pass
