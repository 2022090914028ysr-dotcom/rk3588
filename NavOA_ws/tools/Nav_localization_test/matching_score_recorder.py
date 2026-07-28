#!/usr/bin/env python3
"""
ROS node to record /matching_score and /runner/status to CSV.

Usage:
    rosrun Nav_localization_test matching_score_recorder.py
        or
    python3 matching_score_recorder.py    (if ROS_MASTER_URI is set)

Records to: matching_score_<datetime>.csv
Format:     timestamp,score,mode

Press Ctrl+C to stop.
"""

import os
import sys
import csv
import signal
import datetime
import rospy
from std_msgs.msg import Float32, String


class MatchingScoreRecorder:
    """Subscribes to /matching_score and logs to CSV."""

    def __init__(self):
        output_dir = rospy.get_param('~output_dir', os.path.dirname(os.path.abspath(__file__)))
        topic = rospy.get_param('~topic', '/matching_score')

        # Create output filename with timestamp
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(output_dir, f'matching_score_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'score', 'mode'])

        self.current_mode = 'UNKNOWN'

        # Subscribers
        self.score_sub = rospy.Subscriber(topic, Float32, self.score_callback)
        self.status_sub = rospy.Subscriber('/runner/status', String, self.status_callback)

        # Signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo(f'MatchingScoreRecorder: recording to {self.csv_path}')
        rospy.loginfo(f'Subscribed to: {topic} and /runner/status')

    def score_callback(self, msg):
        """Log matching_score with current mode."""
        now = rospy.Time.now().to_sec()
        self.csv_writer.writerow([f'{now:.3f}', f'{msg.data:.4f}', self.current_mode])
        self.csv_file.flush()

    def status_callback(self, msg):
        """Extract mode from /runner/status JSON."""
        import json
        try:
            data = json.loads(msg.data)
            mode = data.get('mode', self.current_mode)
            self.current_mode = mode
        except (json.JSONDecodeError, AttributeError):
            pass

    def signal_handler(self, sig, frame):
        """Clean shutdown."""
        rospy.loginfo(f'Stopping recorder. Data saved to: {self.csv_path}')
        self.csv_file.close()
        sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('matching_score_recorder', anonymous=True)
    recorder = MatchingScoreRecorder()
    rospy.spin()
