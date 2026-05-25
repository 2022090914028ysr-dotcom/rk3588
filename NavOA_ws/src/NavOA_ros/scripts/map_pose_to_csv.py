#!/usr/bin/env python3

import csv
import os
import sys
from datetime import datetime


def load_ros_modules():
    try:
        import rospy  # type: ignore
        import tf  # type: ignore

        return rospy, tf
    except ModuleNotFoundError as exc:
        system_python = "/usr/bin/python3"
        if sys.executable != system_python and os.path.exists(system_python):
            print(
                "Current Python cannot import ROS dependencies; retrying with /usr/bin/python3",
                file=sys.stderr,
            )
            os.execv(system_python, [system_python, os.path.abspath(__file__), *sys.argv[1:]])
        raise RuntimeError(
            "Unable to import ROS Python dependencies. Install the missing packages or run this script with /usr/bin/python3."
        ) from exc


rospy, tf = load_ros_modules()


def default_output_path():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    log_dir = os.path.join(package_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"map_pose_{timestamp}.csv")


class MapPoseToCsvRecorder:
    def __init__(self):
        rospy.init_node("map_pose_to_csv", anonymous=False)

        self.source_frame = rospy.get_param("~source_frame", "map")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")
        self.output_csv = rospy.get_param("~output_csv", default_output_path())
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.max_samples = int(rospy.get_param("~max_samples", 0))

        output_dir = os.path.dirname(self.output_csv)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        self.file_handle = open(self.output_csv, "w", newline="")
        self.writer = csv.writer(self.file_handle)
        self.writer.writerow(
            [
                "stamp_sec",
                "stamp_nsec",
                "source_frame",
                "target_frame",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "roll",
                "pitch",
                "yaw",
            ]
        )
        self.file_handle.flush()

        self.listener = tf.TransformListener()
        self.sample_count = 0

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Recording TF pose %s -> %s to %s at %.2f Hz",
            self.source_frame,
            self.target_frame,
            self.output_csv,
            self.rate_hz,
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            try:
                stamp = self.listener.getLatestCommonTime(self.source_frame, self.target_frame)
                translation, rotation = self.listener.lookupTransform(
                    self.source_frame, self.target_frame, stamp
                )
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(rotation)

                self.writer.writerow(
                    [
                        stamp.secs,
                        stamp.nsecs,
                        self.source_frame,
                        self.target_frame,
                        translation[0],
                        translation[1],
                        translation[2],
                        rotation[0],
                        rotation[1],
                        rotation[2],
                        rotation[3],
                        roll,
                        pitch,
                        yaw,
                    ]
                )
                self.sample_count += 1

                if self.sample_count % 20 == 0:
                    self.file_handle.flush()
                    rospy.loginfo("Recorded %d pose samples", self.sample_count)

                if self.max_samples > 0 and self.sample_count >= self.max_samples:
                    rospy.loginfo("Reached max_samples=%d, stopping pose recorder", self.max_samples)
                    rospy.signal_shutdown("Completed requested pose capture")
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting for TF %s -> %s",
                    self.source_frame,
                    self.target_frame,
                )

            rate.sleep()

    def shutdown(self):
        if hasattr(self, "file_handle") and self.file_handle and not self.file_handle.closed:
            self.file_handle.flush()
            self.file_handle.close()
            rospy.loginfo("Saved %d pose samples to %s", self.sample_count, self.output_csv)


if __name__ == "__main__":
    recorder = MapPoseToCsvRecorder()
    recorder.run()