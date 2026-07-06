#!/usr/bin/env python
"""
Multi DYP-A21 Ultrasonic Sensors -> ROS /scan (LaserScan)

Default sensor layout:
  /dev/ttyS3 -- front    0 deg
  /dev/ttyS7 -- left   -75 deg
  /dev/ttyS6 -- right  +75 deg

Each sensor's single-point reading is expanded to a beam width (default 30 deg),
producing a sensor_msgs/LaserScan with frame_id = "base_footprint".

Protocol (same as a21_ultrasonic_mm.py):
  FF Data_H Data_L SUM
  SUM = (FF + Data_H + Data_L) & 0xFF
  0xFFFE -> same-frequency interference
  0xFFFD -> no object detected

ROS:
  Topic : /scan
  Type  : sensor_msgs/LaserScan
"""

import argparse
import math
import threading
import serial
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

HEADER = 0xFF
INVALID_VALUES = {0xFFFE, 0xFFFD}


# ---------------------------------------------------------------------------
class A21Sensor(threading.Thread):
    """Single DYP-A21 sensor reader running in a background thread."""

    def __init__(self, name, port, baudrate=115200, timeout=0.05):
        threading.Thread.__init__(self)
        self.daemon = True

        self.name = name
        self.port = port
        self._latest_mm = None
        self._lock = threading.Lock()
        self._running = False
        self.ser = None

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
            self._running = True
        except Exception as e:
            rospy.logerr("[%s] Failed to open %s: %s", self.name, port, e)

    # ---- thread loop -------------------------------------------------------
    def run(self):
        while self._running and not rospy.is_shutdown():
            try:
                val = self._read_one_frame()
                if val is not None:
                    with self._lock:
                        self._latest_mm = val
            except Exception:
                pass  # transient serial glitch, retry next iteration

    # ---- protocol parser ---------------------------------------------------
    def _read_one_frame(self):
        """Return distance in mm (int), or None on failure."""
        first = self.ser.read(1)
        if not first:
            return None
        if first[0] != HEADER:
            return None

        rest = self.ser.read(3)
        if len(rest) != 3:
            return None

        data_h, data_l, recv_sum = rest
        calc_sum = (HEADER + data_h + data_l) & 0xFF
        if calc_sum != recv_sum:
            return None

        value = (data_h << 8) | data_l
        if value in INVALID_VALUES:
            return None

        return value

    # ---- public API --------------------------------------------------------
    def get_mm(self):
        """Return latest distance in mm, or None if no valid data yet."""
        with self._lock:
            return self._latest_mm

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


# ---------------------------------------------------------------------------
class MultiUltrasonicScan:
    """Manages multiple ultrasonic sensors and publishes /scan."""

    def __init__(self,
                 ports,
                 angles_deg,
                 beam_width_deg=20.0,
                 scan_fov_deg=180.0,
                 angular_res_deg=1.0,
                 range_min_m=0.02,
                 range_max_m=3.0,
                 baudrate=115200):

        # ---- start sensor threads ----
        port_names = ["front", "left", "right"]
        self.sensors = []
        self.angles_rad = [math.radians(a) for a in angles_deg]

        for name, port in zip(port_names, ports):
            s = A21Sensor(name, port, baudrate=baudrate)
            self.sensors.append(s)
            s.start()

        self.beam_half_rad = math.radians(beam_width_deg / 2.0)

        # ---- LaserScan geometry ----
        self.angle_min = math.radians(-scan_fov_deg / 2.0)
        self.angle_max = math.radians(+scan_fov_deg / 2.0)
        self.angle_increment = math.radians(angular_res_deg)
        self.range_min = range_min_m
        self.range_max = range_max_m

        self.num_beams = int(round(
            (self.angle_max - self.angle_min) / self.angle_increment
        )) + 1

        # ---- ROS ----
        rospy.init_node("multi_ultrasonic_scan", anonymous=False)
        self.pub = rospy.Publisher("/voscan", LaserScan, queue_size=10)

        # per-sensor raw distance topics (meters, Float32) for virtual-bumper use
        port_names = ["front", "left", "right"]
        self._raw_pubs = {}
        for name in port_names:
            self._raw_pubs[name] = rospy.Publisher(
                "/ultrasonic/%s" % name, Float32, queue_size=5
            )

    # -----------------------------------------------------------------------
    @staticmethod
    def _angle_diff(a, b):
        """Return |a-b|, handling wrap-around at +/- pi."""
        d = abs(a - b)
        if d > math.pi:
            d = 2.0 * math.pi - d
        return d

    def _in_beam(self, beam_angle, sensor_angle):
        return self._angle_diff(beam_angle, sensor_angle) <= self.beam_half_rad

    # -----------------------------------------------------------------------
    def _build_scan_msg(self):
        """Build a LaserScan message from current sensor caches."""
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = []
        intensities = []

        for i in range(self.num_beams):
            beam_angle = self.angle_min + i * self.angle_increment

            best_m = None  # meters, pick closest obstacle

            for sensor, s_angle in zip(self.sensors, self.angles_rad):
                if self._in_beam(beam_angle, s_angle):
                    d_mm = sensor.get_mm()
                    if d_mm is not None:
                        d_m = d_mm / 1000.0
                        if best_m is None or d_m < best_m:
                            best_m = d_m

            if best_m is None:
                ranges.append(float("inf"))
            else:
                if best_m < self.range_min:
                    ranges.append(float("inf"))
                elif best_m > self.range_max:
                    ranges.append(self.range_max)
                else:
                    ranges.append(best_m)

            intensities.append(0.0)

        msg.ranges = ranges
        msg.intensities = intensities
        return msg

    # -----------------------------------------------------------------------
    def loop(self, rate_hz=20):
        rate = rospy.Rate(rate_hz)
        port_names = ["front", "left", "right"]

        while not rospy.is_shutdown():
            msg = self._build_scan_msg()
            self.pub.publish(msg)

            # publish per-sensor raw distances for virtual-bumper (meters)
            for name, sensor in zip(port_names, self.sensors):
                d_mm = sensor.get_mm()
                if d_mm is not None:
                    self._raw_pubs[name].publish(Float32(data=d_mm / 1000.0))

            rate.sleep()

        for s in self.sensors:
            s.stop()


# ===========================================================================
def main():
    # filter out ROS remapping args (__name:=, __log:=, etc.) before argparse
    import sys
    ros_args = [a for a in sys.argv[1:] if not a.startswith('__') and ':=' not in a]
    sys.argv = [sys.argv[0]] + ros_args

    parser = argparse.ArgumentParser(
        description="Multi DYP-A21 -> /scan LaserScan (frame: base_footprint)"
    )

    # ---- serial ports ----
    parser.add_argument("--port-front", default="/dev/ttyS3",
                        help="Front sensor serial port")
    parser.add_argument("--port-left",  default="/dev/ttyS6",
                        help="Left sensor serial port")
    parser.add_argument("--port-right", default="/dev/ttyS7",
                        help="Right sensor serial port")

    # ---- angles ----
    parser.add_argument("--angle-front", type=float, default=0.0,
                        help="Front sensor angle (deg)")
    parser.add_argument("--angle-left",  type=float, default=-80.0,
                        help="Left sensor angle (deg)")
    parser.add_argument("--angle-right", type=float, default=80.0,
                        help="Right sensor angle (deg)")

    # ---- beam & scan ----
    parser.add_argument("--beam-width", type=float, default=25.0,
                        help="Beam width per sensor (deg)")
    parser.add_argument("--scan-fov", type=float, default=180.0,
                        help="Total scan FOV (deg)")
    parser.add_argument("--angular-res", type=float, default=1.0,
                        help="Angular resolution (deg)")

    # ---- misc ----
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=20,
                        help="Publish rate (Hz)")
    parser.add_argument("--range-min", type=float, default=0.02,
                        help="Min range (m)")
    parser.add_argument("--range-max", type=float, default=6.0,
                        help="Max range (m)")

    args = parser.parse_args()

    scanner = MultiUltrasonicScan(
        ports=[args.port_front, args.port_left, args.port_right],
        angles_deg=[args.angle_front, args.angle_left, args.angle_right],
        beam_width_deg=args.beam_width,
        scan_fov_deg=args.scan_fov,
        angular_res_deg=args.angular_res,
        range_min_m=args.range_min,
        range_max_m=args.range_max,
        baudrate=args.baud,
    )
    scanner.loop(rate_hz=args.rate)


if __name__ == "__main__":
    main()
