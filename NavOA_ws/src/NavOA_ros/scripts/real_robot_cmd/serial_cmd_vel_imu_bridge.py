#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bridge /cmd_vel and IMU data over a single serial port.

From this PC:
  - Read IMU CSV lines coming from the other PC via serial (default /dev/ttyS6)
    and publish them as sensor_msgs/Imu on /imu (or a configurable topic).
  - Subscribe to /cmd_vel and forward each command as a CSV line over the
    same serial port so the other PC can consume it.

Expected IMU line format from the other PC:
  IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw\n

Optional extended IMU line format:
    IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw,stamp_sec\n
Expected cmd_vel line format sent to the other PC:
    CMD_VEL,linear_x,angular_z\n
"""

import threading
import time
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

try:
    import serial  # pyserial
except ImportError:
    serial = None


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert ZYX Euler angles (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class SerialCmdVelImuBridge(object):
    def __init__(self):
        rospy.init_node("serial_cmd_vel_imu_bridge", anonymous=False)

        if serial is None:
            raise RuntimeError("pyserial is required for serial_cmd_vel_imu_bridge")

        # Parameters
        self.port = rospy.get_param("~serial_port", "/dev/ttyS7")
        self.baudrate = rospy.get_param("~baudrate", 230400)
        self.serial_timeout = rospy.get_param("~serial_timeout", 0.1)

        # Topics
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.button_topic = rospy.get_param("~button_topic", "/app_controller/button")
        # This is the IMU topic used by the rest of the stack; by default /imu
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        self.imu_time_offset = rospy.get_param("~imu_time_offset", 0.12)
        self.imu_min_dt = rospy.get_param("~imu_min_dt", 0.001)
        self.imu_max_future = rospy.get_param("~imu_max_future", 0.2)
        self.prefer_device_stamp = rospy.get_param("~prefer_device_stamp", True)
        self.last_imu_stamp = None

        # Serial
        self.serial_lock = threading.Lock()
        self.serial_port = None
        self.running = True

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
            )
            rospy.loginfo("Opened serial port %s @ %d", self.port, self.baudrate)
        except Exception as e:
            rospy.logerr("Failed to open serial port %s: %s", self.port, e)
            raise

        # ROS pub/sub
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=50)
        self.cmd_sub = rospy.Subscriber(
            self.cmd_vel_topic, Twist, self._cmd_vel_callback, queue_size=50
        )
        self.btn_sub = rospy.Subscriber(
            self.button_topic, Int32MultiArray, self._button_callback, queue_size=50
        )

        # Reader thread for IMU lines coming from the other PC
        self.reader_thread = threading.Thread(
            target=self._serial_reader_loop, daemon=True
        )
        self.reader_thread.start()

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "serial_cmd_vel_imu_bridge running: IMU/cmd_vel/button over %s (offset=%.3fs, min_dt=%.4fs)",
            self.port,
            self.imu_time_offset,
            self.imu_min_dt,
        )

    # ------------------------------------------------------------------
    # /cmd_vel -> serial
    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist):
        """Forward cmd_vel as simple CSV: vx,vy,w.

        Many existing firmwares expect three comma-separated floats, where
        vx=linear.x [m/s], vy=linear.y [m/s] (unused here, 0.0),
        w=angular.z [rad/s]. Example:
          0.200000,0.000000,0.000000\n
        """
        vx = msg.linear.x
        vy = 0.0
        w = msg.angular.z
        line = "CMD_VEL,{:.6f},{:.6f},{:.6f}\n".format(vx, vy, w)
        rospy.loginfo_throttle(
            0.5,
            "[serial_cmd_vel_imu_bridge] Forwarding cmd_vel: vx=%.3f vy=%.3f w=%.3f as '%s'",
            vx,
            vy,
            w,
            line.strip(),
        )
        data = line.encode("ascii", errors="ignore")

        with self.serial_lock:
            if not self.serial_port or not self.serial_port.is_open:
                rospy.logwarn_throttle(5.0, "Serial not open, cannot send cmd_vel")
                return
            try:
                self.serial_port.write(data)
                self.serial_port.flush()
            except Exception as e:
                rospy.logerr_throttle(5.0, "Error writing cmd_vel to serial: %s", e)

    def _button_callback(self, msg: "Int32MultiArray"):
        try:
            data = list(msg.data) if msg.data is not None else []
            if len(data) < 2:
                return
            button_id = int(data[0])
            event_type = int(data[1])
            line = f"BUTTON,{button_id},{event_type}\n"
            payload = line.encode("ascii", errors="ignore")
            with self.serial_lock:
                if not self.serial_port or not self.serial_port.is_open:
                    rospy.logwarn_throttle(5.0, "Serial not open, cannot send button")
                    return
                self.serial_port.write(payload)
                self.serial_port.flush()
            rospy.loginfo_throttle(
                0.5,
                "[serial_cmd_vel_imu_bridge] Forwarding button: id=%d event=%d as '%s'",
                button_id,
                event_type,
                line.strip(),
            )
        except Exception as e:
            rospy.logerr_throttle(5.0, "Error writing button to serial: %s", e)

    # ------------------------------------------------------------------
    # serial IMU -> /imu
    # ------------------------------------------------------------------
    def _serial_reader_loop(self):
        """Background loop reading IMU CSV lines from serial.

        Expected format from the other PC:
          IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw\n
        """
        while not rospy.is_shutdown() and self.running:
            try:
                with self.serial_lock:
                    if not self.serial_port or not self.serial_port.is_open:
                        time.sleep(0.1)
                        continue
                    raw = self.serial_port.readline()

                if not raw:
                    continue

                try:
                    line = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    continue

                if not line:
                    continue

                if line.startswith("IMU,"):
                    self._handle_imu_line(line)

            except Exception as e:
                rospy.logwarn_throttle(2.0, "Error in serial reader loop: %s", e)
                time.sleep(0.1)

    def _handle_imu_line(self, line: str):
        parts = line.split(",")
        if len(parts) not in (10, 11):
            rospy.logwarn_throttle(5.0, "Invalid IMU line: %s", line)
            return

        try:
            _, ax, ay, az, gx, gy, gz, roll, pitch, yaw = parts[:10]
            ax_f = float(ax)
            ay_f = float(ay)
            az_f = float(az)
            gx_f = float(gx)
            gy_f = float(gy)
            gz_f = float(gz)
            roll_f = float(roll)
            pitch_f = float(pitch)
            yaw_f = float(yaw)
            device_stamp = float(parts[10]) if len(parts) == 11 else None
        except ValueError:
            rospy.logwarn_throttle(5.0, "Failed to parse IMU line: %s", line)
            return

        msg = Imu()
        msg.header.stamp = self._compute_imu_stamp(device_stamp)
        msg.header.frame_id = self.imu_frame_id

        msg.linear_acceleration.x = ax_f
        msg.linear_acceleration.y = ay_f
        msg.linear_acceleration.z = az_f

        msg.angular_velocity.x = gx_f
        msg.angular_velocity.y = gy_f
        msg.angular_velocity.z = gz_f

        qx, qy, qz, qw = euler_to_quaternion(roll_f, pitch_f, yaw_f)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Simple default covariances; can be tuned downstream
        msg.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
        msg.linear_acceleration_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
        msg.orientation_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]

        self.imu_pub.publish(msg)

    def _compute_imu_stamp(self, device_stamp: float):
        if self.prefer_device_stamp and device_stamp is not None:
            stamp = rospy.Time.from_sec(device_stamp)
        else:
            stamp = rospy.Time.now()

        stamp += rospy.Duration.from_sec(self.imu_time_offset)

        max_future_stamp = rospy.Time.now() + rospy.Duration.from_sec(self.imu_max_future)
        if stamp > max_future_stamp:
            stamp = max_future_stamp

        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            stamp = self.last_imu_stamp + rospy.Duration.from_sec(self.imu_min_dt)

        self.last_imu_stamp = stamp
        return stamp

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        self.running = False
        rospy.loginfo("Shutting down serial_cmd_vel_imu_bridge...")

        try:
            if self.reader_thread and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass

        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                    rospy.loginfo("Closed serial port %s", self.port)
                except Exception as e:
                    rospy.logwarn("Error closing serial port %s: %s", self.port, e)


def main():
    try:
        bridge = SerialCmdVelImuBridge()
    except Exception as e:
        # Make initialization errors visible when running with rosrun/roslaunch
        import traceback
        rospy.logerr("serial_cmd_vel_imu_bridge failed to start: %s", e)
        traceback.print_exc()
        return

    rospy.loginfo("serial_cmd_vel_imu_bridge is running")
    rospy.spin()


if __name__ == "__main__":
    main()