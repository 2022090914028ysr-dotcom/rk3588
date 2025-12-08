#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
JY901 IMU 驱动（真实数据版）- 语法已修复
"""

import serial
import math
import time
import threading
from typing import Optional
from dataclasses import dataclass

try:
    import rospy
    from sensor_msgs.msg import Imu
    USE_ROS = True
except ImportError:
    USE_ROS = False

def _log_info(msg):
    if USE_ROS:
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")

def _log_warn(msg):
    if USE_ROS:
        rospy.logwarn(msg)
    else:
        print(f"[WARN] {msg}")

def _log_err(msg):
    if USE_ROS:
        rospy.logerr(msg)
    else:
        print(f"[ERROR] {msg}")


@dataclass
class IMUData:
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    timestamp: float = 0.0
    valid: bool = False


class JY901IMUDriver:
    def __init__(self, serial_port: str = "/dev/ttyS4", baudrate: int = 9600):
        self.serial_port_path = serial_port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.timeout = 0.001
        self.read_command = [0xff, 0xaa, 0x27, 0x00, 0x00]  # 读取命令，触发IMU输出数据

        self.current_data = IMUData()
        self.angle_exceed = 0
        self.angle_effective = 45  # degrees

        # 陀螺仪偏置
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.use_ros = USE_ROS
        if self.use_ros:
            self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
            _log_info("ROS IMU 发布器初始化完成")

    def initialize(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_path,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            _log_info(f"串口初始化成功: {self.serial_port_path} @ {self.baudrate}")
            time.sleep(0.1)
            self._configure_jy901()
            return True
        except Exception as e:
            _log_err(f"初始化失败: {e}")
            return False

    def _configure_jy901(self):
        # JY901 配置命令：启用加速度、角速度、四元数输出 @100Hz
        configs = [
            [0xFF, 0xAA, 0x03, 0x00, 0x00],  # 恢复默认
            [0xFF, 0xAA, 0x01, 0x01, 0x00],  # 启用加速度输出
            [0xFF, 0xAA, 0x01, 0x02, 0x00],  # 启用角速度输出
            [0xFF, 0xAA, 0x01, 0x04, 0x00],  # 启用四元数输出
            [0xFF, 0xAA, 0x02, 0x08, 0x00],  # 设置输出频率 100Hz
        ]
        for cmd in configs:
            self.serial_port.write(bytes(cmd))
            time.sleep(0.05)
        _log_info("JY901 配置完成：启用 0x50/0x51/0x53 帧 @ 100Hz")

    def start_reading(self):
        if not self.serial_port:
            _log_err("串口未初始化")
            return False
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        _log_info("IMU 读取线程启动")
        return True

    def stop_reading(self):
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        _log_info("IMU 读取停止")

    def _read_loop(self):
        while self.running:
            try:
                if self._read_imu_data():
                    self.error_count = 0
                else:
                    self.error_count += 1
                    if self.error_count >= self.max_error_count:
                        _log_err(f"连续 {self.max_error_count} 次读取失败")
                        self.error_count = 0
                time.sleep(0.01)
            except Exception as e:
                _log_err(f"读取循环异常: {e}")
                time.sleep(0.1)

    def _read_imu_data(self) -> bool:
        try:
            # 发送读取命令触发IMU输出
            self.serial_port.write(bytes(self.read_command))

            if not hasattr(self, '_buffer'):
                self._buffer = bytearray()

            avail = getattr(self.serial_port, 'in_waiting', self.serial_port.inWaiting())
            if avail:
                chunk = self.serial_port.read(avail)
                self._buffer.extend(chunk)

            parsed = False
            while len(self._buffer) >= 11:
                idx = self._buffer.find(0x55)
                if idx == -1:
                    self._buffer.clear()
                    break
                if idx > 0:
                    del self._buffer[:idx]
                if len(self._buffer) < 11:
                    break

                frame_type = self._buffer[1]
                if frame_type not in (0x50, 0x51, 0x53):
                    del self._buffer[0]
                    continue

                frame = bytes(self._buffer[:11])
                del self._buffer[:11]

                _log_info(f"收到帧: type=0x{frame_type:02x}, data={[hex(b) for b in frame]}")

                imu_data = self._parse_frame(frame)
                if imu_data and imu_data.valid:
                    self._update_data(imu_data, frame_type)
                    parsed = True
                    _log_info(f"成功解析帧: type=0x{frame_type:02x}")
                else:
                    _log_warn(f"解析帧失败: type=0x{frame_type:02x}, 数据: {[hex(b) for b in frame]}")
            return parsed

        except Exception as e:
            _log_err(f"读取异常: {e}")
            return False

    def _parse_frame(self, data: bytes) -> Optional[IMUData]:
        try:
            frame_type = data[1]
            val1_raw = (data[3] << 8) | data[2]
            val2_raw = (data[5] << 8) | data[4]
            val3_raw = (data[7] << 8) | data[6]

            def to_signed_short(val):
                return val - 65536 if val >= 32768 else val

            if frame_type == 0x50:
                ax = to_signed_short(val1_raw) * 9.80665 / 1000.0
                ay = to_signed_short(val2_raw) * 9.80665 / 1000.0
                az = to_signed_short(val3_raw) * 9.80665 / 1000.0
                return IMUData(accel_x=ax, accel_y=ay, accel_z=az, valid=True)

            elif frame_type == 0x51:
                gx = math.radians(to_signed_short(val1_raw) / 100.0) - self.gyro_bias_x
                gy = math.radians(to_signed_short(val2_raw) / 100.0) - self.gyro_bias_y
                gz = math.radians(to_signed_short(val3_raw) / 100.0) - self.gyro_bias_z
                return IMUData(gyro_x=gx, gyro_y=gy, gyro_z=gz, valid=True)

            elif frame_type == 0x53:
                q0 = to_signed_short(val1_raw) * 3.3e-5
                q1 = to_signed_short(val2_raw) * 3.3e-5
                q2 = to_signed_short(val3_raw) * 3.3e-5
                q3_raw = (data[9] << 8) | data[8]
                q3 = to_signed_short(q3_raw) * 3.3e-5
                norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
                if norm > 0:
                    q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm
                roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
                pitch = math.asin(2*(q0*q2 - q3*q1))
                yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
                return IMUData(roll=roll, pitch=pitch, yaw=yaw, valid=True)

        except Exception as e:
            _log_err(f"帧解析错误: {e}")
            return None

    def _update_data(self, new_data: IMUData, frame_type: int):
        with self.data_lock:
            if frame_type == 0x50:
                self.current_data.accel_x = new_data.accel_x
                self.current_data.accel_y = new_data.accel_y
                self.current_data.accel_z = new_data.accel_z
            elif frame_type == 0x51:
                self.current_data.gyro_x = new_data.gyro_x
                self.current_data.gyro_y = new_data.gyro_y
                self.current_data.gyro_z = new_data.gyro_z
            elif frame_type == 0x53:
                self.current_data.roll = new_data.roll
                self.current_data.pitch = new_data.pitch
                self.current_data.yaw = new_data.yaw
            self.current_data.timestamp = time.time()
            self.current_data.valid = True

        self._check_angle_safety(self.current_data)
        if self.use_ros:
            self._publish_imu_message()

    # ✅ 修复后的函数：参数名 + 类型注解正确
    def _check_angle_safety(self, imu_data: IMUData):
        angle_limit = math.radians(self.angle_effective)
        if abs(imu_data.roll) > angle_limit or abs(imu_data.pitch) > angle_limit:
            self.angle_exceed = 1
        else:
            self.angle_exceed = 0

    def get_current_data(self) -> IMUData:
        with self.data_lock:
            return self.current_data

    def is_data_valid(self) -> bool:
        data = self.get_current_data()
        return data.valid and (time.time() - data.timestamp <= 0.1)

    def calibrate_gyro(self, duration: float = 3.0):
        """校准陀螺仪偏置，机器人静止时调用"""
        _log_info(f"开始陀螺仪校准，保持静止 {duration} 秒...")
        samples = []
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.is_data_valid():
                d = self.get_current_data()
                samples.append((d.gyro_x, d.gyro_y, d.gyro_z))
            time.sleep(0.01)
        if len(samples) < 100:
            _log_err("校准失败：样本不足")
            return False
        self.gyro_bias_x = sum(s[0] for s in samples) / len(samples)
        self.gyro_bias_y = sum(s[1] for s in samples) / len(samples)
        self.gyro_bias_z = sum(s[2] for s in samples) / len(samples)
        _log_info(f"陀螺仪校准完成 - 偏置: x={self.gyro_bias_x:.3f}, y={self.gyro_bias_y:.3f}, z={self.gyro_bias_z:.3f} rad/s")
        return True

    def _publish_imu_message(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = self.current_data.accel_x
        imu_msg.linear_acceleration.y = self.current_data.accel_y
        imu_msg.linear_acceleration.z = self.current_data.accel_z if abs(self.current_data.accel_z) > 1.0 else 9.8  # 默认重力加速度

        imu_msg.angular_velocity.x = self.current_data.gyro_x
        imu_msg.angular_velocity.y = self.current_data.gyro_y
        imu_msg.angular_velocity.z = self.current_data.gyro_z

        cy = math.cos(self.current_data.yaw * 0.5)
        sy = math.sin(self.current_data.yaw * 0.5)
        cp = math.cos(self.current_data.pitch * 0.5)
        sp = math.sin(self.current_data.pitch * 0.5)
        cr = math.cos(self.current_data.roll * 0.5)
        sr = math.sin(self.current_data.roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        imu_msg.angular_velocity_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
        imu_msg.linear_acceleration_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]
        imu_msg.orientation_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]

        try:
            self.imu_pub.publish(imu_msg)
        except Exception as e:
            _log_warn(f"发布 IMU 失败: {e}")

    def close(self):
        self.stop_reading()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            _log_info("串口已关闭")


if __name__ == '__main__':
    if USE_ROS:
        rospy.init_node('jy901_imu_driver', anonymous=True)
        port = rospy.get_param('~port', '/dev/ttyS4')
        baud = rospy.get_param('~baud', 9600)
        imu = JY901IMUDriver(serial_port=port, baudrate=baud)
        if imu.initialize():
            imu.start_reading()
            # 校准陀螺仪
            imu.calibrate_gyro(3.0)
            rospy.loginfo("JY901 IMU 驱动运行中...")
            rospy.spin()
        imu.close()
    else:
        print("非 ROS 模式：打印真实 IMU 数据")
        imu = JY901IMUDriver(serial_port="/dev/ttyS4")
        if imu.initialize():
            imu.start_reading()
            try:
                while True:
                    time.sleep(0.5)
                    if imu.is_data_valid():
                        d = imu.get_current_data()
                        print(
                            f"ACC(m/s²): [{d.accel_x:6.2f}, {d.accel_y:6.2f}, {d.accel_z:6.2f}] | "
                            f"GYRO(rad/s): [{d.gyro_x:6.3f}, {d.gyro_y:6.3f}, {d.gyro_z:6.3f}] | "
                            f"RPY(deg): [{math.degrees(d.roll):6.1f}, {math.degrees(d.pitch):6.1f}, {math.degrees(d.yaw):6.1f}]"
                        )
            except KeyboardInterrupt:
                pass
            imu.close()