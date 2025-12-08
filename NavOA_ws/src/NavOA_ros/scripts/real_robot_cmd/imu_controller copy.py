#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
IMU控制器模块（JY901 修复版）
提供IMU数据读取、解析和里程计融合功能
"""

import serial
import math
import time
import threading
from typing import Optional, Tuple, Callable
from dataclasses import dataclass

try:
    import rospy
    USE_ROS_LOG = True
except ImportError:
    USE_ROS_LOG = False

def _log_info(msg):
    if USE_ROS_LOG:
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")

def _log_warn(msg):
    if USE_ROS_LOG:
        rospy.logwarn(msg)
    else:
        print(f"[WARN] {msg}")

def _log_err(msg):
    if USE_ROS_LOG:
        rospy.logerr(msg)
    else:
        print(f"[ERROR] {msg}")

def _log_debug(msg):
    if USE_ROS_LOG:
        rospy.logdebug(msg)
    else:
        print(f"[DEBUG] {msg}")


@dataclass
class OdometryData:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    timestamp: float = 0.0


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


class IMUController:
    def __init__(self, robot_controller: Optional[object] = None, serial_port: str = "/dev/ttyS4"):
        self.serial_port_path = serial_port
        self.serial_port = None
        self.running = False
        self.read_thread = None 
        self.data_lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.baudrate = 9600
        self.timeout = 0.001
        self.read_command = [0xff, 0xaa, 0x03, 0x0c, 0x0]

        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw_rad = 0.0

        self.odometry_data = OdometryData()
        self.current_data = IMUData()
        self.angle_exceed = 0
        self.angle_effective = 45  # degree

        self.robot_controller = robot_controller
        self.linear_velocity_provider: Optional[Callable[[], float]] = None
        self.last_update_time = None

    def initialize(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_path,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            _log_info(f"IMU初始化成功 - 端口: {self.serial_port_path}, 波特率: {self.baudrate}")
            _log_info(f"IMU调试: 读取命令: {[hex(b) for b in self.read_command]}")
            return True
        except Exception as e:
            _log_err(f"IMU初始化失败: {e}")
            return False

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def create_quaternion_from_yaw(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)

    def update_odometry(self, linear_velocity, angular_velocity, dt):
        vx = linear_velocity
        vy = 0.0
        wz = angular_velocity

        theta = self.yaw_rad

        self.position_x += math.cos(theta) * vx * dt
        self.position_y += math.sin(theta) * vx * dt

        qx, qy, qz, qw = self.create_quaternion_from_yaw(theta)

        self.odometry_data.x = self.position_x
        self.odometry_data.y = self.position_y
        self.odometry_data.qx = qx
        self.odometry_data.qy = qy
        self.odometry_data.qz = qz
        self.odometry_data.qw = qw
        self.odometry_data.vx = vx
        self.odometry_data.vy = vy
        self.odometry_data.wz = wz
        self.odometry_data.timestamp = time.time()
        return self.odometry_data

    def start_reading(self):
        if not self.serial_port:
            _log_err("IMU未初始化，无法开始读取")
            return False
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        _log_info("IMU数据读取线程已启动")
        return True

    def stop_reading(self):
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        _log_info("IMU数据读取已停止")

    def _read_loop(self):
        while self.running:
            try:
                if self._read_imu_data():
                    self.error_count = 0
                else:
                    self.error_count += 1
                    if self.error_count >= self.max_error_count:
                        _log_err(f"IMU连续读取失败 {self.max_error_count} 次，可能存在硬件问题")
                        self.error_count = 0
                time.sleep(0.01)
            except Exception as e:
                _log_err(f"IMU读取循环异常: {e}")
                time.sleep(0.1)

    def _read_imu_data(self) -> bool:
        """稳健地读取并解析一帧IMU数据（累积缓冲 + 帧头搜索）。
        累积数据直到找到完整 0x55 0x52 帧，避免半包问题。
        """
        try:
            # 发送读取命令
            self.serial_port.write(bytes(self.read_command))

            # 累积缓冲区（类级变量，避免每次重置）
            if not hasattr(self, '_buffer'):
                self._buffer = bytearray()

            # 读取新数据到缓冲
            deadline = time.time() + 0.05  # 最多等待 50ms
            while time.time() < deadline:
                avail = getattr(self.serial_port, "in_waiting", None)
                if avail is None:
                    avail = self.serial_port.inWaiting()
                if avail:
                    chunk = self.serial_port.read(avail)
                    if chunk:
                        self._buffer.extend(chunk)
                        # 检查是否能找到完整帧
                        if self._try_parse_frame():
                            return True
                time.sleep(0.0005)

            # 超时未找到帧，清空缓冲避免累积过多
            if len(self._buffer) > 100:
                _log_warn(f"IMU调试: 缓冲区过大 ({len(self._buffer)} 字节)，清空")
                self._buffer.clear()
            return False

        except Exception as e:
            _log_err(f"IMU数据读取异常: {e}")
            return False

    def _try_parse_frame(self) -> bool:
        """尝试从缓冲中解析一帧欧拉角数据"""
        buf = self._buffer
        i = 0
        while i < len(buf) - 1:
            if buf[i] == 0x55 and buf[i + 1] == 0x52:
                # 检查是否有完整 11 字节
                if i + 11 <= len(buf):
                    frame = bytes(buf[i:i + 11])
                    # 移除已解析的数据
                    del self._buffer[:i + 11]
                    # 解析帧
                    imu_data = self._parse_imu_data(frame)
                    if imu_data and imu_data.valid:
                        # 更新里程计
                        current_time = time.time()
                        dt = current_time - self.last_update_time if self.last_update_time is not None else 0.01
                        self.last_update_time = current_time
                        _log_info(f"IMU调试: 里程计更新 - dt={dt:.3f}s, yaw={math.degrees(imu_data.yaw):.2f}°")

                        self.yaw_rad = imu_data.yaw

                        linear_velocity = self._get_linear_velocity()
                        angular_velocity = imu_data.gyro_z
                        _log_info(f"IMU调试: 速度 - linear={linear_velocity:.3f} m/s, angular={angular_velocity:.3f} rad/s")

                        self.update_odometry(linear_velocity, angular_velocity, dt)
                        _log_info(f"IMU调试: 位置更新 - x={self.position_x:.3f}, y={self.position_y:.3f}")

                        with self.data_lock:
                            self.current_data = imu_data

                        self._check_angle_safety(imu_data)
                        return True
                    else:
                        _log_warn(f"IMU调试: 解析欧拉角帧失败，帧数据: {[hex(b) for b in frame]}")
                else:
                    # 帧不完整，等待更多数据
                    break
            i += 1
        return False

    def _parse_imu_data(self, data: bytes) -> Optional[IMUData]:
        """解析 JY901 的 11 字节欧拉角帧 (0x55 0x52)"""
        try:
            if len(data) < 11:
                _log_warn(f"IMU调试: 欧拉角帧长度不足 11 字节: {len(data)}")
                return None

            if data[0] != 0x55 or data[1] != 0x52:
                _log_warn(f"IMU调试: 非欧拉角帧: {hex(data[0])} {hex(data[1])}")
                return None

            # 解析 Roll, Pitch, Yaw (单位: 0.01°)
            roll_raw = (data[3] << 8) | data[2]
            pitch_raw = (data[5] << 8) | data[4]
            yaw_raw = (data[7] << 8) | data[6]

            # 转为有符号 short (-32768 ~ 32767)
            def to_signed_short(val):
                if val >= 32768:
                    return val - 65536
                return val

            roll_deg = to_signed_short(roll_raw) / 100.0
            pitch_deg = to_signed_short(pitch_raw) / 100.0
            yaw_deg = to_signed_short(yaw_raw) / 100.0

            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)

            _log_info(f"IMU调试: 解析成功 - roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°")

            return IMUData(
                accel_x=0.0,
                accel_y=0.0,
                accel_z=9.8,
                gyro_x=0.0,
                gyro_y=0.0,
                gyro_z=0.0,  # 欧拉角帧不包含角速度
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                timestamp=time.time(),
                valid=True
            )

        except Exception as e:
            _log_err(f"IMU欧拉角帧解析失败: {e}")
            return None

    def _check_angle_safety(self, imu_data: IMUData):
        angle_limit = math.radians(self.angle_effective)
        if abs(imu_data.roll) > angle_limit or abs(imu_data.pitch) > angle_limit:
            self.angle_exceed = 1
        else:
            self.angle_exceed = 0

    def get_current_data(self) -> IMUData:
        with self.data_lock:
            data = self.current_data
            return data

    def get_odometry_data(self) -> OdometryData:
        """线程安全地获取当前里程计数据副本"""
        with self.data_lock:
            odom = OdometryData(
                x=self.odometry_data.x,
                y=self.odometry_data.y,
                z=self.odometry_data.z,
                qx=self.odometry_data.qx,
                qy=self.odometry_data.qy,
                qz=self.odometry_data.qz,
                qw=self.odometry_data.qw,
                vx=self.odometry_data.vx,
                vy=self.odometry_data.vy,
                vz=self.odometry_data.vz,
                wx=self.odometry_data.wx,
                wy=self.odometry_data.wy,
                wz=self.odometry_data.wz,
                timestamp=self.odometry_data.timestamp
            )
            _log_info(f"IMU调试: 获取里程计 - x={odom.x:.3f}, y={odom.y:.3f}, yaw={math.degrees(math.atan2(2*(odom.qw*odom.qz + odom.qx*odom.qy), 1-2*(odom.qy**2 + odom.qz**2))):.2f}°")
            return odom

    def get_euler_angles(self) -> Tuple[float, float, float]:
        data = self.get_current_data()
        return data.roll, data.pitch, data.yaw

    def get_angular_velocity(self) -> Tuple[float, float, float]:
        data = self.get_current_data()
        return data.gyro_x, data.gyro_y, data.gyro_z

    def is_data_valid(self) -> bool:
        data = self.get_current_data()
        if not data.valid:
            return False
        if time.time() - data.timestamp > 0.1:
            return False
        return True

    def is_angle_safe(self) -> bool:
        return self.angle_exceed == 0

    def reset_yaw(self):
        _log_info("偏航角重置功能需根据具体IMU型号实现")

    def calibrate(self) -> bool:
        _log_info("开始IMU校准，请保持机器人静止...")
        calibration_samples = []
        start_time = time.time()
        while time.time() - start_time < 3.0:
            data = self.get_current_data()
            if data.valid:
                calibration_samples.append(data)
            time.sleep(0.01)
        if len(calibration_samples) < 100:
            _log_err("校准失败：数据样本不足")
            return False
        avg_roll = sum(s.roll for s in calibration_samples) / len(calibration_samples)
        avg_pitch = sum(s.pitch for s in calibration_samples) / len(calibration_samples)
        _log_info(f"IMU校准完成 - Roll偏移: {math.degrees(avg_roll):.2f}°, Pitch偏移: {math.degrees(avg_pitch):.2f}°")
        return True

    def close(self):
        self.stop_reading()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            _log_info("IMU串口连接已关闭")

    def attach_robot_controller(self, controller: object):
        self.robot_controller = controller

    def set_linear_velocity_provider(self, provider: Callable[[], float]):
        self.linear_velocity_provider = provider


    def _get_linear_velocity(self) -> float:
        """优先从自定义 provider 或 RobotController 获取实时线速度（带调试信息）"""
        try:
            # 方式1: 自定义回调
            if self.linear_velocity_provider is not None:
                value = self.linear_velocity_provider()
                if isinstance(value, (int, float)):
                    return float(value)
                else:
                    _log_warn(f"[速度来源] 自定义 provider 返回非数值: {type(value)}")

            # 方式2: 从 RobotController 获取实际线速度（从电机转速计算）
            if self.robot_controller is not None:
                if hasattr(self.robot_controller, "get_actual_linear_velocity"):
                    actual_lin = self.robot_controller.get_actual_linear_velocity()
                    return float(actual_lin)
                else:
                    _log_warn("[速度来源] RobotController 无 'get_actual_linear_velocity' 属性")
            else:
                _log_debug("[速度来源] robot_controller 未绑定")

        except Exception as exc:
            _log_warn(f"[速度来源] 读取线速度失败，使用默认值0: {exc}")

        return 0.0