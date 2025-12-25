#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
IMU控制器模块
提供IMU数据读取和处理功能 (已移除对 robot_config.py 的依赖)
"""

import serial
import math
import time
import threading
import platform
from typing import List, Optional, Tuple
from dataclasses import dataclass

# 移除对 robot_config 的导入
# from .robot_config import config 


@dataclass
class IMUData:
    """IMU数据结构"""
    # 加速度 (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    # 角速度 (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # 欧拉角 (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # 时间戳
    timestamp: float = 0.0
    
    # 数据有效性
    valid: bool = False


class IMUController:
    """IMU控制器类"""
    
    def __init__(self):
        # --- 替换 robot_config 中的 IMU/安全配置 ---
        # 1. IMU 串口配置 (硬编码)
        if platform.system() == 'Windows':
            self.imu_port = "COM13"         # Windows系统默认串口
        else:
            self.imu_port = "/dev/ttyS4"  # Linux系统默认串口
        self.imu_baudrate = 230400
        self.read_timeout = 0.01  # 10ms 超时

        # 2. 安全配置 (硬编码)
        self.angle_effective_deg = 45.0  # 安全角度限制 (度)
        self.angle_effective_rad = math.radians(self.angle_effective_deg)
        self.imu_angle_exceed = 0  # 0:安全, 1:超出限制
        self.center_pitch = math.radians(4.0) # 俯仰角中心点偏移 (原在 RobotConfig 中)
        # ---------------------------------------------
        
        self.serial_port = None
        self.running = False
        self.read_thread = None
        self.current_data = IMUData()
        self.data_lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.simulation_mode = False  
        self.roll_bias = 0.0
        self.pitch_bias = 0.0

        self.is_calibrated_flag = False
        self.last_read_time = 0
        self.read_count = 0
        self.frequency = 0.0
        self.last_frequency_update = 0
        self.data_buffer = bytearray()
        
        import atexit
        atexit.register(self.close)
        
    def get_frequency(self) -> float:
        """获取实际的IMU读取频率"""
        with self.data_lock:
            return self.frequency
            
    def initialize(self) -> bool:
        """初始化IMU串口连接"""
        try:
            self.serial_port = serial.Serial(
                port=self.imu_port,
                baudrate=self.imu_baudrate,
                timeout=self.read_timeout
            )
            self.serial_port.reset_input_buffer()
            print(f"IMU初始化成功 - 端口: {self.imu_port}, 波特率: {self.imu_baudrate}, 超时: {self.read_timeout*1000:.0f}ms")
            return True
        except Exception as e:
            print(f"IMU初始化失败: {e}")
            print("尝试使用IMU模拟模式 / Trying IMU simulation mode")
            self.simulation_mode = True
            self.current_data = IMUData(
                roll=0.0, pitch=0.0, yaw=0.0,
                gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
                timestamp=time.time(), valid=True
            )
            print("IMU模拟模式初始化成功 / IMU simulation mode initialized successfully")
            return True

    def start_reading(self):
        """开始读取IMU数据"""
        if not self.simulation_mode and not self.serial_port:
            print("IMU未初始化，无法开始读取")
            return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        if self.simulation_mode:
            print("IMU模拟数据读取线程已启动 / IMU simulation data reading thread started")
        else:
            print("IMU数据读取线程已启动")
        return True
    
    def stop_reading(self):
        """停止读取IMU数据"""
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        print("IMU数据读取已停止")
    
    def _read_loop(self):
        """IMU数据读取循环"""
        while self.running:
            try:
                current_time = time.time()
                
                if self._read_imu_data():
                    # Update frequency tracking
                    with self.data_lock:
                        if self.last_read_time > 0:
                            self.read_count += 1
                            if self.read_count % 10 == 0:
                                elapsed = current_time - self.last_frequency_update
                                if elapsed > 0:
                                    self.frequency = 10 / elapsed
                                    self.last_frequency_update = current_time
                        else:
                            self.last_frequency_update = current_time
                            
                        self.last_read_time = current_time
                    
                    self.error_count = 0
                else:
                    self.error_count += 1
                    if self.error_count >= self.max_error_count:
                        print(f"IMU连续读取失败 {self.max_error_count} 次，可能存在硬件问题")
                        self.error_count = 0 
                
                if self.simulation_mode:
                    time.sleep(0.002) 
                
            except Exception as e:
                print(f"IMU读取循环异常: {e}")
                time.sleep(0.1)
    
    def _read_imu_data(self) -> bool:
        """读取一次IMU数据"""
        try:
            if self.simulation_mode:
                current_time = time.time()
                roll_offset = math.sin(current_time * 0.5) * 0.02
                pitch_offset = math.cos(current_time * 0.3) * 0.01
                yaw_offset = math.sin(current_time * 0.1) * 0.005
                
                imu_data = IMUData(
                    roll=roll_offset,
                    pitch=pitch_offset,
                    yaw=yaw_offset,
                    gyro_x=roll_offset * 0.1,
                    gyro_y=pitch_offset * 0.1,
                    gyro_z=yaw_offset * 0.1,
                    timestamp=current_time,
                    valid=True
                )
                
                with self.data_lock:
                    self.current_data = imu_data
                
                # 检查角度是否超出安全范围 (不再更新全局 config)
                self._check_angle_safety(imu_data)
                return True
            
            # 真实硬件模式：流式解析
            if not self.serial_port or not self.serial_port.is_open:
                return False

            try:
                bytes_waiting = self.serial_port.in_waiting
                if bytes_waiting > 0:
                    read_size = min(bytes_waiting, 256)
                    new_data = self.serial_port.read(read_size)
                    if new_data:
                        self.data_buffer.extend(new_data)
                else:
                    new_data = self.serial_port.read(11)
                    if new_data:
                        self.data_buffer.extend(new_data)
            except Exception:
                pass

            if len(self.data_buffer) > 1024:
                self.data_buffer = self.data_buffer[-512:]

            accel_data = None
            gyro_data = None
            angle_data = None
            parsed_any = False

            while len(self.data_buffer) >= 11:
                if self.data_buffer[0] != 0x55:
                    self.data_buffer.pop(0)
                    continue

                frame_type = self.data_buffer[1]
                if frame_type not in (0x51, 0x52, 0x53, 0x54):
                    self.data_buffer.pop(0)
                    continue

                packet = bytes(self.data_buffer[:11])
                checksum = sum(packet[:10]) & 0xFF
                if checksum != packet[10]:
                    self.data_buffer.pop(0)
                    continue

                if frame_type == 0x51:  
                    accel_data = self._parse_accel_packet(packet)
                    parsed_any = True
                elif frame_type == 0x52: 
                    gyro_data = self._parse_gyro_packet(packet)
                    parsed_any = True
                elif frame_type == 0x53: 
                    angle_data = self._parse_angle_packet(packet)
                    parsed_any = True

                del self.data_buffer[:11]

            if parsed_any:
                with self.data_lock:
                    if accel_data:
                        self.current_data.accel_x, self.current_data.accel_y, self.current_data.accel_z = accel_data
                    if gyro_data:
                        self.current_data.gyro_x, self.current_data.gyro_y, self.current_data.gyro_z = gyro_data
                    if angle_data:
                        self.current_data.roll, self.current_data.pitch, self.current_data.yaw = angle_data
                    
                    self.current_data.timestamp = time.time()
                    self.current_data.valid = True

                self._check_angle_safety(self.current_data)
                return True

            return False
            
        except Exception as e:
            print(f"IMU数据读取异常: {e}")
            return False
    
    def _parse_accel_packet(self, packet: bytes) -> Tuple[float, float, float]:
        """解析加速度帧 (0x55 0x51) -> m/s²"""
        ax_raw = (packet[3] << 8) | packet[2]
        ay_raw = (packet[5] << 8) | packet[4]
        az_raw = (packet[7] << 8) | packet[6]
        
        if ax_raw >= 32768: ax_raw -= 65536
        if ay_raw >= 32768: ay_raw -= 65536
        if az_raw >= 32768: az_raw -= 65536
        
        # 转换为 m/s² (量程 ±16g)
        ax = ax_raw / 32768.0 * 16.0 * 9.8
        ay = ay_raw / 32768.0 * 16.0 * 9.8
        az = az_raw / 32768.0 * 16.0 * 9.8
        return ax, ay, az

    def _parse_gyro_packet(self, packet: bytes) -> Tuple[float, float, float]:
        """解析角速度帧 (0x55 0x52) -> rad/s"""
        wx_raw = (packet[3] << 8) | packet[2]
        wy_raw = (packet[5] << 8) | packet[4]
        wz_raw = (packet[7] << 8) | packet[6]
        
        if wx_raw >= 32768: wx_raw -= 65536
        if wy_raw >= 32768: wy_raw -= 65536
        if wz_raw >= 32768: wz_raw -= 65536
        
        # 转换为 rad/s (量程 ±2000°/s)
        wx = math.radians(wx_raw / 32768.0 * 2000.0)
        wy = math.radians(wy_raw / 32768.0 * 2000.0)
        wz = math.radians(wz_raw / 32768.0 * 2000.0)
        return wx, wy, wz

    def _parse_angle_packet(self, packet: bytes) -> Tuple[float, float, float]:
        """解析角度帧 (0x55 0x53) -> rad"""
        roll_raw = (packet[3] << 8) | packet[2]
        pitch_raw = (packet[5] << 8) | packet[4]
        yaw_raw = (packet[7] << 8) | packet[6]
        
        if roll_raw >= 32768: roll_raw -= 65536
        if pitch_raw >= 32768: pitch_raw -= 65536
        if yaw_raw >= 32768: yaw_raw -= 65536
        
        # 转换为 rad (量程 ±180°)
        roll = math.radians(roll_raw / 32768.0 * 180.0)
        pitch = math.radians(pitch_raw / 32768.0 * 180.0)
        yaw = math.radians(yaw_raw / 32768.0 * 180.0)
        return roll, pitch, yaw
    
    def _check_angle_safety(self, imu_data: IMUData):
        """检查IMU角度安全性 (只更新内部状态)"""
        if abs(imu_data.roll) > self.angle_effective_rad or abs(imu_data.pitch) > self.angle_effective_rad:
            self.imu_angle_exceed = 1
        else:
            self.imu_angle_exceed = 0
    
    def get_current_data(self) -> IMUData:
        """获取当前IMU数据（已去除偏移）"""
        with self.data_lock:
            data = self.current_data

            return IMUData(
                accel_x=data.accel_x,
                accel_y=data.accel_y,
                accel_z=data.accel_z,
                gyro_x=data.gyro_x,
                gyro_y=data.gyro_y,
                gyro_z=data.gyro_z,
                roll=data.roll - self.roll_bias,
                pitch=data.pitch - self.pitch_bias,
                yaw=data.yaw,
                timestamp=data.timestamp,
                valid=data.valid
            )
            

    
    def get_euler_angles(self) -> Tuple[float, float, float]:
        """返回去除校准偏移后的欧拉角 (roll, pitch, yaw)"""
        data = self.get_current_data()
        return data.roll, data.pitch, data.yaw

    def get_pitch(self) -> float:
        """方便函数：返回俯仰角 (弧度)。"""
        try:
            _, pitch, _ = self.get_euler_angles()
            return pitch
        except Exception as e:
            # 移除 config.debug_enabled 检查
            print(f"[IMU] Failed to get pitch: {e}")
            return 0.0

    def get_pitch_rate(self) -> float:
        """返回俯仰角角速度 (rad/s)。"""
        try:
            _, wy, _ = self.get_angular_velocity()
            return wy
        except Exception as e:
            # 移除 config.debug_enabled 检查
            print(f"[IMU] Failed to get pitch_rate: {e}")
            return 0.0

    
    def get_angular_velocity(self) -> Tuple[float, float, float]:
        """获取角速度 (wx, wy, wz)"""
        data = self.get_current_data()
        return data.gyro_x, data.gyro_y, data.gyro_z
    
    def is_data_valid(self) -> bool:
        """检查数据是否有效"""
        data = self.get_current_data()
        current_time = time.time()
        # 数据超过100ms认为过期
        return data.valid and (current_time - data.timestamp < 0.1)
    
    def is_angle_safe(self) -> bool:
        """检查角度是否安全"""
        return self.imu_angle_exceed == 0
    
    def get_compensated_pitch(self) -> float:
        """获取补偿后的俯仰角"""
        data = self.get_current_data()
        return data.pitch + self.center_pitch # 使用内部的 self.center_pitch
    
    def reset_yaw(self):
        """重置偏航角（如果支持）"""
        print("偏航角重置功能需要根据具体IMU型号实现")

    def is_calibrated(self) -> bool:
        """检查IMU是否已完成校准"""
        return self.is_calibrated_flag
    

    def calibrate(self) -> bool:
        """IMU校准，计算并存储方向偏移。"""
        print("Starting IMU calibration. Please keep the robot stationary and level...")

        calibration_samples = []
        calibration_time = 3.0
        start_time = time.time()

        while time.time() - start_time < calibration_time:
            data = self.get_current_data()
            if data.valid:
                calibration_samples.append((data.roll, data.pitch))
            time.sleep(0.01)

        if len(calibration_samples) < 100:
            print("Calibration failed: Not enough valid data samples were collected.")
            return False

        avg_roll = sum(sample[0] for sample in calibration_samples) / len(calibration_samples)
        avg_pitch = sum(sample[1] for sample in calibration_samples) / len(calibration_samples)

        self.roll_bias = avg_roll
        self.pitch_bias = avg_pitch
        self.is_calibrated_flag = True 

        print(f"✅ IMU calibration complete.")
        print(f"   - Roll bias: {math.degrees(self.roll_bias):.2f}°")
        print(f"   - Pitch bias: {math.degrees(self.pitch_bias):.2f}°")
        
        return True

    
    def close(self):
        """关闭IMU连接"""
        print("正在关闭IMU连接...")
        self.stop_reading()
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("IMU串口连接已关闭")
            except Exception as e:
                print(f"关闭IMU串口时出错: {e}")
        elif self.serial_port:
            print("IMU串口已关闭或未打开")
        else:
            print("没有IMU串口需要关闭")


# 全局IMU控制器实例
imu_controller = IMUController()