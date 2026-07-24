#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
IMU控制器模块
提供IMU数据读取和处理功能
"""

import serial
import math
import time
import threading
import platform
from typing import List, Optional, Tuple
from dataclasses import dataclass


# ──────────────────────────────────────────────
# IMU / Robot 配置（内联，不依赖外部 config 模块）
# ──────────────────────────────────────────────
class IMUConfig:
    """IMU配置"""
    def __init__(self):
        import platform
        if platform.system() == 'Windows':
            self.port = "COM13"
        else:
            self.port = "/dev/ttyS6"
        self.baudrate = 230400
        self.timeout = 0.001
        self.angle_effective = 45  # 度
        self.data = [0.0] * 13      # [ax, ay, az, wx, wy, wz, roll, pitch, yaw, ...]
        self.angle_exceed = 0
        self.y_wc = 0.0


class SafetyConfig:
    """安全配置"""
    def __init__(self):
        self.imu_angle_exceed = 0


class RobotConfig:
    """机器人全局配置"""
    def __init__(self):
        self.imu = IMUConfig()
        self.safety = SafetyConfig()
        self.debug_enabled = False


# 全局配置实例
config = RobotConfig()
# ──────────────────────────────────────────────


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
        self.serial_port = None
        self.running = False
        self.read_thread = None
        self.current_data = IMUData()
        self.data_lock = threading.Lock()
        self.error_count = 0
        self.max_error_count = 10
        self.simulation_mode = False  # 模拟模式标志
        self.roll_bias = 0.0
        self.pitch_bias = 0.0

        self.pitch = 0.0
        self.is_calibrated_flag = False
        # Frequency tracking
        self.last_read_time = 0
        self.read_count = 0
        self.frequency = 0.0
        self.last_frequency_update = 0
        # IMU读取命令
        self.read_command = [0xff, 0xaa, 0x03, 0x0c, 0x0]
        # 串口流式缓存：用于从任意字节边界重同步并提取完整帧
        self._rx_buffer = bytearray()
        self._max_rx_buffer_size = 2048
        self.frequency = 0.0
        
        # Register cleanup handler
        import atexit
        atexit.register(self.close)
        
    def get_frequency(self) -> float:
        """获取实际的IMU读取频率"""
        with self.data_lock:
            return self.frequency
            
    def initialize(self) -> bool:
        """初始化IMU串口连接"""
        # 尝试使用真实串口连接
        try:
            self.serial_port = serial.Serial(
                port=config.imu.port,
                baudrate=config.imu.baudrate,
                timeout=config.imu.timeout
            )
            print(f"IMU初始化成功 - 端口: {config.imu.port}, 波特率: {config.imu.baudrate}")
            return True
        except Exception as e:
            print(f"IMU初始化失败: {e}")
            # 如果串口初始化失败，尝试使用模拟模式
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
                            # Update frequency every 10 readings for smoother values
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
                        self.error_count = 0  # 重置计数器，继续尝试
                time.sleep(0.001)  # 500Hz读取频率
            except Exception as e:
                print(f"IMU读取循环异常: {e}")
                time.sleep(0.1)
    
    def _read_imu_data(self) -> bool:
        """读取一次IMU数据"""
        try:
            # 模拟模式：生成模拟数据
            if self.simulation_mode:
                # 生成轻微变化的模拟IMU数据
                current_time = time.time()
                # 添加小幅度的正弦波动来模拟真实的IMU数据
                roll_offset = math.sin(current_time * 0.5) * 0.02  # ±0.02 rad
                pitch_offset = math.cos(current_time * 0.3) * 0.01  # ±0.01 rad
                yaw_offset = math.sin(current_time * 0.1) * 0.005   # ±0.005 rad
                
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
                    # 更新全局配置中的IMU数据（保持兼容性）
                    config.imu.data[0] = imu_data.accel_x
                    config.imu.data[1] = imu_data.accel_y
                    config.imu.data[2] = imu_data.accel_z
                    config.imu.data[3] = imu_data.gyro_x
                    config.imu.data[4] = imu_data.gyro_y
                    config.imu.data[5] = imu_data.gyro_z
                    config.imu.data[6] = imu_data.roll
                    config.imu.data[7] = imu_data.pitch
                    config.imu.data[8] = imu_data.yaw
                
                # 检查角度是否超出安全范围
                self._check_angle_safety(imu_data)
                return True
            
            # 真实硬件模式：读取串口数据
            # 发送读取命令
            if self.serial_port:
                self.serial_port.write(self.read_command)
            
            # 等待数据
            start_time = time.time()
            while time.time() - start_time < 0.05:  # 最多等待50ms
                if self.serial_port:
                    count = self.serial_port.in_waiting
                    if count >= 22:  # 需要22字节(0x52+0x53)
                        break
                time.sleep(0.0005)
            
            if self.serial_port:
                count = self.serial_port.in_waiting
            else:
                return False
            
            if count <= 0:
                return False
            
            # 读取数据
            if self.serial_port:
                com_input = self.serial_port.read(count)
            else:
                return False

            if not com_input:
                return False

            self._rx_buffer.extend(com_input)
            if len(self._rx_buffer) > self._max_rx_buffer_size:
                # 保留尾部数据，避免缓存无限增长
                self._rx_buffer = self._rx_buffer[-self._max_rx_buffer_size:]

            frame_block = self._extract_imu_frame_block()
            if frame_block is None:
                return False
            
            # 解析数据
            imu_data = self._parse_imu_data(frame_block)
            if imu_data:
                with self.data_lock:
                    self.current_data = imu_data
                    # 更新全局配置中的IMU数据（保持兼容性）
                    config.imu.data[0] = imu_data.accel_x
                    config.imu.data[1] = imu_data.accel_y
                    config.imu.data[2] = imu_data.accel_z
                    config.imu.data[3] = imu_data.gyro_x
                    config.imu.data[4] = imu_data.gyro_y
                    config.imu.data[5] = imu_data.gyro_z
                    config.imu.data[6] = imu_data.roll
                    config.imu.data[7] = imu_data.pitch
                    config.imu.data[8] = imu_data.yaw
                
                # 检查角度是否超出安全范围
                self._check_angle_safety(imu_data)
                return True
            
            return False
            
        except Exception as e:
            print(f"IMU数据读取异常: {e}")
            return False

    def _is_valid_wit_frame(self, frame: bytes, frame_type: int) -> bool:
        """校验单个WIT 11字节数据帧。"""
        if len(frame) != 11:
            return False
        if frame[0] != 0x55 or frame[1] != frame_type:
            return False
        checksum = sum(frame[:10]) & 0xFF
        return checksum == frame[10]

    def _extract_imu_frame_block(self) -> Optional[bytes]:
        """
        从接收缓存中提取连续三帧(0x51/0x52/0x53)。
        若存在错位或坏帧，会自动丢弃前导字节并重同步。
        """
        frame_block_size = 33
        while len(self._rx_buffer) >= frame_block_size:
            head_idx = self._rx_buffer.find(0x55)
            if head_idx < 0:
                self._rx_buffer.clear()
                return None

            if head_idx > 0:
                del self._rx_buffer[:head_idx]
                if len(self._rx_buffer) < frame_block_size:
                    return None

            candidate = bytes(self._rx_buffer[:frame_block_size])
            frame_acc = candidate[0:11]
            frame_gyro = candidate[11:22]
            frame_angle = candidate[22:33]

            if (
                self._is_valid_wit_frame(frame_acc, 0x51)
                and self._is_valid_wit_frame(frame_gyro, 0x52)
                and self._is_valid_wit_frame(frame_angle, 0x53)
            ):
                del self._rx_buffer[:frame_block_size]
                return candidate

            # 非法块：滑动1字节继续找，完成重同步
            del self._rx_buffer[0]
        return None
    
    def _parse_imu_data(self, data: bytes) -> Optional[IMUData]:
        """解析IMU原始数据 - 新33字节格式"""
        try:
            # 新格式包含3个数据帧：
            # 帧1 (0-10): 0x55 0x51 + 加速度数据
            # 帧2 (11-21): 0x55 0x52 + 角速度数据  
            # 帧3 (22-32): 0x55 0x53 + 角度数据
            
            # 解析加速度数据 (帧1: 0x55 0x51)
            # 加速度数据位于字节2-7
            ax_raw = (data[3] << 8 | data[2])  # 字节2-3
            ay_raw = (data[5] << 8 | data[4])  # 字节4-5
            az_raw = (data[7] << 8 | data[6])  # 字节6-7
            
            # 转换为有符号整数并计算加速度值 (g)
            if ax_raw >= 32768: ax_raw -= 65536
            if ay_raw >= 32768: ay_raw -= 65536  
            if az_raw >= 32768: az_raw -= 65536
            
            accel_x = ax_raw / 32768.0 * 16 * 9.8  # ±16g量程，转换为m/s²
            accel_y = ay_raw / 32768.0 * 16 * 9.8
            accel_z = az_raw / 32768.0 * 16 * 9.8

            # # 暂时将加速度设为静止状态的理论值
            # # 静止时，Z轴应该感受到重力加速度约1g，X和Y轴应该接近0
            # # 这样可以提供更合理的静止状态显示
            # accel_x = 0.0  # X轴加速度
            # accel_y = 0.0  # Y轴加速度 
            # accel_z = 9.8  # Z轴重力加速度（静止时约1g）
          
            # 解析角速度数据 (帧2: 0x55 0x52)
            # 角速度数据位于字节13-18 (相对于整个数据包)
            wx_raw = (data[14] << 8 | data[13])  # 字节13-14
            wy_raw = (data[16] << 8 | data[15])  # 字节15-16
            wz_raw = (data[18] << 8 | data[17])  # 字节17-18
            
            # 转换为有符号整数并计算角速度值 (°/s)
            if wx_raw >= 32768: wx_raw -= 65536
            if wy_raw >= 32768: wy_raw -= 65536
            if wz_raw >= 32768: wz_raw -= 65536
            
            wx = wx_raw / 32768.0 * 2000  # ±2000°/s量程
            wy = wy_raw / 32768.0 * 2000
            wz = wz_raw / 32768.0 * 2000
            
            gyro_x = math.radians(wx)
            gyro_y = math.radians(wy)
            gyro_z = math.radians(wz)
            
            # 解析角度数据 (帧3: 0x55 0x53)
            # 角度数据位于字节24-29 (相对于整个数据包)
            rx_raw = (data[25] << 8 | data[24])  # 字节24-25
            ry_raw = (data[27] << 8 | data[26])  # 字节26-27
            rz_raw = (data[29] << 8 | data[28])  # 字节28-29
            
            # 转换为有符号整数并计算角度值 (°)
            if rx_raw >= 32768: rx_raw -= 65536
            if ry_raw >= 32768: ry_raw -= 65536
            if rz_raw >= 32768: rz_raw -= 65536
            
            rx = rx_raw / 32768.0 * 180  # ±180°量程
            ry = ry_raw / 32768.0 * 180
            rz = rz_raw / 32768.0 * 180
        
            roll = math.radians(rx)
            pitch = math.radians(ry)
            yaw = math.radians(rz)
            mag = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
            # print(f"Accel magnitude: {mag:.2f} m/s²")
            
            return IMUData(
                accel_x=accel_x,
                accel_y=accel_y,
                accel_z=accel_z,
                gyro_x=gyro_x,
                gyro_y=gyro_y,
                gyro_z=gyro_z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                timestamp=time.time(),
                valid=True
            )
        except Exception as e:
            print(f"IMU数据解析失败: {e}")
            return None
    
    def _check_angle_safety(self, imu_data: IMUData):
        """检查IMU角度安全性"""
        angle_limit = math.radians(config.imu.angle_effective)
        
        if abs(imu_data.roll) > angle_limit or abs(imu_data.pitch) > angle_limit:
            config.imu.angle_exceed = 1
            config.safety.imu_angle_exceed = 1
        else:
            config.imu.angle_exceed = 0
            config.safety.imu_angle_exceed = 0
    
    def get_current_data(self) -> IMUData:
        """获取当前IMU数据（已去除偏移）"""
        with self.data_lock:
            data = self.current_data
            # Make a copy so we don't overwrite original
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

    def get_raw_data(self) -> IMUData:
        """获取当前IMU原始数据（未去除roll/pitch偏移）"""
        with self.data_lock:
            data = self.current_data
            return IMUData(
                accel_x=data.accel_x,
                accel_y=data.accel_y,
                accel_z=data.accel_z,
                gyro_x=data.gyro_x,
                gyro_y=data.gyro_y,
                gyro_z=data.gyro_z,
                roll=data.roll,
                pitch=data.pitch,
                yaw=data.yaw,
                timestamp=data.timestamp,
                valid=data.valid
            )

    def get_raw_and_current_data(self) -> Tuple[IMUData, IMUData]:
        """同时获取同一帧IMU原始数据和去偏移后的数据。"""
        with self.data_lock:
            data = self.current_data
            raw_data = IMUData(
                accel_x=data.accel_x,
                accel_y=data.accel_y,
                accel_z=data.accel_z,
                gyro_x=data.gyro_x,
                gyro_y=data.gyro_y,
                gyro_z=data.gyro_z,
                roll=data.roll,
                pitch=data.pitch,
                yaw=data.yaw,
                timestamp=data.timestamp,
                valid=data.valid
            )
            corrected_data = IMUData(
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
            return raw_data, corrected_data
            

    
    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Return Euler angles (roll, pitch, yaw) with calibration bias removed.
        IMPORTANT: This is the single source of truth for orientation.
        """
        data = self.get_current_data()

        roll = data.roll
        pitch = data.pitch
        yaw = data.yaw

        # If your IMU is mounted upside-down, flip here:
        # pitch = -pitch

        return roll, pitch, yaw

    def get_pitch(self) -> float:
        """Convenience: return only pitch angle (radians)."""
        try:
            _, pitch, _ = self.get_euler_angles()
            return pitch
        except Exception as e:
            if config.debug_enabled:
                print(f"[IMU] Failed to get pitch: {e}")
            return 0.0

    def get_pitch_rate(self) -> float:
        """Return pitch angular velocity (rad/s)."""
        try:
            _, wy, _ = self.get_angular_velocity()
            return wy
        except Exception as e:
            if config.debug_enabled:
                print(f"[IMU] Failed to get pitch_rate: {e}")
            return 0.0


    
    def get_angular_velocity(self) -> Tuple[float, float, float]:
        """获取角速度 (wx, wy, wz)"""
        data = self.get_current_data()
        return data.gyro_x, data.gyro_y, data.gyro_z
    
    def is_data_valid(self) -> bool:
        """检查数据是否有效"""
        data = self.get_current_data()
        # 检查数据是否在合理范围内且时间戳不太旧
        if not data.valid:
            return False
        
        current_time = time.time()
        if current_time - data.timestamp > 0.1:  # 数据超过100ms认为过期
            return False
        
        return True
    
    def is_angle_safe(self) -> bool:
        """检查角度是否安全"""
        return config.imu.angle_exceed == 0
    
    # def get_compensated_pitch(self) -> float:
    #     """获取补偿后的俯仰角"""
    #     data = self.get_current_data()
    #     return data.pitch # + config.center_y
    
    def reset_yaw(self):
        """重置偏航角（如果支持）"""
        # 这里可以实现偏航角重置逻辑
        # 具体实现取决于IMU型号
        print("偏航角重置功能需要根据具体IMU型号实现")

    def is_calibrated(self) -> bool:
        """检查IMU是否已完成校准"""
        return self.is_calibrated_flag
    

    def calibrate(self) -> bool:
        """
        Calibrates the IMU by calculating and storing the orientation bias.
        
        This function collects sensor data for a few seconds while the robot is
        expected to be stationary and level. It then averages this data to
        determine the zero-offset for roll and pitch.
        """
        print("Starting IMU calibration. Please keep the robot stationary and level...")

        calibration_samples = []
        # Collect 3 seconds of data for a stable average.
        calibration_time = 3.0
        start_time = time.time()

        while time.time() - start_time < calibration_time:
            # Get the latest raw data from the IMU.
            data = self.get_current_data()
            if data.valid:
                # Store the roll and pitch readings.
                calibration_samples.append((data.roll, data.pitch))
            # Sample at roughly 100Hz.
            time.sleep(0.01)

        # Ensure we have enough data to form a reliable average.
        if len(calibration_samples) < 100:
            print("Calibration failed: Not enough valid data samples were collected.")
            return False

        # Calculate the average roll and pitch over the calibration period.
        avg_roll = sum(sample[0] for sample in calibration_samples) / len(calibration_samples)
        avg_pitch = sum(sample[1] for sample in calibration_samples) / len(calibration_samples)

        # Store the calculated averages as the bias values.
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