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
import atexit
from typing import List, Optional, Tuple
from dataclasses import dataclass

@dataclass
class IMUData:
    """IMU数据结构"""
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
    """IMU控制器类"""
    
    def __init__(self):
<<<<<<< HEAD
        # --- 替换 robot_config 中的 IMU/安全配置 ---
        # 1. IMU 串口配置 (硬编码)
        # if platform.system() == 'Windows':
        #     self.imu_port = "COM13"         # Windows系统默认串口
        # else:
        #     self.imu_port = "/dev/ttyS1"  # Linux系统默认串口
        # self.imu_baudrate = 230400
        # self.read_timeout = 0.01  # 10ms 超时
=======
        if platform.system() == 'Windows':
            self.imu_port = "COM13"
        else:
            self.imu_port = "/dev/ttyS4"
            
        self.imu_baudrate = 230400
        self.read_timeout = 0.01
>>>>>>> 31e5e34a157de07d1203330c8b1c5f884cf08ed1

        self.angle_effective_deg = 45.0
        self.angle_effective_rad = math.radians(self.angle_effective_deg)
        self.imu_angle_exceed = 0
        self.center_pitch = math.radians(4.0)
        
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

        self.pitch = 0.0
        self.is_calibrated_flag = False
        self.last_read_time = 0
        self.read_count = 0
        self.frequency = 0.0
        self.last_frequency_update = 0
        self.read_command = [0xff, 0xaa, 0x03, 0x0c, 0x0]
        self.debug_counter = 0

        atexit.register(self.close)
        
    def get_frequency(self) -> float:
        with self.data_lock:
            return self.frequency
            
    def initialize(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.imu_port,
                baudrate=self.imu_baudrate,
                timeout=self.read_timeout
            )
            self.serial_port.reset_input_buffer()
            print(f"IMU初始化成功 - 端口: {self.imu_port}, 波特率: {self.imu_baudrate}")
            return True
        except Exception as e:
            print(f"IMU初始化失败: {e}")
            print("尝试使用IMU模拟模式")
            self.simulation_mode = True
            self.current_data = IMUData(
                roll=0.0, pitch=0.0, yaw=0.0,
                gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
                timestamp=time.time(), valid=True
            )
            return True

    def start_reading(self):
        if not self.simulation_mode and not self.serial_port:
            print("IMU未初始化，无法开始读取")
            return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("IMU数据读取线程已启动")
        return True
    
    def stop_reading(self):
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        print("IMU数据读取已停止")
    
    def _read_loop(self):
        while self.running:
            try:
                current_time = time.time()
                
                if self._read_imu_data():
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

                        if self.read_count > 0 and self.read_count % 10 == 0:
                            print(f"[IMU] 读取频率≈ {self.frequency:.1f} Hz, 总样本数: {self.read_count}")
                    
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
                
                self._check_angle_safety(imu_data)
                return True
            
            if self.serial_port:
                self.serial_port.flushInput()
                self.serial_port.write(self.read_command)
            
            start_time = time.time()
            while time.time() - start_time < 0.05:
                if self.serial_port:
                    count = self.serial_port.inWaiting()
                    if count >= 33:
                        break
                time.sleep(0.0005)
            
            if self.serial_port:
                count = self.serial_port.inWaiting()
            else:
                return False
            
            if count < 33:
                return False
            
            if self.serial_port:
                com_input = self.serial_port.read(count)
                com = com_input
                idx = com.find(b'\x55\x51')
                if idx == -1:
                    return False
                
                if len(com) - idx < 33:
                    need = 33 - (len(com) - idx)
                    more = bytearray()
                    end_time = time.time() + 0.01
                    while len(more) < need and time.time() < end_time:
                        more.extend(self.serial_port.read(need - len(more)))
                    if len(more) < need:
                        return False
                    com = com[idx:] + bytes(more)
                else:
                    com = com[idx:idx+33]

                com_input = com
            else:
                return False
            
            if (len(com_input) < 33 or 
                com_input[0] != 0x55 or com_input[1] != 0x51 or
                com_input[11] != 0x55 or com_input[12] != 0x52 or
                com_input[22] != 0x55 or com_input[23] != 0x53):
                return False
            
            imu_data = self._parse_imu_data(com_input)
            if imu_data:
                with self.data_lock:
                    self.current_data = imu_data
                
                self._check_angle_safety(imu_data)

                self.debug_counter += 1
                if self.debug_counter % 10 == 0:
                    roll_deg = math.degrees(imu_data.roll)
                    pitch_deg = math.degrees(imu_data.pitch)
                    yaw_deg = math.degrees(imu_data.yaw)

                    print(f"[IMU] Raw angles: roll={imu_data.roll:.3f} rad ({roll_deg:.2f}°), "
                          f"pitch={imu_data.pitch:.3f} rad ({pitch_deg:.2f}°), "
                          f"yaw={imu_data.yaw:.3f} rad ({yaw_deg:.1f}°)")
                return True
            
            return False
            
        except Exception as e:
            print(f"IMU数据读取异常: {e}")
            return False
    
    def _parse_imu_data(self, data: bytes) -> Optional[IMUData]:
        try:
            ax_raw = (data[3] << 8 | data[2])
            ay_raw = (data[5] << 8 | data[4])
            az_raw = (data[7] << 8 | data[6])
            
            if ax_raw >= 32768: ax_raw -= 65536
            if ay_raw >= 32768: ay_raw -= 65536  
            if az_raw >= 32768: az_raw -= 65536
            
            accel_x = ax_raw / 32768.0 * 16 * 9.8
            accel_y = ay_raw / 32768.0 * 16 * 9.8
            accel_z = az_raw / 32768.0 * 16 * 9.8
          
            wx_raw = (data[14] << 8 | data[13])
            wy_raw = (data[16] << 8 | data[15])
            wz_raw = (data[18] << 8 | data[17])
            
            if wx_raw >= 32768: wx_raw -= 65536
            if wy_raw >= 32768: wy_raw -= 65536
            if wz_raw >= 32768: wz_raw -= 65536
            
            wx = wx_raw / 32768.0 * 2000
            wy = wy_raw / 32768.0 * 2000
            wz = wz_raw / 32768.0 * 2000
            
            gyro_x = math.radians(wx)
            gyro_y = math.radians(wy)
            gyro_z = math.radians(wz)
            
            rx_raw = (data[25] << 8 | data[24])
            ry_raw = (data[27] << 8 | data[26])
            rz_raw = (data[29] << 8 | data[28])
            
            if rx_raw >= 32768: rx_raw -= 65536
            if ry_raw >= 32768: ry_raw -= 65536
            if rz_raw >= 32768: rz_raw -= 65536
            
            rx = rx_raw / 32768.0 * 180
            ry = ry_raw / 32768.0 * 180
            rz = rz_raw / 32768.0 * 180
            
            roll = math.radians(rx)
            pitch = math.radians(ry)
            yaw = math.radians(rz)
            
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
        if abs(imu_data.roll) > self.angle_effective_rad or abs(imu_data.pitch) > self.angle_effective_rad:
            self.imu_angle_exceed = 1
        else:
            self.imu_angle_exceed = 0
    
    def get_current_data(self) -> IMUData:
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
        data = self.get_current_data()
        return data.roll, data.pitch, data.yaw

    def get_pitch(self) -> float:
        try:
            _, pitch, _ = self.get_euler_angles()
            return pitch
        except Exception as e:
            print(f"[IMU] Failed to get pitch: {e}")
            return 0.0

    def get_pitch_rate(self) -> float:
        try:
            _, wy, _ = self.get_angular_velocity()
            return wy
        except Exception as e:
            print(f"[IMU] Failed to get pitch_rate: {e}")
            return 0.0

    def get_angular_velocity(self) -> Tuple[float, float, float]:
        data = self.get_current_data()
        return data.gyro_x, data.gyro_y, data.gyro_z
    
    def is_data_valid(self) -> bool:
        data = self.get_current_data()
        current_time = time.time()
        return data.valid and (current_time - data.timestamp < 0.1)
    
    def is_angle_safe(self) -> bool:
        return self.imu_angle_exceed == 0
    
    def get_compensated_pitch(self) -> float:
        data = self.get_current_data()
        return data.pitch + self.center_pitch
    
    def reset_yaw(self):
        print("偏航角重置功能需要根据具体IMU型号实现")

    def is_calibrated(self) -> bool:
        return self.is_calibrated_flag
    
    def calibrate(self) -> bool:
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

imu_controller = IMUController()