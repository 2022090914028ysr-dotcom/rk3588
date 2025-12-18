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

        self.pitch = 0.0
        self.is_calibrated_flag = False
        self.last_read_time = 0
        self.read_count = 0
        self.frequency = 0.0
        self.last_frequency_update = 0
        self.read_command = [0xff, 0xaa, 0x03, 0x0c, 0x0]
        # self.data_buffer = bytearray()
        # 调试计数器（用于控制打印频率）
        self.debug_counter = 0

        
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

                        # 调试：每隔固定次数打印一次频率和总样本数
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
                    # config.imu.data[3] = imu_data.gyro_x
                    # config.imu.data[4] = imu_data.gyro_y
                    # config.imu.data[5] = imu_data.gyro_z
                    # config.imu.data[6] = imu_data.roll
                    # config.imu.data[7] = imu_data.pitch
                    # config.imu.data[8] = imu_data.yaw
                
                # 检查角度是否超出安全范围
                self._check_angle_safety(imu_data)
                return True
            
            # 真实硬件模式：读取串口数据
            # 清空输入缓冲区
            if self.serial_port:
                self.serial_port.flushInput()
            
            # 发送读取命令
            if self.serial_port:
                self.serial_port.write(self.read_command)
            
            # 等待数据
            start_time = time.time()
            while time.time() - start_time < 0.05:  # 最多等待50ms
                if self.serial_port:
                    count = self.serial_port.inWaiting()
                    if count >= 33:  # 新格式需要33字节
                        break
                time.sleep(0.0005)
            
            if self.serial_port:
                count = self.serial_port.inWaiting()
            else:
                return False
            
            if count < 33:  # 新格式需要33字节
                return False
            
            # 读取数据
            if self.serial_port:
                com_input = self.serial_port.read(count)
                # print(f"IMU原始数据: {[hex(b) for b in com_input]}, 长度: {len(com_input)}")
                # 对齐到第一个完整的加速度帧头 (0x55 0x51)
                com = com_input
                idx = com.find(b'\x55\x51')
                if idx == -1:
                    # 没有找到帧头，丢弃本次数据，等待下次
                    return False
                # 如果从 idx 到末尾不足 33 字节，短时间补读剩余字节
                if len(com) - idx < 33:
                    need = 33 - (len(com) - idx)
                    more = bytearray()
                    end_time = time.time() + 0.01
                    while len(more) < need and time.time() < end_time:
                        more.extend(self.serial_port.read(need - len(more)))
                    if len(more) < need:
                        # 无法补齐，丢弃本次半包
                        return False
                    com = com[idx:] + bytes(more)
                else:
                    com = com[idx:idx+33]

                com_input = com
                # print(f"对齐后的IMU数据: {[hex(b) for b in com_input]}, 长度: {len(com_input)}")
            else:
                return False
            
            # 验证数据格式 - 新格式包含3个数据帧
            if (len(com_input) < 33 or 
                com_input[0] != 0x55 or com_input[1] != 0x51 or  # 加速度帧
                com_input[11] != 0x55 or com_input[12] != 0x52 or  # 角速度帧
                com_input[22] != 0x55 or com_input[23] != 0x53):   # 角度帧
                return False
            
            # 解析数据
            imu_data = self._parse_imu_data(com_input)
            if imu_data:
                with self.data_lock:
                    self.current_data = imu_data
                    # 更新全局配置中的IMU数据（保持兼容性）
                    # config.imu.data[3] = imu_data.gyro_x
                    # config.imu.data[4] = imu_data.gyro_y
                    # config.imu.data[5] = imu_data.gyro_z
                    # config.imu.data[6] = imu_data.roll
                    # config.imu.data[7] = imu_data.pitch
                    # config.imu.data[8] = imu_data.yaw
                
                # 检查角度是否超出安全范围
                self._check_angle_safety(imu_data)

                # 调试：每隔一定次数打印一次原始欧拉角和加速度，用于确认轴和符号
                self.debug_counter += 1
                if self.debug_counter % 10 == 0:
                    roll_deg = math.degrees(imu_data.roll)
                    pitch_deg = math.degrees(imu_data.pitch)
                    yaw_deg = math.degrees(imu_data.yaw)

                    # 计算重力主方向（近似）：看哪一轴的加速度绝对值最大
                    ax, ay, az = imu_data.accel_x, imu_data.accel_y, imu_data.accel_z
                    g_mag = math.sqrt(ax*ax + ay*ay + az*az)
                    axis_info = "unknown"
                    if g_mag > 1e-3:
                        nx, ny, nz = ax/g_mag, ay/g_mag, az/g_mag
                        # 选择绝对值最大的分量
                        vals = [(abs(nx), 'X', nx), (abs(ny), 'Y', ny), (abs(nz), 'Z', nz)]
                        vals.sort(reverse=True, key=lambda v: v[0])
                        _, axis_name, axis_sign = vals[0]
                        sign_str = '+' if axis_sign > 0 else '-'
                        axis_info = f"gravity ≈ {sign_str}{axis_name} 轴"

                        print(f"[IMU] Raw angles: roll={imu_data.roll:.3f} rad ({roll_deg:.2f}°), "
                          f"pitch={imu_data.pitch:.3f} rad ({pitch_deg:.2f}°), "
                          f"yaw={imu_data.yaw:.3f} rad ({yaw_deg:.1f}°)")
                        # print(f"[IMU] Linear accel: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f} m/s², {axis_info}")
                return True
            
            return False
            
        except Exception as e:
            print(f"IMU数据读取异常: {e}")
            return False
    
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