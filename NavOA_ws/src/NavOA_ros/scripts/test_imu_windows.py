#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Windows系统下IMU硬件连接测试脚本
用于验证Windows系统下是否能正确尝试连接真实IMU硬件
"""

import sys
import os
import platform

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from imu_controller import IMUController, config

def test_imu_connection():
    """测试IMU连接"""
    print("=== Windows系统IMU硬件连接测试 ===")
    print(f"操作系统: {platform.system()}")
    print(f"配置的IMU串口: {config.imu.port}")
    print(f"波特率: {config.imu.baudrate}")
    print(f"超时时间: {config.imu.timeout}")
    print()
    
    # 创建IMU控制器
    imu = IMUController()
    
    print("正在尝试初始化IMU...")
    success = imu.initialize()
    
    if success:
        if imu.simulation_mode:
            print("✓ IMU初始化成功 - 模拟模式")
            print("  原因: 无法连接到真实硬件，自动切换到模拟模式")
            print("  说明: 如需使用真实硬件，请确保:")
            print("    1. IMU设备已正确连接到USB端口")
            print("    2. 已安装正确的USB转串口驱动")
            print("    3. 在设备管理器中确认串口号")
            print("    4. 串口未被其他程序占用")
        else:
            print("✓ IMU初始化成功 - 真实硬件模式")
            print("  成功连接到真实IMU硬件")
            
            # 启动数据读取
            print("\n正在启动数据读取...")
            imu.start_reading()
            
            # 持续读取数据（按Ctrl+C停止）
            import time
            import math
            print("  正在持续读取IMU数据，按Ctrl+C停止...")
            try:
                count = 0
                while True:
                    time.sleep(0.1)
                    data = imu.get_current_data()
                    roll_deg = math.degrees(data.roll)
                    pitch_deg = math.degrees(data.pitch)
                    yaw_deg = math.degrees(data.yaw)
                    
                    # 角速度 (°/s)
                    gyro_x_deg = math.degrees(data.gyro_x)
                    gyro_y_deg = math.degrees(data.gyro_y)
                    gyro_z_deg = math.degrees(data.gyro_z)
                    
                    # 加速度 (m/s²)
                    accel_x = data.accel_x
                    accel_y = data.accel_y
                    accel_z = data.accel_z
                    
                    count += 1
                    print(f"  数据 {count}: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°")
                    print(f"           角速度: X={gyro_x_deg:.3f}, Y={gyro_y_deg:.3f}, Z={gyro_z_deg:.3f} °/s")
                    print(f"           加速度: X={accel_x:.3f}, Y={accel_y:.3f}, Z={accel_z:.3f} m/s²")
            except KeyboardInterrupt:
                print("\n  用户停止数据读取")
            
            imu.stop_reading()
    else:
        print("✗ IMU初始化失败")
    
    # 清理
    imu.close()
    print("\n测试完成")

if __name__ == "__main__":
    test_imu_connection()