#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
机器人配置文件
支持10个电机的配置管理
"""

import torch
import global_config
from dataclasses import dataclass
from typing import Dict, List, Tuple
import math

@dataclass
class MotorConfig:
    """单个电机配置"""
    id: int
    name: str
    min_angle: float
    max_angle: float
    init_angle: float
    stiffness: float
    damping: float
    can_channel: str = 'can0'
    max_torque: float = 10.0  # 最大扭矩限制 (Nm)
    reverse:bool = True
    # 实时状态
    current_angle: float = 0.0
    current_velocity: float = 0.0
    current_torque: float = 0.0
    target_angle: float = 0.0


class RobotConfig:
    """机器人整体配置类"""

    def __init__(self):
        # 控制参数
        self.decimation = 4
        self.dt = 0.02
        self.control_frequency = 100 # Hz

        # 多线程RL控制器配置
        self.threaded_rl_enabled = True  # 启用多线程RL模式
        self.rl_inference_frequency = 50  # RL推理频率 (Hz)
        self.max_action_age = 0.1  # 最大动作年龄 (秒)
        self.action_buffer_size = 10  # 动作缓冲区大小
        self.enable_action_prediction = True  # 启用动作预测
        self.fallback_mode = 'last_action'  # 降级模式: 'last_action', 'prediction', 'hold_position'
        self.max_fallback_duration = 1.0  # 最大降级持续时间 (秒)

        # UDP数据发送配置
        self.udp_enabled = False  # 启用UDP数据发送
        self.udp_target_ip = "192.168.110.255"  # 广播地址（根据当前网络调整）
        self.udp_target_port = 5005  # 广播端口
        self.udp_send_frequency = 10.0  # UDP发送频率 (Hz)
        self.udp_buffer_size = 100  # UDP发送缓冲区大小
        self.udp_timeout = 0.001  # UDP发送超时时间 (秒)
        self.udp_max_packet_size = 65507  # UDP最大包大小 (bytes)

        # 调试配置 (Debug Configuration)
        self.debug_enabled = True  # 总调试开关 (Overall Debug Switch)
        self.imu_debug_enabled = False  # IMU观测值调试日志  update_observation : robot_state (IMU Observation Debug Log)
        self.joint_debug_enabled = False  # 关节状态观测值调试日志 update_observation : robot_state (Joint Status Observation Debug Log)
        self.control_debug_enabled = False  # 遥控器控制命令调试日志  update_observation : robot_state (Remote Control Command Debug Log)
        self.model_debug_enabled = False  # 模型推理结果调试日志    get_rl_actions : robot_controller (Model Inference Result Debug Log)
        self.motor_debug_enabled = False  # 电机发送命令调试日志   send_motor_command : motor_controller (Motor Send Command Debug Log)
        self.performance_debug_enabled = True  # 机器人总体性能监控调试日志 _control_loop : robot_controller (Robot Overall Performance Monitoring Debug Log)
        self.motor_debug_counter = 0  # 电机调试计数器 (Motor Debug Counter)
        self.debug_print_interval = 100  # 调试信息打印间隔（每N次循环打印一次）(Debug Information Print Interval - Print Every N Cycles)

        # 运动命令 (Motion Commands)
        self.commands = [0.0, 0.0, 0.0]  # [x_vel, y_vel, yaw_vel]
        self.center_y = math.radians(4.0)
        self.run_speed = global_config.run_speed  # 运行速度 (Run Speed)
        print(f"[CONFIG] 设置运行速度为: {self.run_speed} m/s")
        self.start = 0

        # 10个电机配置 - 基于用户提供的训练参数 (10 Motor Configurations - Based on User-Provided Training Parameters)
        self.motors = {
            # 左腿电机 (5个) (Left Leg Motors (5))
            'J_L0': MotorConfig(
                id=1, name='J_L0',
                min_angle=-0.7, max_angle=0.7, init_angle=0.0,
                stiffness=15, damping=1, can_channel='can1', max_torque=8.0                          # 13 0.3
            ),
            'J_L1': MotorConfig(
                id=2, name='J_L1',
                min_angle=-0.38, max_angle=0.46, init_angle=-0.08,
                stiffness=25, damping=1, can_channel='can1', max_torque=12.0                         # 15 0.65
            ),
            'J_L2': MotorConfig(
                id=3, name='J_L2',
                reverse=False,
                min_angle=-1.57, max_angle=1.57, init_angle=0.56,
                stiffness=25, damping=0.65, can_channel='can1', max_torque=15.0                      # 15 0.65
            ),
            'J_L3': MotorConfig(
                id=4, name='J_L3',
                min_angle=-2.35, max_angle=0, init_angle=-1.12,
                stiffness=25, damping=0.65, can_channel='can1', max_torque=15.0                      # 15 0.65
            ),
            'J_L4_ankle': MotorConfig(
                id=5, name='J_L4_ankle',
                min_angle=-1.2, max_angle=1.2, init_angle=-0.57,
                stiffness=15, damping=0.65, can_channel='can1', max_torque=8.0                       # 13 0.3
            ),

            # 右腿电机 (5个) (Right Leg Motors (5))
            'J_R0': MotorConfig(
                id=6, name='J_R0',
                min_angle=-0.7, max_angle=0.7, init_angle=0.0,
                stiffness=15, damping=1, can_channel='can0', max_torque=8.0
            ),
            'J_R1': MotorConfig(
                id=7, name='J_R1',
                min_angle=-0.38, max_angle=0.47, init_angle=0.08,
                stiffness=25, damping=1, can_channel='can0', max_torque=12.0
            ),
            'J_R2': MotorConfig(
                id=8, name='J_R2',
                reverse=False,
                min_angle=-1.57, max_angle=1.57, init_angle=-0.56,
                stiffness=25, damping=0.65, can_channel='can0', max_torque=15.0
            ),
            'J_R3': MotorConfig(
                id=9, name='J_R3',
                min_angle=0, max_angle=2.35, init_angle=1.12,
                stiffness=25, damping=0.65, can_channel='can0', max_torque=15.0
            ),
            'J_R4_ankle': MotorConfig(
                id=10, name='J_R4_ankle',
                min_angle=-1.2, max_angle=1.2, init_angle=0.57,
                stiffness=15, damping=0.65, can_channel='can0', max_torque=8.0
            )
        }

        # 默认关节角度字典 (Default Joint Angles Dictionary - Used for Training)
        self.default_joint_angles = {
            'J_L0':  0.0,
            'J_L1': -0.08,
            'J_L2':  0.56,
            'J_L3': -1.12,
            'J_L4_ankle': -0.57,
            'J_R0':  0.0,
            'J_R1':  0.08,
            'J_R2': -0.56,
            'J_R3':  1.12,
            'J_R4_ankle': 0.57
        }

        # 站立姿态关节角度 (Standing Posture Joint Angles)
        self.default_joint_angles_st = {
            'J_L0':  0.0,
            'J_L1': -0.08,
            'J_L2':  0.56,
            'J_L3': -1.12,
            'J_L4_ankle': -0.57,
            'J_R0':  0.0,
            'J_R1':  0.08,
            'J_R2': -0.56,
            'J_R3':  1.12,
            'J_R4_ankle': 0.57
        }

        # === Stabilizer parameters ===
        self.stabilizer = {
            "kp_pitch": 0.15,       # 比例增益 (根据需要调整) (Proportional Gain - Adjust as Needed)
            "kd_pitch": 0.02,       # 微分增益 (Derivative Gain)
            "torque_clip": 0.3,     # 最大补偿扭矩 (Maximum Compensation Torque)

            "hip_bias_ratio": 0.5,  # 髋关节分担比例 (0.5 = 50%) (Hip Joint Share Ratio - 0.5 = 50%)
            "ankle_bias_ratio": 1.0 # 踝关节分担比例 (1.0 = 100%) (Ankle Joint Share Ratio - 1.0 = 100%)
        }
        # self.stabilizer = stabilizer
        # 仿真参数
        self.sim = SimConfig()

        # IMU配置
        self.imu = IMUConfig()

        # 安全参数
        self.safety = SafetyConfig()

        # 补偿参数
        self.compensation = CompensationConfig()

        # 训练增益配置（与训练环境保持一致）
        self.training_gains = TrainingGainsConfig()

        # 默认模型路径配置
        self.model_paths = ModelPathsConfig()

    def get_motor_by_id(self, motor_id: int) -> MotorConfig:
        """根据ID获取电机配置"""
        for motor in self.motors.values():
            if motor.id == motor_id:
                return motor
        raise ValueError(f"Motor with ID {motor_id} not found")

    def get_motor_by_name(self, name: str) -> MotorConfig:
        """根据名称获取电机配置"""
        if name in self.motors:
            return self.motors[name]
        raise ValueError(f"Motor with name {name} not found")

    def get_motors_by_channel(self, channel: str) -> List[MotorConfig]:
        """根据CAN通道获取电机列表"""
        return [motor for motor in self.motors.values() if motor.can_channel == channel]

    def get_motor_init_angles_tensor(self) -> torch.Tensor:
        """获取所有电机初始角度的张量"""
        angles = [motor.init_angle for motor in self.motors.values()]
        return torch.tensor([angles])

    def get_motor_names_ordered(self) -> List[str]:
        """获取按ID排序的电机名称列表"""
        sorted_motors = sorted(self.motors.items(), key=lambda x: x[1].id)
        return [name for name, _ in sorted_motors]


class SimConfig:
    """仿真相关配置 - 兼容训练模型的10维位置偏移量格式"""
    def __init__(self):
        self.clip_observations = 100
        self.clip_actions = 100
        self.ang_vel = 0.3
        self.commands_scale = [0.8, 0.8, self.ang_vel]
        self.dof_pos = 1.0
        self.dof_vel = 0.05
        self.num_obs = 39
        self.n_proprio = 39
        self.history_len = 10
        self.quat = 0.25
        # 低通滤波系数（仿真/控制之间一致性）
        self.action_filter = 0.5
        # 训练模型兼容配置
        self.num_actions = 10  # 10个关节的位置偏移量（与训练模型保持一致）
        self.action_scale = 0.25  # 动作缩放因子（与训练时保持一致）
        self.control_type = "P"   # 控制类型：P(位置), V(速度), T(扭矩)
        # 电机数量
        self.num_motors = 10

class IMUConfig:
    """IMU配置"""

    def __init__(self):
        # 根据操作系统自动选择串口
        import platform
        if platform.system() == 'Windows':
            self.port = "COM13"         # Windows系统默认串口
        else:
            self.port = "/dev/ttyS4"  # Linux系统默认串口
        self.baudrate = 230400
        self.timeout = 0.001
        self.angle_effective = 45  # 度
        # IMU数据 [ax, ay, az, wx, wy, wz, roll, pitch, yaw, ...]
        self.data = [0.0] * 13
        self.angle_exceed = 0
        self.y_wc = 0.0


class CompensationConfig:
    """补偿参数配置"""

    def __init__(self):

        self.hip_gain = 0.55
        self.ankle_gain = 0.7
        self.max_bias = 0.3  
        self.gravity_gain = 0.1
        self.ratio = 0.5
        # 重力补偿系数
        self.gravity_compensation = {
            'hip_yaw_left': 0.1,
            'hip_roll_left': 0.15,
            'hip_pitch_left': 0.2,
            'knee_left': 0.25,
            'ankle_pitch_left': 0.1,
            'hip_yaw_right': 0.1,
            'hip_roll_right': 0.15,
            'hip_pitch_right': 0.2,
            'knee_right': 0.25,
            'ankle_pitch_right': 0.1
        }

        # 动态补偿系数
        self.dynamic_compensation = {
            'hip_yaw_left': 0.05,
            'hip_roll_left': 0.08,
            'hip_pitch_left': 0.1,
            'knee_left': 0.12,
            'ankle_pitch_left': 0.05,
            'hip_yaw_right': 0.05,
            'hip_roll_right': 0.08,
            'hip_pitch_right': 0.1,
            'knee_right': 0.12,
            'ankle_pitch_right': 0.05
        }

        # 摩擦补偿系数
        self.friction_compensation = {
            'hip_yaw_left': 0.02,
            'hip_roll_left': 0.03,
            'hip_pitch_left': 0.04,
            'knee_left': 0.05,
            'ankle_pitch_left': 0.02,
            'hip_yaw_right': 0.02,
            'hip_roll_right': 0.03,
            'hip_pitch_right': 0.04,
            'knee_right': 0.05,
            'ankle_pitch_right': 0.02
        }

        # 不同场景下的速度比例
        self.velocity_ratio = {
            'execute_actions': 1.0,    # 执行RL动作时的速度比例
            'hold_position': 0.5,      # 保持位置时的速度比例
            'move_to_initial': 0.8,    # 移动到初始位置时的速度比例
            'test_motors': 0.3         # 测试电机时的速度比例
        }

        # 不同场景下的扭矩比例
        self.torque_ratio = {
            'execute_actions': 1.0,    # 执行RL动作时的扭矩比例
            'hold_position': 0.8,      # 保持位置时的扭矩比例
            'move_to_initial': 0.9,    # 移动到初始位置时的扭矩比例
            'test_motors': 0.5         # 测试电机时的扭矩比例
        }
        # 默认最大扭矩限制
        self.default_max_torque = 10.0  # Nm

    # def compute_bias(self, pitch: float, imu_controller=None):
    #     """Compute pitch-based compensation for hips & ankles."""
    #     raw_pitch = 0.0
    #     if imu_controller is not None:
    #         try:
    #             raw_pitch = imu_controller.get_current_data().pitch
    #         except Exception:
    #             raw_pitch = 0.0
    #     error = pitch - raw_pitch  # corrected pitch
    #     hip_bias = self.hip_gain * error
    #     ankle_bias = self.ankle_gain * error
    #     # clamp
    #     hip_bias = max(-self.max_bias, min(self.max_bias, hip_bias))
    #     ankle_bias = max(-self.max_bias, min(self.max_bias, ankle_bias))

    #     # print(f"[COMP] raw={raw_pitch:.3f}, target={pitch:.3f}, "
    #     #     f"hip_bias={hip_bias:.3f}, ankle_bias={ankle_bias:.3f}, ratio={self.ratio:.2f}")
    #     return hip_bias, ankle_bias

class TrainingGainsConfig:
    """训练增益配置 - 与训练环境的PD增益保持一致"""

    def __init__(self):
        # 训练时使用的P增益（10个关节）
        # 基于legged_robot.py中的默认增益设置
        self.p_gains = [
            20.0,  # J_L0 (左髋偏航)
            20.0,  # J_L1 (左髋侧摆)
            20.0,  # J_L2 (左髋俯仰)
            20.0,  # J_L3 (左膝)
            20.0,  # J_L4_ankle (左踝)
            20.0,  # J_R0 (右髋偏航)
            20.0,  # J_R1 (右髋侧摆)
            20.0,  # J_R2 (右髋俯仰)
            20.0,  # J_R3 (右膝)
            20.0   # J_R4_ankle (右踝) 
        ]

        # 训练时使用的D增益（10个关节）
        self.d_gains = [
            0.5,   # J_L0 (左髋偏航)
            0.5,   # J_L1 (左髋侧摆)
            0.5,   # J_L2 (左髋俯仰)
            0.5,   # J_L3 (左膝)
            0.5,   # J_L4_ankle (左踝)
            0.5,   # J_R0 (右髋偏航)
            0.5,   # J_R1 (右髋侧摆)
            0.5,   # J_R2 (右髋俯仰)
            0.5,   # J_R3 (右膝)
            0.5    # J_R4_ankle (右踝)
        ]


class SafetyConfig:
    """安全配置"""

    def __init__(self):
        self.motor_angle_exceed = 0
        self.imu_angle_exceed = 0
        self.emergency_stop = False

        # MIT模式参数范围
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.T_MIN = -12.0
        self.T_MAX = 12.0
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0


class ModelPathsConfig:
    """模型路径配置"""
    def __init__(self):
        # 模型目录
        self.models_dir = "models"

        # 动作模型配置 - 支持多种动作类型
        self.action_models = {
            'walking': {
                'path': 'models/model_jit_0828.pt',
                'name': '行走模型',
                'description': '用于前进、后退、转向等运动',
                'priority': 1,  # 优先级，数字越小优先级越高
                'trigger_conditions': {
                    'min_linear_velocity':  0.02,
                    'min_angular_velocity': 0.01
                }
            },
            'standing': {
                'path': 'models/model_jit_stand_1128.pt',
                'name': '站立模型',
                'description': '用于静止平衡和姿态保持',
                'priority': 2,
                'trigger_conditions': {
                    'max_linear_velocity':  0.02,
                    'max_angular_velocity': 0.02,
                    'hold_time': 1.0  # 需要保持静止1秒才切换
                }
            },
            'running': {
                'path': 'models/policy_running.pt',
                'name': '跑步模型',
                'description': '用于高速运动',
                'priority': 0,
                'trigger_conditions': {
                    'min_linear_velocity':  0.8,
                    'min_angular_velocity': 0.1
                }
            },
            'climbing': {
                'path': 'models/policy_climbing.pt',
                'name': '爬坡模型',
                'description': '用于爬坡和复杂地形',
                'priority': 1,
                'trigger_conditions': {
                    'manual_trigger': True  # 需要手动触发
                }
            },
            'dancing': {
                'path': 'models/policy_dancing.pt',
                'name': '舞蹈模型',
                'description': '用于舞蹈动作',
                'priority': 3,
                'trigger_conditions': {
                    'manual_trigger': True
                }
            }
        }

        # 遥控器按键映射到动作模型 (使用按钮ID而非已删除的button_*字段)
        # 支持两种映射方式:
        # 1. 固定切换: button_id: 'model_type' - 直接切换到指定模型
        # 2. 循环切换: button_id: ['model1', 'model2', ...] - 在多个模型间循环切换
        self.remote_model_mapping = {
            # 按钮ID: 动作类型
            # 5: 'stop',  # R1按键 - 已在 remote_controller 中处理停止功能
            # 6: 'mode_switch',  # Select按键 - 已在 remote_controller 中处理模式切换
            9: ['standing', 'walking'],  # L3按键 - 在站立和行走模型间切换
            # 10: 'dancing',  # R3按键 - 切换到舞蹈模型
        }

        # 默认模型（向后兼容）
        self.default_single_model = "models/model_jit_0828.pt"

        # 自动加载选项
        self.auto_load_available_models = True
        self.enable_auto_switching = True  # 启用自动切换
        self.enable_manual_switching = True  # 启用手动切换


# 全局配置实例
config = RobotConfig()
