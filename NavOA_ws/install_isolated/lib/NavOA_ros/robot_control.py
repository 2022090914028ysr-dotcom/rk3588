import motor_control 
import time
import threading

class RobotController:
    _instance = None
    _initialized = False
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(RobotController, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if RobotController._initialized:
            return
        with RobotController._lock:
            if RobotController._initialized:
                return

            # 实际初始化逻辑
            self.motor_controller = motor_control.MotorController()
            self.max_linear_velocity = 2.0
            self.max_angle_velocity = 1.0
            self.linear_velocity = 0.0
            self.angle_velocity = 0.0
            self.wheel_base = 0.364
            self.wheel_radius = 0.0695

            self.velocity_lock = threading.Lock()

            self.last_command_time = time.time()
            self.timeout_duration = 0.5
            self.timeout_active = True
            self.timeout_thread = None
            self.timeout_running = False

            RobotController._initialized = True

    @classmethod
    def get_instance(cls):
        return cls()

    def robot_init(self):
        # 注意：允许多次调用，但只真正初始化一次
        if hasattr(self, '_motor_initialized') and self._motor_initialized:
            return True

        try:
            if not self.motor_controller.initialize():
                print("电机初始化失败")
                return False
            print("电机初始化成功")

            self.timeout_running = True
            self.timeout_thread = threading.Thread(target=self.timeout_monitor, daemon=True)
            self.timeout_thread.start()

            self._motor_initialized = True
            return True

        except Exception as e:
            print(f"初始化机器人失败: {e}")
            return False

    def robot_close(self):
        if not getattr(self, '_motor_initialized', False):
            return

        try:
            self.timeout_running = False
            if self.timeout_thread and self.timeout_thread.is_alive():
                self.timeout_thread.join(timeout=1.0)

            self.update_robot_velocity(0, 0)

            if not self.motor_controller.close():
                raise Exception("电机关闭失败")
            print("电机关闭成功")

            self._motor_initialized = False

        except Exception as e:
            print(f"关闭机器人失败: {e}")

    def timeout_monitor(self):
        while self.timeout_running:
            if time.time() - self.last_command_time > self.timeout_duration and not self.timeout_active:
                print("Command timeout detected, sending zero velocities")
                self.timeout_active = True
                self.update_robot_velocity(0, 0)
            time.sleep(0.1)

    def update_robot_velocity(self, linear_velocity, angle_velocity):
        try:
            self.last_command_time = time.time()
            self.timeout_active = False

            if not isinstance(linear_velocity, (int, float)) or not isinstance(angle_velocity, (int, float)):
                raise TypeError("线速度和角速度必须是数值类型")

            if self.wheel_base <= 0 or self.wheel_radius <= 0:
                raise ValueError("机器人参数未正确设置")

            linear_velocity = max(min(linear_velocity, self.max_linear_velocity), -self.max_linear_velocity)
            angle_velocity = max(min(angle_velocity, self.max_angle_velocity), -self.max_angle_velocity)
            #print(f"RobotController: 设置速度 linear={linear_velocity:.3f}, angular={angle_velocity:.3f}")
            
            self.linear_velocity = linear_velocity
            self.angle_velocity = angle_velocity
           
            w_left, w_right = self.differential_drive(linear_velocity, angle_velocity)
            rpm_left = round(self.rads_to_rpm(w_left))
            rpm_right = round(self.rads_to_rpm(w_right))

            return self.motor_controller.set_sync_velocity(rpm_left, rpm_right)

        except Exception as e:
            print(f"更新速度失败: {e}")
            return False

    def get_velocity(self):
        with self.velocity_lock:
            
            return self.linear_velocity, self.angle_velocity

    def set_robot_parameters(self, wheel_base, wheel_radius):
        if wheel_base > 0 and wheel_radius > 0:
            self.wheel_base = wheel_base
            self.wheel_radius = wheel_radius
            return True
        else:
            print("错误: 轮距和轮半径必须为正数")
            return False

    def differential_drive(self, linear_velocity, angle_velocity):
        v_left = linear_velocity + (angle_velocity * self.wheel_base) / 2
        v_right = linear_velocity - (angle_velocity * self.wheel_base) / 2
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius
        return w_left, w_right

    def rads_to_rpm(self, rads):
        return rads * 60 / (2 * 3.14159265359)

    def rpm_to_rads(self, rpm):
        return rpm * (2 * 3.14159265359) / 60

    def get_actual_linear_velocity(self):
        """从电机转速计算实际线速度"""
        try:
            # 假设 motor_controller 有 get_motor_speeds() 返回 (rpm_left, rpm_right)
            rpm_left, rpm_right = self.motor_controller.get_motor_speeds()
            rads_left = self.rpm_to_rads(rpm_left)
            rads_right = self.rpm_to_rads(-rpm_right)  # 右轮编码器方向相反，取反
            v_left = rads_left * self.wheel_radius
            v_right = rads_right * self.wheel_radius
            avg_v = (v_left + v_right) / 2.0
            #print(f"实际速度计算: 左轮 {rpm_left:.2f} rpm ({v_left:.3f} m/s), 右轮 {rpm_right:.2f} rpm ({v_right:.3f} m/s), 平均 {avg_v:.3f} m/s")
            return avg_v
        except Exception as e:
            print(f"获取实际线速度失败: {e}")
            return 0.0