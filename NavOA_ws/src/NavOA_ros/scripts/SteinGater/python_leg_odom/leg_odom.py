"""
CJW_Leg_Body_KF Python Implementation
Pure leg robot body position and velocity estimation using Kalman Filter
Based on MIT's IMU fusion approach for legged robot body position/velocity estimation
"""

import numpy as np
from typing import Optional, Tuple


EKF_IL = 0
EKF_EIL = 1

FOOT_SLIP_MIN = 1.0
SWING_NOISE = 1000.0
FOOT_PLANE = 0


class CJW_Leg_Body_KF:
    """
    Legged Robot Body Position/Velocity Kalman Filter
    
    State vector: [body_p(3), body_v(3), foot_p(3*branchn)]
    Total dimension: 3*branchn + 6
    
    Filter methods:
        EKF_IL (0): MIT IMU + leg odometry fusion
        EKF_EIL (1): MIT + external position observation fusion
    """
    
    def __init__(self, branchn: int = 2):
        self.branchn = branchn
        
        self.KF_Type = EKF_IL
        self.T_predict = 0.0
        
        self.body_p_id = 0
        self.body_v_id = 3
        self.foot_p_id = 6
        
        self.foot_dp = np.zeros((3, branchn))
        self.foot_dv = np.zeros((3, branchn))
        self.foot_contact = np.zeros(branchn)
        self.foot_p0 = np.zeros((3, branchn))
        
        self.imu_acc = np.zeros(3)
        self.ex_position = np.zeros(3)
        self.ex_velocity = np.zeros(3)
        self.ex_acceleration = np.zeros(3)
        
        self.XN = 0
        self.UN = 0
        self.ZN = 0
        self.ParaN = 0
        
        self.Para = None
        self._x = None
        self._u = None
        self._z = None
        self._A = None
        self._B = None
        self._C = None
        self._P = None
        self._Q = None
        self._R = None
        self._I = None
    
    def kf_resize(self, n_x: int, n_u: int, n_z: int):
        """Resize Kalman filter matrices"""
        self.XN = n_x
        self.UN = n_u
        self.ZN = n_z
        
        self._x = np.zeros((n_x, 1))
        self._u = np.zeros((n_u, 1))
        self._z = np.zeros((n_z, 1))
        
        self._A = np.eye(n_x)
        self._B = np.zeros((n_x, n_u))
        self._C = np.zeros((n_z, n_x))
        
        self._P = np.eye(n_x)
        self._Q = np.eye(n_x)
        self._R = np.eye(n_z)
        self._I = np.eye(n_x)
    
    def kf_once(self):
        """One step Kalman filter calculation"""
        self._x = self._A @ self._x + self._B @ self._u
        
        At = self._A.T
        Pm = self._A @ self._P @ At + self._Q
        
        Ct = self._C.T
        zModel = self._C @ self._x
        ey = self._z - zModel
        
        S = self._C @ Pm @ Ct + self._R
        
        S_ey = np.linalg.solve(S, ey)
        self._x += Pm @ Ct @ S_ey
        
        S_C = np.linalg.solve(S, self._C)
        self._P = (self._I - Pm @ Ct @ S_C) @ Pm
        
        self._P = (self._P + self._P.T) / 2.0
    
    def set_state(self, input_data: np.ndarray):
        """Set state estimate value"""
        self._x = input_data.reshape(-1, 1)
    
    def set_state_from_list(self, input_data: list):
        """Set state estimate value from list"""
        self._x = np.array(input_data).reshape(-1, 1)
    
    def get_state(self) -> np.ndarray:
        """Get state estimate value"""
        return self._x.flatten()
    
    def set_input(self, input_data: np.ndarray):
        """Set state input value"""
        self._u = input_data.reshape(-1, 1)
    
    def set_input_from_list(self, input_data: list):
        """Set state input value from list"""
        self._u = np.array(input_data).reshape(-1, 1)
    
    def get_input(self) -> np.ndarray:
        """Get state input value"""
        return self._u.flatten()
    
    def set_obs(self, input_data: np.ndarray):
        """Set observation data"""
        self._z = input_data.reshape(-1, 1)
    
    def set_obs_from_list(self, input_data: list):
        """Set observation data from list"""
        self._z = np.array(input_data).reshape(-1, 1)
    
    def get_obs(self) -> np.ndarray:
        """Get observation data"""
        return self._z.flatten()
    
    def set_para(self, input_data: np.ndarray):
        """Set noise parameters"""
        self.Para = input_data.reshape(-1, 1)
    
    def set_para_from_list(self, input_data: list):
        """Set noise parameters from list"""
        self.Para = np.array(input_data).reshape(-1, 1)
    
    def get_para(self) -> np.ndarray:
        """Get noise parameters"""
        return self.Para.flatten()
    
    def set_body_p(self, bodyp: np.ndarray):
        """Set current body position"""
        self._x[self.body_p_id:self.body_p_id+3, 0] = bodyp
    
    def set_body_p_from_list(self, bodyp: list):
        """Set current body position from list"""
        self._x[self.body_p_id:self.body_p_id+3, 0] = np.array(bodyp)
    
    def set_body_v(self, bodyv: np.ndarray):
        """Set current body velocity"""
        self._x[self.body_v_id:self.body_v_id+3, 0] = bodyv
    
    def set_body_v_from_list(self, bodyv: list):
        """Set current body velocity from list"""
        self._x[self.body_v_id:self.body_v_id+3, 0] = np.array(bodyv)
    
    def get_body_p(self) -> np.ndarray:
        """Get estimated body position"""
        return self._x[self.body_p_id:self.body_p_id+3, 0]
    
    def get_body_v(self) -> np.ndarray:
        """Get estimated body velocity"""
        return self._x[self.body_v_id:self.body_v_id+3, 0]
    
    def set_leg_foot_p(self, footp: np.ndarray):
        """Set current leg foot positions"""
        for i in range(self.branchn):
            self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0] = footp[:, i]
    
    def set_leg_foot_p_from_list(self, footp: list):
        """Set current leg foot positions from list"""
        footp_arr = np.array(footp).reshape(3, self.branchn)
        for i in range(self.branchn):
            self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0] = footp_arr[:, i]
    
    def get_leg_foot_p(self) -> np.ndarray:
        """Get current leg foot positions"""
        legp = np.zeros((3, self.branchn))
        for i in range(self.branchn):
            legp[:, i] = self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0]
        return legp
    
    def set_body_leg_state(self, bodyp: np.ndarray, bodyv: np.ndarray, footp: np.ndarray):
        """Set body position, velocity and leg foot positions"""
        self._x[self.body_p_id:self.body_p_id+3, 0] = bodyp
        self._x[self.body_v_id:self.body_v_id+3, 0] = bodyv
        for i in range(self.branchn):
            self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0] = footp[:, i]
    
    def set_body_leg_state_from_list(self, bodyp: list, bodyv: list, footp: list):
        """Set body position, velocity and leg foot positions from lists"""
        self._x[self.body_p_id:self.body_p_id+3, 0] = np.array(bodyp)
        self._x[self.body_v_id:self.body_v_id+3, 0] = np.array(bodyv)
        footp_arr = np.array(footp).reshape(3, self.branchn)
        for i in range(self.branchn):
            self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0] = footp_arr[:, i]
    
    def get_body_leg_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get body position, velocity and leg foot positions"""
        bodyp = self._x[self.body_p_id:self.body_p_id+3, 0]
        bodyv = self._x[self.body_v_id:self.body_v_id+3, 0]
        footp = self.get_leg_foot_p()
        return bodyp, bodyv, footp
    
    def set_leg_obs(self, footdp: np.ndarray, footdv: np.ndarray, contact: np.ndarray):
        """Set leg odometry observations"""
        self.foot_dp = footdp.copy()
        self.foot_dv = footdv.copy()
        
        for i in range(self.branchn):
            flag = 0
            if contact[i] >= 1:
                the_dp = self.foot_p0[:, i] - self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0].flatten()
                if self.foot_contact[i] < 0:
                    flag = 1
                elif np.linalg.norm(the_dp) > FOOT_SLIP_MIN:
                    flag = 1
            
            if flag:
                self.foot_p0[:, i] = self._x[self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1), 0].flatten()
            
            self.foot_contact[i] = contact[i]
    
    def set_leg_obs_from_list(self, footdp: list, footdv: list, contact: list):
        """Set leg odometry observations from lists"""
        self.set_leg_obs(
            np.array(footdp).reshape(3, self.branchn),
            np.array(footdv).reshape(3, self.branchn),
            np.array(contact)
        )
    
    def set_imu_acc(self, acc: np.ndarray):
        """Set IMU acceleration data"""
        self.imu_acc = acc.copy()
    
    def set_imu_acc_from_list(self, acc: list):
        """Set IMU acceleration data from list"""
        self.imu_acc = np.array(acc)
    
    def set_ext_obs(self, p: np.ndarray, v: np.ndarray, a: np.ndarray = None):
        """Set external observation data"""
        self.ex_position = p.copy()
        self.ex_velocity = v.copy()
        if a is not None:
            self.ex_acceleration = a.copy()
    
    def set_ext_obs_from_list(self, p: list, v: list, a: list = None):
        """Set external observation data from lists"""
        self.set_ext_obs(np.array(p), np.array(v), np.array(a) if a is not None else None)
    
    def set_type(self, kf_type: int):
        """Set Kalman filter type"""
        self.KF_Type = kf_type
        self.T_predict = 0.0
        
        self.XN = 3 * self.branchn + 6
        self.UN = 3
        
        if self.KF_Type == EKF_EIL:
            self.ZN = 7 * self.branchn + 6
        else:
            self.ZN = 7 * self.branchn
        
        self.ParaN = 8
        
        self.kf_resize(self.XN, self.UN, self.ZN)
        self.Para = np.zeros((self.ParaN, 1))
        
        self.body_p_id = 0
        self.body_v_id = 3
        self.foot_p_id = 6
    
    def initial(self, dt: float):
        """Initialize Kalman filter"""
        self.T_predict = dt
        
        self._A = np.eye(self.XN)
        self._A[0:3, 3:6] = dt * np.eye(3)
        
        self._B = np.zeros((self.XN, self.UN))
        self._B[0:3, 0:3] = 0.5 * dt * dt * np.eye(3)
        self._B[3:6, 0:3] = dt * np.eye(3)
        
        self._C = np.zeros((self.ZN, self.XN))
        
        for i in range(self.branchn):
            self._C[3*i:3*(i+1), 0:3] = np.eye(3)
            self._C[3*i:3*(i+1), self.foot_p_id + 3*i:self.foot_p_id + 3*(i+1)] = -np.eye(3)
            self._C[3*(self.branchn + i):3*(self.branchn + i + 1), 3:6] = np.eye(3)
            self._C[6*self.branchn + i, 8 + 3*i] = 1
        
        if self.KF_Type == EKF_EIL:
            self._C[7*self.branchn:7*self.branchn+3, 0:3] = np.eye(3)
            self._C[7*self.branchn+3:7*self.branchn+6, 3:6] = np.eye(3)
        
        self._P = 100 * np.eye(self.XN)
        self._Q = np.eye(self.XN)
        self._R = np.eye(self.ZN)
        
        self.foot_p0 = np.zeros((3, self.branchn))
    
    def estimate_once(self):
        """Run one step of Kalman filter estimation"""
        Kpb = self.Para[0, 0]
        Kvb = self.Para[1, 0]
        Kpf = self.Para[2, 0]
        Kpbf = self.Para[3, 0]
        Kpfv = self.Para[4, 0]
        Kfz = self.Para[5, 0]
        Kpex = self.Para[6, 0]
        Kvex = self.Para[7, 0]
        
        self._Q = np.eye(self.XN)
        self._Q[0:3, 0:3] = (self.T_predict / 20.0) * Kpb * np.eye(3)
        self._Q[3:6, 3:6] = (self.T_predict * 9.8 / 20.0) * Kvb * np.eye(3)
        self._Q[6:6+3*self.branchn, 6:6+3*self.branchn] = self.T_predict * Kpf * np.eye(3 * self.branchn)
        
        self._R = np.eye(self.ZN)
        self._R[0:3*self.branchn, 0:3*self.branchn] = Kpbf * np.eye(3 * self.branchn)
        self._R[3*self.branchn:6*self.branchn, 3*self.branchn:6*self.branchn] = Kpfv * np.eye(3 * self.branchn)
        self._R[6*self.branchn:7*self.branchn, 6*self.branchn:7*self.branchn] = Kfz * np.eye(self.branchn)
        
        if self.KF_Type == EKF_EIL:
            self._R[7*self.branchn:7*self.branchn+3, 7*self.branchn:7*self.branchn+3] = Kpex * np.eye(3)
            self._R[7*self.branchn+3:7*self.branchn+6, 7*self.branchn+3:7*self.branchn+6] = Kvex * np.eye(3)
        
        p0 = self._x[0:3, 0]
        v0 = self._x[3:6, 0]
        
        self._u = self.imu_acc.reshape(-1, 1)
        
        if self.KF_Type == EKF_EIL:
            self._z[7*self.branchn:7*self.branchn+3, 0] = self.ex_position
            self._z[7*self.branchn+3:7*self.branchn+6, 0] = self.ex_velocity
        
        for i in range(self.branchn):
            trust = min(self.foot_contact[i], 1.0)
            thek0 = 1 + SWING_NOISE * (1 - trust)
            
            i1 = 3 * i
            qindex = 6 + i1
            rindex1 = i1
            rindex2 = 3 * self.branchn + i1
            rindex3 = 6 * self.branchn + i
            
            self._Q[qindex:qindex+3, qindex:qindex+3] = thek0 * self._Q[qindex:qindex+3, qindex:qindex+3]
            
            self._R[rindex1:rindex1+3, rindex1:rindex1+3] = self._R[rindex1:rindex1+3, rindex1:rindex1+3]
            self._R[rindex2:rindex2+3, rindex2:rindex2+3] = thek0 * self._R[rindex2:rindex2+3, rindex2:rindex2+3]
            self._R[rindex3, rindex3] = thek0 * self._R[rindex3, rindex3]
            
            self._z[rindex1:rindex1+3, 0] = -self.foot_dp[:, i]
            
            v_term = (1.0 - trust) * v0 + trust * (-self.foot_dv[:, i])
            self._z[rindex2:rindex2+3, 0] = v_term
            
            z_height = (1.0 - trust) * (p0[2] + self.foot_dp[2, i]) + FOOT_PLANE * trust * self.foot_p0[2, i]
            self._z[rindex3, 0] = z_height
        
        self.kf_once()
        
        if np.linalg.det(self._P[0:2, 0:2]) > 0.000001:
            self._P[0:2, 2:2+4+3*self.branchn] = 0
            self._P[2:2+4+3*self.branchn, 0:2] = 0
            self._P[0:2, 0:2] = self._P[0:2, 0:2] / 10


class LegOdometry:
    """
    Leg Odometry class for bipedal robot
    Wrapper around CJW_Leg_Body_KF with convenient interface
    """
    
    def __init__(self, num_legs: int = 2, dt: float = 0.005, filter_type: int = EKF_IL):
        """
        Initialize Leg Odometry
        
        Args:
            num_legs: Number of legs (default: 2 for bipedal robot)
            dt: Sampling period in seconds (default: 0.005 = 5ms)
            filter_type: Filter type (EKF_IL or EKF_EIL)
        """
        self.num_legs = num_legs
        self.dt = dt
        self.filter_type = filter_type
        
        self.kf = CJW_Leg_Body_KF(branchn=num_legs)
        self.kf.set_type(filter_type)
        self.kf.initial(dt)
        
        self._initialized = False
    
    def set_noise_parameters(self, 
                              kpb: float = 1.0, 
                              kvb: float = 1.0, 
                              kpf: float = 1.0,
                              kpbf: float = 1.0,
                              kpfv: float = 1.0,
                              kfz: float = 1.0,
                              kpex: float = 1.0,
                              kvex: float = 1.0):
        """
        Set Kalman filter noise parameters
        
        Args:
            kpb: Prediction position noise
            kvb: Prediction velocity noise
            kpf: Prediction foot position noise
            kpbf: Observation leg position noise
            kpfv: Observation leg velocity noise
            kfz: Observation leg height noise
            kpex: Observation external position noise
            kvex: Observation external velocity noise
        """
        params = np.array([kpb, kvb, kpf, kpbf, kpfv, kfz, kpex, kvex], dtype=np.float64)
        self.kf.set_para(params)
    
    def initialize_state(self, 
                         body_position: np.ndarray, 
                         body_velocity: np.ndarray,
                         foot_positions: np.ndarray):
        """
        Initialize the filter state
        
        Args:
            body_position: Initial body position [x, y, z]
            body_velocity: Initial body velocity [vx, vy, vz]
            foot_positions: Initial foot positions (3 x num_legs array)
        """
        self.kf.set_body_leg_state(body_position, body_velocity, foot_positions)
        self._initialized = True
    
    def update(self,
               imu_acceleration: np.ndarray,
               foot_delta_positions: np.ndarray,
               foot_velocities: np.ndarray,
               contact_flags: np.ndarray,
               external_position: Optional[np.ndarray] = None,
               external_velocity: Optional[np.ndarray] = None):
        """
        Update leg odometry with new sensor data
        
        Args:
            imu_acceleration: IMU acceleration measurement [ax, ay, az]
            foot_delta_positions: Foot delta positions from leg kinematics (3 x num_legs)
            foot_velocities: Foot velocities (3 x num_legs)
            contact_flags: Contact status for each leg (num_legs,) - 1 = contact, 0 = swing
            external_position: Optional external position observation
            external_velocity: Optional external velocity observation
        """
        self.kf.set_imu_acc(imu_acceleration)
        self.kf.set_leg_obs(foot_delta_positions, foot_velocities, contact_flags)
        
        if external_position is not None and external_velocity is not None:
            self.kf.set_ext_obs(external_position, external_velocity)
        
        self.kf.estimate_once()
    
    def get_estimate(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current state estimate
        
        Returns:
            Tuple of (body_position, body_velocity, foot_positions)
        """
        return self.kf.get_body_leg_state()
    
    def reset(self, 
              body_position: np.ndarray, 
              body_velocity: np.ndarray,
              foot_positions: np.ndarray):
        """
        Reset the filter with new initial state
        
        Args:
            body_position: Body position [x, y, z]
            body_velocity: Body velocity [vx, vy, vz]
            foot_positions: Foot positions (3 x num_legs array)
        """
        self.kf.set_type(self.filter_type)
        self.kf.initial(self.dt)
        self.initialize_state(body_position, body_velocity, foot_positions)


def create_default_odometry(num_legs: int = 2, dt: float = 0.005) -> LegOdometry:
    """
    Create a leg odometry instance with default parameters
    
    Args:
        num_legs: Number of legs
        dt: Sampling period
    
    Returns:
        Configured LegOdometry instance
    """
    odom = LegOdometry(num_legs=num_legs, dt=dt)
    
    odom.set_noise_parameters(
        kpb=0.1,
        kvb=0.1,
        kpf=0.1,
        kpbf=0.1,
        kpfv=0.1,
        kfz=0.1,
        kpex=0.1,
        kvex=0.1
    )
    
    return odom