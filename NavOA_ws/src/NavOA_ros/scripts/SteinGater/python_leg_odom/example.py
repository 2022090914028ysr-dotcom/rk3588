"""
Example usage of Leg Odometry for Bipedal Robot
Demonstrates how to use the CJW_Leg_Body_KF implementation
"""

import numpy as np
import sys
sys.path.insert(0, '.')

from leg_odom import LegOdometry, create_default_odometry, EKF_IL, EKF_EIL


def example_basic_usage():
    """Basic usage example"""
    print("=" * 60)
    print("Example 1: Basic Usage")
    print("=" * 60)
    
    odom = create_default_odometry(num_legs=2, dt=0.005)
    
    body_p = np.array([0.0, 0.0, 0.5])
    body_v = np.array([0.0, 0.0, 0.0])
    foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.initialize_state(body_p, body_v, foot_p)
    
    imu_acc = np.array([0.0, 0.0, -9.8])
    foot_delta = np.array([
        [0.01, -0.01],
        [0.0,   0.0],
        [0.0,   0.0]
    ])
    foot_vel = np.array([
        [0.1, -0.1],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    contact = np.array([1.0, 1.0])
    
    for i in range(10):
        odom.update(imu_acc, foot_delta, foot_vel, contact)
        bp, bv, fp = odom.get_estimate()
        print(f"Step {i+1}: body_pos={bp}, body_vel={bv}")
    
    print()


def example_with_external_obs():
    """Example with external position observation (EKF_EIL)"""
    print("=" * 60)
    print("Example 2: With External Observations (EKF_EIL)")
    print("=" * 60)
    
    odom = LegOdometry(num_legs=2, dt=0.005, filter_type=EKF_EIL)
    odom.set_noise_parameters(
        kpb=0.1, kvb=0.1, kpf=0.1,
        kpbf=0.1, kpfv=0.1, kfz=0.1,
        kpex=0.05, kvex=0.05
    )
    
    body_p = np.array([0.0, 0.0, 0.5])
    body_v = np.array([0.0, 0.0, 0.0])
    foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.initialize_state(body_p, body_v, foot_p)
    
    imu_acc = np.array([0.1, 0.0, -9.8])
    foot_delta = np.array([
        [0.02, -0.02],
        [0.0,   0.0],
        [0.0,   0.0]
    ])
    foot_vel = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    contact = np.array([1.0, 1.0])
    
    ext_pos = np.array([0.1, 0.0, 0.5])
    ext_vel = np.array([0.1, 0.0, 0.0])
    
    for i in range(5):
        odom.update(imu_acc, foot_delta, foot_vel, contact, ext_pos, ext_vel)
        bp, bv, fp = odom.get_estimate()
        print(f"Step {i+1}:")
        print(f"  body_pos: [{bp[0]:.4f}, {bp[1]:.4f}, {bp[2]:.4f}]")
        print(f"  body_vel: [{bv[0]:.4f}, {bv[1]:.4f}, {bv[2]:.4f}]")
    
    print()


def example_swing_leg():
    """Example with swing leg detection"""
    print("=" * 60)
    print("Example 3: Swing Leg Detection")
    print("=" * 60)
    
    odom = create_default_odometry(num_legs=2, dt=0.005)
    
    body_p = np.array([0.0, 0.0, 0.5])
    body_v = np.array([0.0, 0.0, 0.0])
    foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.initialize_state(body_p, body_v, foot_p)
    
    imu_acc = np.array([0.0, 0.0, -9.8])
    
    contact_stance = np.array([1.0, 1.0])
    contact_swing = np.array([1.0, 0.0])
    
    foot_delta_stance = np.array([
        [0.01, -0.01],
        [0.0,   0.0],
        [0.0,   0.0]
    ])
    
    foot_delta_swing = np.array([
        [0.01, -0.05],
        [0.0,   0.0],
        [0.0,   0.0]
    ])
    
    foot_vel = np.zeros((3, 2))
    
    print("Stance phase (both legs in contact):")
    for i in range(3):
        odom.update(imu_acc, foot_delta_stance, foot_vel, contact_stance)
        bp, bv, fp = odom.get_estimate()
        print(f"  Step {i+1}: pos_x={bp[0]:.4f}, vel_x={bv[0]:.4f}")
    
    print("\nSwing phase (one leg in contact):")
    for i in range(3):
        odom.update(imu_acc, foot_delta_swing, foot_vel, contact_swing)
        bp, bv, fp = odom.get_estimate()
        print(f"  Step {i+1}: pos_x={bp[0]:.4f}, vel_x={bv[0]:.4f}")
    
    print()


def example_custom_parameters():
    """Example with custom noise parameters"""
    print("=" * 60)
    print("Example 4: Custom Noise Parameters")
    print("=" * 60)
    
    odom = LegOdometry(num_legs=2, dt=0.01)
    
    odom.set_noise_parameters(
        kpb=0.5,   
        kvb=0.8,   
        kpf=0.3,   
        kpbf=0.2,  
        kpfv=0.4,  
        kfz=0.1,   
        kpex=0.1,  
        kvex=0.2   
    )
    
    body_p = np.array([0.0, 0.0, 0.5])
    body_v = np.array([0.0, 0.0, 0.0])
    foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.initialize_state(body_p, body_v, foot_p)
    
    imu_acc = np.array([0.0, 0.0, -9.8])
    foot_delta = np.array([
        [0.0,  0.0],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    foot_vel = np.zeros((3, 2))
    contact = np.array([1.0, 1.0])
    
    for i in range(5):
        odom.update(imu_acc, foot_delta, foot_vel, contact)
        bp, bv, fp = odom.get_estimate()
        print(f"Step {i+1}: body_z={bp[2]:.4f}")
    
    print()


def example_reset():
    """Example of resetting the filter"""
    print("=" * 60)
    print("Example 5: Reset Filter")
    print("=" * 60)
    
    odom = create_default_odometry(num_legs=2)
    
    body_p = np.array([0.0, 0.0, 0.5])
    body_v = np.array([0.0, 0.0, 0.0])
    foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.initialize_state(body_p, body_v, foot_p)
    
    imu_acc = np.array([0.0, 0.0, -9.8])
    foot_delta = np.array([
        [0.01, -0.01],
        [0.0,   0.0],
        [0.0,   0.0]
    ])
    foot_vel = np.zeros((3, 2))
    contact = np.array([1.0, 1.0])
    
    for i in range(3):
        odom.update(imu_acc, foot_delta, foot_vel, contact)
    
    bp, bv, fp = odom.get_estimate()
    print(f"Before reset: pos={bp}")
    
    new_body_p = np.array([5.0, 2.0, 0.5])
    new_body_v = np.array([0.0, 0.0, 0.0])
    new_foot_p = np.array([
        [0.2, -0.2],
        [0.0,  0.0],
        [0.0,  0.0]
    ])
    
    odom.reset(new_body_p, new_body_v, new_foot_p)
    
    bp, bv, fp = odom.get_estimate()
    print(f"After reset:  pos={bp}")
    
    print()


if __name__ == "__main__":
    example_basic_usage()
    example_with_external_obs()
    example_swing_leg()
    example_custom_parameters()
    example_reset()
    
    print("=" * 60)
    print("All examples completed!")
    print("=" * 60)