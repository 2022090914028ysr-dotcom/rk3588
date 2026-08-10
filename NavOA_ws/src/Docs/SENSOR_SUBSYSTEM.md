# Sensor Subsystem

> **Date:** 2026-07-24  
> **Packages:** `lidar_ros`, `ascam_ws`, `NavOA_ros` (IMU, ultrasonic, serial)

---

## 1. Sensor Overview

| Sensor | Interface | Baud Rate | ROS Topic(s) | Package |
|--------|-----------|-----------|--------------|---------|
| NVistar VP100/T10 2D LiDAR | Serial `/dev/ttyS4` | 230400 | `/scan` | `lidar_ros` |
| Ascamera HP60C Depth Camera | USB | N/A | `/ascamera/rgb/*`, `/ascamera/depth/*` | `ascam_ws/ascamera` |
| WIT-series IMU | Serial `/dev/ttyS6` | 230400 | `/imu` (local) | `NavOA_ros` |
| Motor controller IMU | Serial `/dev/ttyS1` | 460800 | `/imu_serial` (via bridge) | `NavOA_ros` |
| DYP-A21 Ultrasonic ×3 | UART | N/A | `/ultrasonic/front`, `/ultrasonic/left`, `/ultrasonic/right` | `NavOA_ros` |
| DYP-A21 Ultrasonic (scan mode) | UART | N/A | `/Ultrasound_scan` | `NavOA_ros` |

---

## 2. LiDAR (lidar_ros)

### 2.1 Hardware

- **Model:** NVistar VP100 / T10 series
- **Type:** 2D time-of-flight LiDAR
- **Field of View:** 360° (configurable, limited to 180° in ROS)
- **Range:** 0.1 — 15 m
- **Angular Resolution:** 1° (360 points per revolution, fixed mode)
- **Interface:** Serial (TTL) `/dev/ttyS4` at 230400 baud
- **Protocol:** 0x0208 (PROTOCOL_MODEL_NORMAL_NO_QUALITY) — each point: distance (uint16, mm) + intensity (uint8)

### 2.2 ROS Node

**Executable:** `lidar_node` (C++)

**Parameters:**
- Port: `/dev/ttyS4` (default)
- Baud rate: 230400
- Frame: `laser_frame`
- Scan angle: -180° to 180° (full circle)
- Range: 0.1 — 15.0 m

**Published Topics:**
- `/scan` (sensor_msgs/LaserScan) — at 10-15 Hz

### 2.3 SDK Structure

```
lidar_ros/
├── sdk/
│   ├── include/lidar.hpp              ← Main SDK class: Lidar
│   ├── include/lidar/lidar_protocol.hpp ← Protocol definitions
│   ├── src/lidar.cpp                  ← SDK implementation
│   ├── src/lidar/lidar_protocol.cpp   ← Protocol parsing + checksum
│   └── src/interface/serial/unix/interface_serial.cpp ← termios serial
└── ros/
    ├── src/ros_node.cpp               ← ROS node: init, collect, publish
    ├── src/ros_convert.cpp            ← SDK → LaserScan conversion
    ├── src/ros_client.cpp             ← Diagnostic client
    └── include/ros_convert.hpp
```

### 2.4 Static TF Configuration

From `lidar.launch`:
```
base_link → laser_frame
  x: 0.19 m
  y: 0.0 m
  z: 0.20 m
  yaw: -1.57 rad (90° rotation — LiDAR is mounted sideways)
```

The LiDAR is mounted forward-facing on the robot but rotated 90° so that scan angle 0° points to the robot's left. The static TF accounts for this rotation.

---

## 3. Depth Camera (ascam_ws)

### 3.1 Hardware

- **Model:** Ascamera HP60C
- **Type:** Stereo depth camera
- **RGB Resolution:** 640 × 480
- **Depth Resolution:** 640 × 480
- **Interface:** USB

### 3.2 ROS Node

**Package:** `ascamera` (C++ node `ascamera_node`)

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/ascamera/rgb/image_raw` | sensor_msgs/Image | RGB image |
| `/ascamera/rgb/camera_info` | sensor_msgs/CameraInfo | RGB calibration |
| `/ascamera/depth/image_raw` | sensor_msgs/Image | Depth image |
| `/ascamera/depth/camera_info` | sensor_msgs/CameraInfo | Depth calibration |
| `/ascamera/points` | sensor_msgs/PointCloud2 | Colored point cloud |

### 3.3 Launch Configuration

From `sensors.launch`:
```xml
<include file="$(find ascamera)/launch/hp60c.launch">
  <arg name="depth_fps" value="15"/>
  <arg name="rgb_fps" value="15"/>
  <arg name="depth_width" value="640"/>
  <arg name="depth_height" value="480"/>
  <arg name="rgb_width" value="640"/>
  <arg name="rgb_height" value="480"/>
</include>
```

### 3.4 Visual Processing Modules

| Package | Function |
|---------|----------|
| `ascam_visual` | Point cloud viz, IR transform, depth/RGB viewer, AR tag tracking, QR code, object detection, laser-to-image projection |
| `depth_to_scancamera` | Extract single row from depth image → simulated 2D laser scan |
| `astra_tracker` | OpenCV-based tracking |
| `yahboomcar_mediapipe` | MediaPipe gesture/pose recognition |
| `yahboomcar_msgs` | Custom message types |

---

## 4. IMU (Inertial Measurement Unit)

### 4.1 Dual IMU Architecture

The system has two IMU sources on separate serial ports:

```
Source 1: WIT-series IMU on /dev/ttyS6 (230400 baud)
  → imu_controller.py → imu_publisher.py → /imu
                                              │
                                              ▼
Source 2: Motor controller IMU on /dev/ttyS1 (460800 baud)
  → serial_cmd_vel_imu_bridge.py → /imu_serial
                                              │
                                              ▼
                                        imu_mux.py
                                              │
                                              ▼
                                        /imu_mux
```

### 4.2 WIT IMU Protocol

**File:** `NavOA_ros/scripts/imu_controller.py`

The WIT-series IMU sends 33-byte frame blocks via serial:

```
Byte 0:  0x55 (header)
Byte 1:  0x51 (accel) / 0x52 (gyro) / 0x53 (angle)
Byte 2-3: X-axis (int16, little-endian)
Byte 4-5: Y-axis (int16)
Byte 6-7: Z-axis (int16)
Byte 8-9: Temperature (int16)
Byte 10:  Checksum (sum bytes 0-9)

Scale factors:
- Accel: raw / 32768 × 16g (range: ±16g)
- Gyro:  raw / 32768 × 2000°/s (range: ±2000°/s)
- Angle: raw / 32768 × 180° (range: ±180°)
```

**Data rates:** ~500 Hz raw (background thread), configurable publish rate

**Calibration:**
- Roll/pitch bias removal (configurable via `--calibrate`)
- Angle safety checking (detects sensor saturation/failure)

### 4.3 IMU Multiplexer

**File:** `NavOA_ros/scripts/imu_mux.py`

```
Input:  /imu_serial (primary, from serial bridge)
        /imu         (fallback, from local WIT IMU)
Output: /imu_mux

Logic:
- If /imu_serial messages arrive within 0.2s window → publish /imu_serial
- If /imu_serial times out (>0.2s) → fall back to /imu
- Switch back to /imu_serial as soon as messages resume
```

### 4.4 IMU Usage

| Consumer | IMU Source | Purpose |
|----------|-----------|---------|
| Cartographer SLAM | — (disabled) | `use_imu_data=false` in all configs |
| Guard scripts | `/imu_mux` | Roll/pitch fall detection |
| EKF fusion | `/imu` (Z-axis gyro only) | Angular velocity for odometry fusion |
| Localization recovery | `/imu_mux` | Acceleration spike detection |

---

## 5. Ultrasonic Sensors

### 5.1 Hardware

- **Model:** DYP-A21 × 3 (front, left, right)
- **Type:** Ultrasonic range finder
- **Interface:** UART

### 5.2 Protocol

4-byte frame:
```
Byte 0: 0xFF (header)
Byte 1: Data high (6-bit)
Byte 2: Data low (8-bit)
Byte 3: Checksum (Byte1 + Byte2) & 0xFF

Distance = (Byte1 << 8 | Byte2) / 1000  (meters)

Special values:
  0xFFFE = Interference / object too close
  0xFFFD = No object detected
```

### 5.3 ROS Nodes

**a21_ultrasonic_mm.py** — Raw distance publisher:
- Publishes `/ultrasonic/distance_mm` (std_msgs/Int32)

**multi_ultrasonic_scan (in sensors.launch)** — Individual distance topics:
- `/ultrasonic/front` (Float32) — Front ultrasonic
- `/ultrasonic/left` (Float32) — Left ultrasonic
- `/ultrasonic/right` (Float32) — Right ultrasonic
- **Consumed by:** `vel_filter.py` for virtual bumper

**a21_ultrasonic_scan.py** — Synthesized LaserScan:
- Creates a 30-degree LaserScan on `/Ultrasound_scan`
- Distributes center distance across 31 scan points with angle-dependent attenuation
- Includes moving average filtering and Gaussian noise model
- Simulated data fallback

---

## 6. rf2o Laser Odometry

### 6.1 Algorithm

**Package:** `rf2o_laser_odometry` (C++, MAPIR Lab, ICRA 2016)

Range-flow based 2D laser odometry. Estimates planar motion (x, y, yaw) from consecutive laser scans without requiring wheel odometry.

**Key characteristics:**
- Computation: ~0.9 ms per frame (single core)
- Frequency: 20 Hz
- Method: Multi-scale image pyramid, range derivatives, weighted least squares

### 6.2 ROS Node

**Parameters:**
- `laser_scan_topic`: `/scan`
- `odom_topic`: `/odom_rf2o`
- `publish_tf`: `true`
- `base_frame_id`: `base_link`
- `freq`: 20.0 Hz

### 6.3 EKF Fusion (robot_localization)

**File:** `rf2o_laser_odometry/launch/ekf_fusion.launch`

Fuses rf2o laser odometry with IMU Z-axis angular velocity:

```
Inputs:
  /odom_rf2o  (Odometry, 20 Hz) — X, Y, yaw position + velocity
  /imu        (Imu, ~100 Hz)    — angular_velocity.z only

Fusion:
  robot_localization ekf_localization_node
  2D mode (X, Y, yaw)
  predict_to_current_time = true

Output:
  /odom (Odometry, 30 Hz)
```

**EKF Parameters:**
- RF2O X/Y covariance: 0.01
- RF2O yaw covariance: 0.02
- Process noise X/Y: 0.05
- Process noise yaw: 0.08
- Process noise Wz: 0.001

---

## 7. Serial Communication Architecture

### 7.1 Serial Port Summary

| Port | Baud | Connected To | Bidirectional? | Data Rate |
|------|------|-------------|----------------|-----------|
| `/dev/ttyS1` | 460800 | Motor controller | ✅ Yes | cmd_vel 30 Hz out, IMU ~50 Hz in |
| `/dev/ttyS4` | 230400 | LiDAR | ❌ No (LiDAR TX only) | 360 points × 10-15 Hz |
| `/dev/ttyS6` | 230400 | WIT IMU | ❌ No (IMU TX only) | ~500 Hz raw frames |

### 7.2 Serial Bridge Data Flow

```
Motor Controller ←→ Serial (ttyS1) ←→ serial_cmd_vel_imu_bridge.py ←→ ROS
     │                                                                    │
     │  TX: CSV lines                                                     │
     │    "IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw\r\n"                     │
     │    "BUTTON,id,type\r\n"                                            │
     │    JSONL: {"protocol":"xrobot-nav-jsonl-v1",...}                   │
     │                                                                    │
     │  RX: CSV lines                                                     │
     │    "CMD_VEL,vx,vy,w\r\n"                                           │
     └────────────────────────────────────────────────────────────────────┘
```

### 7.3 Thread Model

**serial_cmd_vel_imu_bridge.py:**
- **Main thread:** Initialization + spinning
- **Reader thread:** Continuously reads serial port, parses CSV/JSONL, publishes ROS messages
- **Writer thread:** Subscribes to `/cmd_vel`, throttles to 30 Hz, writes to serial with thread-safe queue
