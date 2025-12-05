# 🤖 KATECH Robot Workspace - AI Codebase Guide

> **Purpose**: This document is designed to help AI assistants quickly understand the entire codebase structure, architecture, and key components of the KATECH robot workspace.

---

## 📋 Quick Reference

| Item | Value |
|------|-------|
| **Project Name** | Mobile-Cobot Integrated Control System |
| **Organization** | KATECH (Korea Automotive Technology Institute) |
| **ROS Version** | ROS 1 Noetic (Docker container) |
| **Host OS** | Ubuntu 24.04 LTS |
| **Python Version** | 3.8+ |
| **Primary Language** | Python (with C++ for Doosan robot drivers) |
| **Communication** | ROS Topics, ROS Services, WebSocket (WooshRobot SDK) |

---

## 🎯 Project Overview

### System Purpose
This project integrates a **mobile robot (TR200)** and a **collaborative robot (Doosan A0912)** under a central control system. The robots work sequentially in a coordinated workflow:

1. Mobile robot moves to target position
2. Collaborative robot performs manipulation task
3. Mobile robot returns to original position
4. Collaborative robot returns to home position

### Hardware Components

| Robot | Model | IP Address | Communication |
|-------|-------|------------|---------------|
| Mobile Robot | TR200 | 169.254.128.2 | WebSocket (Port 5480) |
| Collaborative Robot | Doosan A0912 | 192.168.137.100 | ROS Services |
| 3D Camera | Zivid | - | Zivid SDK |

---

## 📁 Directory Structure

```
robot_ws/
├── src/                              # ROS packages source
│   ├── mobile_robot_server/          # Mobile robot ROS service server
│   ├── mobile_lidar_control/         # LiDAR-based motion control
│   ├── testbed_operation/            # Integration client & QR vision
│   ├── woosh_robot_py/               # WooshRobot SDK (TR200 mobile robot)
│   ├── doosan-robot/                 # Doosan collaborative robot packages
│   ├── serial/                       # Serial communication library
│   └── beginner_tutorials/           # ROS tutorials
├── docs/                             # Documentation
│   ├── ENVIRONMENT_SETUP.md          # Docker/ROS environment setup
│   ├── INTEGRATION_GUIDE.md          # System integration guide
│   └── QR/                           # QR code vision documentation
├── Project.md                        # Project architecture document
├── README.md                         # Main README
└── Dockerfile                        # Docker configuration
```

---

## 📦 Package Details

### 1. `mobile_robot_server` - Mobile Robot Service Server

**Purpose**: Provides ROS service interface for controlling the TR200 mobile robot.

**Key Files**:
| File | Description |
|------|-------------|
| `scripts/ldj_mobile_posiotion_server_twist.py` | Main service server with Twist velocity control |
| `srv/MobilePositionTwist.srv` | Service definition for distance-based movement |

**Service Definition** (`MobilePositionTwist.srv`):
```
# Request
float32 distance    # Distance to move (m), negative = backward
---
# Response
bool success
string message
```

**Key Class**: `SmoothTwistController`
- Uses trapezoidal velocity profile for smooth motion
- Asyncio-based communication with WooshRobot SDK
- Parameters: `max_speed=0.12 m/s`, `accel=0.25 m/s²`, `decel=0.50 m/s²`

---

### 2. `mobile_lidar_control` - LiDAR Motion Control

**Purpose**: Provides LiDAR-based position correction and precise motion control.

**Key Files**:
| File | Description |
|------|-------------|
| `scripts/twist_motion_controller.py` | Twist-based motion controller with trapezoidal profile |
| `scripts/lidar_subscriber.py` | LiDAR data subscriber (WooshRobot → ROS LaserScan) |
| `config/params.yaml` | Configuration parameters |
| `srv/MoveDistance.srv` | Service definition |

**Service Definition** (`MoveDistance.srv`):
```
# Request
float32 distance    # Distance to move (m), positive=forward, negative=backward
---
# Response
bool success
string message
```

**Key Class**: `TwistMotionController`
- Motion states: `IDLE`, `ACCELERATING`, `CRUISING`, `DECELERATING`, `STOPPING`, `EMERGENCY_STOP`
- Emergency stop service: `/mobile_lidar_control/emergency_stop`

**Configuration Parameters** (`params.yaml`):
```yaml
control:
  max_linear_vel: 0.5       # m/s
  max_angular_vel: 1.0      # rad/s
  control_rate: 10.0        # Hz
correction:
  position_tolerance: 0.01  # m
  use_lidar_correction: true
```

---

### 3. `testbed_operation` - Integration & Vision

**Purpose**: Client scripts for integrated robot operation and QR code-based pose correction.

**Key Files**:
| File | Description |
|------|-------------|
| `scripts/testbed_operation_client_all_with_camera.py` | Full integration client with QR correction |
| `scripts/qr_pose_correction_server_test.py` | QR pose estimation server (Zivid + ArUco) |
| `scripts/battery_check.py` | Mobile robot battery status utility |
| `srv/QRPoseCorrection.srv` | QR-based pose correction service |
| `srv/MoveJoint.srv` | Doosan robot joint movement service |

**QRPoseCorrection Service**:
```
# Request
bool robot_ready
float64[6] current_joint_pos    # Current robot joint angles [deg]
string measurement_point_id     # e.g., "A-point1"
---
# Response
bool success
string message
float64 delta_x, delta_y, delta_z           # Position correction [mm]
float64 delta_rx, delta_ry, delta_rz        # Rotation correction [deg]
float64[9] rotation_matrix                  # 3x3 rotation matrix (row-major)
float64 qr_confidence                       # Detection confidence [0.0-1.0]
float64[3] qr_position                      # QR position in camera frame [mm]
```

**MoveJoint Service** (Doosan Robot):
```
float64[6] pos      # Target joint angles [deg]
float64 vel         # Velocity [deg/s]
float64 acc         # Acceleration [deg/s²]
float64 time        # Movement time [s]
int8 mode           # 0=ABSOLUTE, 1=RELATIVE
---
bool success
```

---

### 4. `woosh_robot_py` - WooshRobot SDK

**Purpose**: Python SDK for communicating with TR200 mobile robot via WebSocket.

**Architecture**:
```
woosh_robot_py/
├── woosh_robot.py          # Main robot interface class
├── woosh_interface.py      # Interface definitions & settings
├── woosh_base.py           # Base communication classes
└── woosh/proto/            # Protocol Buffer definitions
    ├── robot/              # Robot state, commands
    ├── map/                # Map management
    ├── task/               # Task execution
    └── ros/                # ROS action integration
```

**Key Class**: `WooshRobot`

**Important Methods**:
| Method | Description |
|--------|-------------|
| `run()` | Start WebSocket connection |
| `twist_req(Twist)` | Send velocity command (linear, angular) |
| `robot_info_req()` | Get robot information (battery, status) |
| `robot_pose_speed_req()` | Get current pose and speed |
| `switch_map_req()` | Load/switch navigation map |
| `init_robot_req()` | Initialize robot position |
| `switch_control_mode_req()` | Switch control mode (Auto/Manual) |
| `scanner_data_sub()` | Subscribe to LiDAR data |

**Connection Settings**:
```python
from woosh_interface import CommuSettings
settings = CommuSettings(
    addr="169.254.128.2",   # Robot IP
    port=5480,               # WebSocket port
    identity="my_client"     # Client identifier
)
robot = WooshRobot(settings)
await robot.run()
```

**Twist Command Example**:
```python
from woosh.proto.robot.robot_pack_pb2 import Twist
await robot.twist_req(Twist(linear=0.1, angular=0.0))  # Move forward
```

---

### 5. `doosan-robot` - Doosan Collaborative Robot

**Purpose**: Official Doosan Robotics ROS packages for controlling A0912 cobot.

**Sub-packages**:
| Package | Description |
|---------|-------------|
| `dsr_launcher` | Launch files for robot startup |
| `dsr_control` | Controller manager and joint control |
| `dsr_msgs` | ROS message and service definitions |
| `dsr_description` | URDF/Xacro robot description |
| `moveit_config_a0912` | MoveIt configuration for A0912 |
| `dsr_example` | Example scripts |

**Key Services** (under `/dsr01a0912/`):
| Service | Description |
|---------|-------------|
| `/motion/move_joint` | Joint space movement |
| `/motion/move_line` | Cartesian linear movement |
| `/motion/move_jointx` | Joint movement with Cartesian target |
| `/system/get_current_posj` | Get current joint position |
| `/system/get_current_posx` | Get current Cartesian position |
| `/gripper/robotiq_2f_open` | Open Robotiq gripper |
| `/gripper/robotiq_2f_close` | Close Robotiq gripper |

**Launch Command**:
```bash
roslaunch dsr_launcher dsr_moveit.launch model:=a0912 mode:=real host:=192.168.137.100
```

---

## 🔄 Data Flow Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    Central Coordinator / Client                   │
│              (testbed_operation_client_all_with_camera.py)       │
└──────────────────────────────────────────────────────────────────┘
                    │                              │
                    ▼                              ▼
    ┌───────────────────────────┐    ┌───────────────────────────┐
    │   /mobile_positiontwist   │    │  /dsr01a0912/motion/...   │
    │      (ROS Service)        │    │      (ROS Services)       │
    └───────────────────────────┘    └───────────────────────────┘
                    │                              │
                    ▼                              ▼
    ┌───────────────────────────┐    ┌───────────────────────────┐
    │   Mobile Robot Server     │    │    Doosan Robot Driver    │
    │ (ldj_mobile_posiotion_    │    │    (dsr_launcher)         │
    │  server_twist.py)         │    │                           │
    └───────────────────────────┘    └───────────────────────────┘
                    │                              │
          WebSocket │                         Ethernet
                    ▼                              ▼
    ┌───────────────────────────┐    ┌───────────────────────────┐
    │    TR200 Mobile Robot     │    │   Doosan A0912 Cobot      │
    │    (169.254.128.2:5480)   │    │   (192.168.137.100)       │
    └───────────────────────────┘    └───────────────────────────┘
```

---

## 🔌 ROS Interface Summary

### ROS Services

| Service Name | Type | Package | Description |
|--------------|------|---------|-------------|
| `/mobile_positiontwist` | `MobilePositionTwist` | mobile_robot_server | Move mobile robot by distance |
| `/mobile_lidar_control/move_distance` | `MoveDistance` | mobile_lidar_control | LiDAR-corrected movement |
| `/mobile_lidar_control/emergency_stop` | `std_srvs/Empty` | mobile_lidar_control | Emergency stop |
| `/qr_pose_correction` | `QRPoseCorrection` | testbed_operation | QR-based pose estimation |
| `/dsr01a0912/motion/move_joint` | `MoveJoint` | dsr_msgs | Doosan joint movement |

### ROS Topics

| Topic Name | Type | Description |
|------------|------|-------------|
| `/mobile_lidar/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/dsr01a0912/state` | `dsr_msgs/RobotState` | Doosan robot state |
| `/mobile/status` | `std_msgs/String` | Mobile robot status |
| `/doosan/status` | `std_msgs/String` | Doosan robot status |

---

## 🧩 Code Patterns

### Pattern 1: Asyncio + ROS Integration

The codebase uses a pattern for integrating asyncio with ROS:

```python
def run_asyncio_loop(controller):
    """Run asyncio event loop in separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(controller.run())

def main():
    rospy.init_node('my_node')
    controller = MyController()
    
    # Run asyncio in separate thread
    thread = Thread(target=run_asyncio_loop, args=(controller,), daemon=True)
    thread.start()
    
    rospy.spin()
```

### Pattern 2: Service Client Helper Functions

```python
def mobile_move(distance=0.1):
    """Move mobile robot by specified distance"""
    service_name = '/mobile_positiontwist'
    rospy.wait_for_service(service_name)
    
    move_mobile = rospy.ServiceProxy(service_name, MobilePositionTwist)
    req = MobilePositionTwistRequest()
    req.distance = distance
    
    response = move_mobile(req)
    return response.success

def dsr_move_joint(pos, vel=30.0, acc=30.0):
    """Move Doosan robot to joint position"""
    service_name = '/dsr01a0912/motion/move_joint'
    rospy.wait_for_service(service_name)
    
    move_joint = rospy.ServiceProxy(service_name, MoveJoint)
    req = MoveJointRequest()
    req.pos = pos
    req.vel = vel
    req.acc = acc
    
    return move_joint(req).success
```

### Pattern 3: Trapezoidal Velocity Profile

```python
# Acceleration → Cruise → Deceleration
while is_moving:
    remaining = target_distance - estimated_distance
    stop_dist = (speed ** 2) / (2 * decel)
    
    if remaining < tolerance:
        # Stop condition
        current_speed = 0.0
        is_moving = False
    elif remaining <= stop_dist:
        # Deceleration phase
        current_speed = max(current_speed - decel * dt, 0)
    else:
        # Acceleration/cruise phase
        current_speed = min(current_speed + accel * dt, max_speed)
    
    await robot.twist_req(Twist(linear=current_speed, angular=0.0))
    estimated_distance += current_speed * dt
```

---

## 🚀 Quick Start Commands

### 1. Docker Container Access
```bash
docker exec -it my_noetic_ws bash
source /root/catkin_ws/devel/setup.bash
```

### 2. Start Doosan Robot
```bash
roslaunch dsr_launcher dsr_moveit.launch model:=a0912 mode:=real host:=192.168.137.100
```

### 3. Start Mobile Robot Server
```bash
rosrun mobile_robot_server ldj_mobile_posiotion_server_twist.py
```

### 4. Test Mobile Robot Movement
```bash
rosservice call /mobile_positiontwist "{distance: 0.3}"   # Forward 0.3m
rosservice call /mobile_positiontwist "{distance: -0.3}"  # Backward 0.3m
```

### 5. Test Doosan Robot Movement
```bash
rosservice call /dsr01a0912/motion/move_joint \
  "{pos: [90.0, 0.0, 90.0, 0.0, 90.0, -90.0], vel: 30.0, acc: 30.0}"
```

### 6. Check Battery Status
```bash
rosrun testbed_operation battery_check.py
```

---

## 🔧 Configuration Reference

### Mobile Robot Parameters
```yaml
robot_ip: "169.254.128.2"
robot_port: 5480
max_linear_vel: 0.12        # m/s
linear_accel: 0.25          # m/s²
linear_decel: 0.50          # m/s²
control_rate: 50            # Hz
position_tolerance: 0.03    # m
```

### Doosan Robot Default Positions
```python
HOME_POSITION = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]  # Joint angles [deg]
QR_MEASUREMENT_POS = [49.54, 31.27, 87.67, 0.0, 61.06, -130.46]
```

### QR/ArUco Detection
```python
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_LENGTH = 0.02084  # meters (20.84mm)
```

---

## 📝 Important Notes for AI Assistants

### 1. ROS Version
- This project uses **ROS 1 Noetic** (NOT ROS 2)
- Use `rospy` (not `rclpy`)
- Use `roslaunch`, `rosrun`, `rosservice` commands

### 2. Asyncio Integration
- WooshRobot SDK is asyncio-based
- ROS callbacks run in main thread
- Use separate thread for asyncio event loop

### 3. Service Definitions Location
- Custom services are in each package's `srv/` directory
- Must run `catkin_make` after modifying `.srv` files

### 4. Network Configuration
- Doosan robot: LAN (192.168.137.x subnet)
- Mobile robot: Wi-Fi/Direct (169.254.128.x subnet)
- Both use `--network="host"` in Docker

### 5. Unit Conventions
- Mobile robot distances: meters (m)
- Doosan joint angles: degrees (deg)
- QR pose corrections: millimeters (mm) for position
- Velocities: m/s (mobile), deg/s (Doosan)

---

## 📚 Related Documentation

| Document | Location | Description |
|----------|----------|-------------|
| Environment Setup | `docs/ENVIRONMENT_SETUP.md` | Docker/ROS setup guide |
| Integration Guide | `docs/INTEGRATION_GUIDE.md` | System integration details |
| QR Implementation | `docs/QR/QR_NODE_IMPLEMENTATION_GUIDE.md` | QR vision system guide |
| Project Architecture | `Project.md` | Original project specification |
| Main README | `README.md` | Project overview |

---

## 🔍 Common Tasks for AI Assistants

### Task 1: Add New Motion Command
1. Modify `MobilePositionTwist.srv` or create new service
2. Update service server in `ldj_mobile_posiotion_server_twist.py`
3. Add client function in `testbed_operation_client_*.py`

### Task 2: Modify Robot Trajectory
1. Edit joint positions in client scripts
2. Use `move_joint` service with new positions
3. Consider velocity and acceleration limits

### Task 3: Integrate New Sensor
1. Create subscriber in appropriate package
2. Convert sensor data to ROS message type
3. Add to launch file if needed

### Task 4: Debug Communication Issues
1. Check robot IP connectivity: `ping <IP>`
2. Verify ROS services: `rosservice list`
3. Check topic data: `rostopic echo <topic>`
4. View node graph: `rqt_graph`

---

**Document Version**: 1.0.0  
**Last Updated**: 2025-12-05  
**Author**: KATECH Robotics Team  
**Contact**: djlee2@katech.re.kr
