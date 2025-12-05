# Codebase Summary & Analysis

## 1. Project Overview
This project is an integrated control system for a **Mobile Robot** and a **Doosan Collaborative Robot (A0912)**. It uses **ROS 1 Noetic** to coordinate the two robots to perform a sequential task: mobile robot movement followed by a series of robotic arm manipulations.

**Key Characteristic**: Unlike the design document (`Project.md`) which suggests ROS Actions and Topics, the actual implementation relies heavily on **Standard ROS Services** for synchronous control.

## 2. Directory Structure
The workspace is organized into the following key packages:

```
src/
├── mobile_robot_server/       # Mobile Robot Control Package
│   └── scripts/
│       └── mobile_posiotion_server_twist.py  # Main Mobile Node
│
├── testbed_operation/         # Central Coordinator Package
│   └── scripts/
│       └── testbed_operation_client_all.py   # Main Coordinator Node (The "Conductor")
│       └── battery_check.py                  # Utility
│
├── doosan-robot/              # Doosan Robot Production Packages
│   ├── dsr_msgs/              # Service & Message definitions
│   └── ...                    # Other driver/moveit packages
│
└── woosh_robot_py/            # Mobile Robot SDK/Driver (woosh)
```

## 3. Key Components Analysis

### A. Central Coordinator
- **File**: `src/testbed_operation/scripts/testbed_operation_client_all.py`
- **Role**: Validates the entire sequence. It acts as a Service Client for both robots.
- **Logic**:
    1.  **DSR Move**: Calls `/dsr01a0912/motion/move_joint` service.
    2.  **Mobile Move**: Calls `/mobile_positiontwist` service.
    3.  **Sequence**: Hardcoded sequence of "Move Arm -> Move Mobile -> Move Arm..."

### B. Mobile Robot Node
- **File**: `src/mobile_robot_server/scripts/mobile_posiotion_server_twist.py`
- **Role**: wrapper node that exposes a ROS Service (`/mobile_positiontwist`) to control the mobile robot.
- **Implementation**:
    - Uses `asyncio` and `SmoothTwistController` class.
    - Controls velocity (`cmd_vel`) directly to achieve target distance.
    - Implements custom acceleration/deceleration profiles.

### C. Doosan Robot Node
- **Role**: Standard Doosan Robot driver nodes (likely `doosan_robot` package) are used.
- **Interface**: The coordinator directly calls standard services provided by the driver, such as `/dsr01a0912/motion/move_joint`.

## 4. Operational Logic (Sequence)
Based on `testbed_operation_client_all.py`, the system executes the following loop 4 times:

1.  **Initialization**: Move Arm to Home.
2.  **Measurement Sequence A**: Move Arm to 3 different points (A-point 1, 2, 3).
3.  **Reset**: Move Arm to Home.
4.  **Mobile Move**: Move Mobile Robot **Forward 0.3m**.
5.  **Measurement Sequence B**: Move Arm to 4 different points (B-point 1, 2, 3, 4).
6.  **Reset**: Move Arm to Home.
7.  **Mobile Move**: Move Mobile Robot **Forward 0.6m**.
8.  **Measurement Sequence C**: Move Arm to 5 different points (C-point 1..5).
9.  **Reset**: Move Arm to Home.
10. **Return**: Move Mobile Robot **Backward 0.9m** (Return to start).

## 5. Discrepancies vs Design
| Feature | Design Document (`Project.md`) | Actual Implementation |
| :--- | :--- | :--- |
| **Communication** | ROS Actions + Topics (`/mobile/cmd`) | ROS Services (`/mobile_positiontwist`, `/move_joint`) |
| **Coordinator Pkg** | `central_coordinator` | `testbed_operation` |
| **Mobile Pkg** | `mobile_robot_control` | `mobile_robot_server` |
| **State Monitoring** | Topic Subscription (`/status`) | Blocking Service Calls (Wait for completion) |

> [!NOTE]
> The current system uses a **Synchronous Blocking** approach via Services, which is simpler but less reactive than the Action-based design proposed in the documentation.
