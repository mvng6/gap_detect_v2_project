# Doosan Robot Control 🤖

두산로봇(Doosan Robotics Cobots)을 제어하기 위한 확장 가능한 ROS 패키지입니다.

기존 두산로봇 SDK(`doosan-robot`)를 기반으로 사용하기 쉽고 확장 가능한 인터페이스를 제공합니다.

## ✨ 주요 특징

- **🔌 간편한 연결**: IP 주소만으로 로봇과 쉽게 연결
- **📊 상태 모니터링**: 실시간 로봇 상태, 관절 위치 확인
- **🎯 확장 가능한 구조**: 클래스 기반 설계로 기능 추가 용이
- **⚙️ ROS 통합**: ROS 토픽/서비스를 통한 표준 인터페이스
- **📝 상세한 문서**: Google C++ Style Guide 준수, Doxygen 주석

## 📋 목차

- [요구사항](#요구사항)
- [설치](#설치)
- [사용 방법](#사용-방법)
- [설정](#설정)
- [아키텍처](#아키텍처)
- [API 문서](#api-문서)
- [예제](#예제)
- [문제 해결](#문제-해결)

## 🔧 요구사항

### 시스템 환경
- **OS**: Ubuntu 20.04 (ROS Noetic) 또는 Docker 환경
- **ROS**: ROS Noetic
- **C++ 컴파일러**: C++17 지원 (GCC 7.0+)

### 의존성 패키지
```bash
# ROS 패키지
sudo apt-get install ros-noetic-ros-base
sudo apt-get install ros-noetic-moveit

# 두산로봇 패키지 (이 워크스페이스에 포함)
- dsr_msgs
- dsr_control
- dsr_launcher
```

## 🚀 설치

### 1. 워크스페이스 준비
이미 `robot_ws`가 있다면 생략하세요.

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
```

### 2. 패키지 복사/생성
이 패키지는 이미 `~/robot_ws/src/doosan_robot_control/`에 있습니다.

### 3. 빌드
```bash
cd ~/robot_ws

# Catkin 빌드
catkin_make

# 또는 catkin_tools 사용
catkin build doosan_robot_control

# 환경변수 설정
source devel/setup.bash
```

## 📖 사용 방법

### 기본 사용법

#### 방법 1: 독립 실행 (dsr_control과 별개)

먼저 별도 터미널에서 두산로봇 드라이버 실행:
```bash
# 터미널 1: 두산로봇 드라이버
roslaunch dsr_launcher dsr_moveit.launch \
  model:=a0912 \
  mode:=real \
  host:=192.168.137.100
```

그 다음 이 패키지 실행:
```bash
# 터미널 2: Custom Control Node
roslaunch doosan_robot_control doosan_robot_control.launch \
  model:=a0912 \
  host:=192.168.137.100
```

#### 방법 2: 통합 실행 (한 번에)

```bash
roslaunch doosan_robot_control doosan_robot_with_dsr.launch \
  model:=a0912 \
  mode:=real \
  host:=192.168.137.100
```

### Launch 파일 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `model` | `a0912` | 로봇 모델 (a0509, a0912, m1013 등) |
| `host` | `192.168.137.100` | 로봇 IP 주소 |
| `port` | `12345` | 로봇 포트 |
| `mode` | `real` | 동작 모드 (`real` / `virtual`) |
| `robot_id` | `dsr01` | 로봇 네임스페이스 |
| `control_rate` | `10.0` | 제어 주기 (Hz) |
| `run_demo` | `false` | 데모 동작 실행 여부 |

### 예제 명령어

```bash
# 기본 실행 (A0912, Real 모드)
roslaunch doosan_robot_control doosan_robot_control.launch

# M1013 모델, 다른 IP
roslaunch doosan_robot_control doosan_robot_control.launch \
  model:=m1013 \
  host:=192.168.1.100

# Virtual 모드 (시뮬레이션)
roslaunch doosan_robot_control doosan_robot_control.launch \
  mode:=virtual

# 데모 모드 활성화
roslaunch doosan_robot_control doosan_robot_control.launch \
  run_demo:=true
```

## ⚙️ 설정

### YAML 설정 파일

`config/robot_config.yaml` 파일에서 세부 설정을 변경할 수 있습니다:

```yaml
# 로봇 연결
robot_id: "dsr01"
robot_model: "a0912"
host: "192.168.137.100"
port: 12345
mode: "real"

# 제어 설정
control_rate: 10.0
default_velocity: 0.5
default_acceleration: 0.5

# 안전 설정
enable_joint_limit_check: true
enable_singularity_check: true

# 사용자 정의 위치
named_positions:
  home: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  ready: [0.0, -20.0, 110.0, 0.0, 90.0, 0.0]
```

## 🏗️ 아키텍처

### 디렉토리 구조

```
doosan_robot_control/
├── src/
│   ├── doosan_robot_controller.cpp      # 메인 컨트롤러 구현
│   └── doosan_robot_control_node.cpp    # ROS 노드 진입점
├── include/doosan_robot_control/
│   └── doosan_robot_controller.h        # 컨트롤러 헤더
├── launch/
│   ├── doosan_robot_control.launch      # 기본 launch 파일
│   └── doosan_robot_with_dsr.launch     # dsr_control 통합 launch
├── config/
│   └── robot_config.yaml                # 설정 파일
├── CMakeLists.txt                        # 빌드 설정
├── package.xml                           # 패키지 정보
└── README.md                             # 이 문서
```

### 클래스 구조

```cpp
namespace doosan_robot_control {

// 설정 구조체
struct RobotConfig { ... }
struct RobotStatus { ... }

// 메인 컨트롤러 클래스
class DoosanRobotController {
public:
    // 초기화 및 연결
    bool initialize();
    bool waitForConnection(double timeout_sec);
    
    // 정보 조회
    RobotStatus getRobotStatus() const;
    std::array<double, 6> getCurrentJointPositions() const;
    bool getCurrentPose(geometry_msgs::Pose& pose);
    
    // 동작 제어
    bool moveJoint(const std::array<double, 6>& positions, ...);
    bool moveLine(const geometry_msgs::Pose& pose, ...);
    bool stopMotion(int stop_mode);
    
    // 제어 루프
    void spin();
    bool spinOnce();
};

} // namespace doosan_robot_control
```

## 📚 API 문서

### 주요 메서드

#### `initialize()`
로봇 제어를 위한 ROS 토픽/서비스 초기화

```cpp
bool initialize();
```

#### `waitForConnection()`
로봇과의 연결 대기

```cpp
bool waitForConnection(double timeout_sec = 10.0);
```

**Parameters:**
- `timeout_sec`: 타임아웃 (초)

**Returns:** 연결 성공 여부

#### `moveJoint()`
관절 공간 이동 (Joint Space Motion)

```cpp
bool moveJoint(
    const std::array<double, 6>& joint_positions,
    double velocity = 0.5,
    double acceleration = 0.5
);
```

**Parameters:**
- `joint_positions`: 목표 관절 위치 (rad, 6축)
- `velocity`: 속도 비율 (0.0 ~ 1.0)
- `acceleration`: 가속도 비율 (0.0 ~ 1.0)

**Returns:** 명령 전송 성공 여부

#### `getCurrentJointPositions()`
현재 관절 위치 조회

```cpp
std::array<double, 6> getCurrentJointPositions() const;
```

**Returns:** 6축 관절 위치 (rad)

### ROS 토픽

#### Published Topics

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/katech/doosan_status` | `std_msgs/String` | 로봇 상태 메시지 |
| `/katech/doosan_connected` | `std_msgs/Bool` | 연결 상태 (latched) |

#### Subscribed Topics

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/{ns}{model}/state` | `dsr_msgs/RobotState` | 로봇 상태 |
| `/{ns}{model}/joint_states` | `sensor_msgs/JointState` | 관절 상태 |
| `/{ns}{model}/error` | `dsr_msgs/RobotError` | 에러 정보 |

### ROS 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/{ns}{model}/motion/move_joint` | `dsr_msgs/MoveJoint` | 관절 이동 |
| `/{ns}{model}/motion/move_line` | `dsr_msgs/MoveLine` | 직선 이동 |
| `/{ns}{model}/motion/stop` | `dsr_msgs/Stop` | 정지 |
| `/{ns}{model}/system/get_current_pose` | `dsr_msgs/GetCurrentPose` | 현재 위치 |

## 💡 예제

### C++ 코드에서 사용

```cpp
#include <ros/ros.h>
#include "doosan_robot_control/doosan_robot_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_robot_app");
    ros::NodeHandle nh;
    
    // 설정
    doosan_robot_control::RobotConfig config;
    config.robot_model = "a0912";
    config.host = "192.168.137.100";
    config.mode = "real";
    
    // 컨트롤러 생성
    doosan_robot_control::DoosanRobotController controller(nh, config);
    
    // 초기화 및 연결
    if (!controller.initialize()) {
        ROS_ERROR("초기화 실패");
        return 1;
    }
    
    if (!controller.waitForConnection(30.0)) {
        ROS_ERROR("연결 실패");
        return 1;
    }
    
    // 현재 위치 확인
    auto joints = controller.getCurrentJointPositions();
    ROS_INFO("현재 J1: %.2f도", joints[0] * 180.0 / M_PI);
    
    // 홈 포지션으로 이동
    std::array<double, 6> home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    controller.moveJoint(home, 0.3, 0.3);
    
    // 제어 루프 실행
    controller.spin();
    
    return 0;
}
```

### Python에서 ROS 토픽 사용

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String

def status_callback(msg):
    rospy.loginfo(f"로봇 상태: {msg.data}")

def main():
    rospy.init_node('robot_monitor')
    
    # 상태 구독
    rospy.Subscriber('/katech/doosan_status', String, status_callback)
    
    # 연결 상태 확인
    rospy.wait_for_message('/katech/doosan_connected', Bool, timeout=10.0)
    rospy.loginfo("로봇 연결됨!")
    
    rospy.spin()

if __name__ == '__main__':
    main()
```

## 🔧 문제 해결

### 1. "로봇 연결 실패" 에러

**증상:**
```
[ERROR] 로봇 연결 실패!
```

**해결 방법:**
1. `dsr_control` 노드가 실행 중인지 확인:
   ```bash
   rosnode list | grep dsr
   ```

2. 로봇 IP 주소 확인:
   - 티치펜던트에서 설정 → 네트워크 확인
   - `ping 192.168.137.100` 테스트

3. 네임스페이스 확인:
   ```bash
   rostopic list | grep dsr01
   ```

### 2. Joint States 토픽이 수신되지 않음

**증상:**
```
대기 중... (10초 경과)
```

**해결 방법:**
```bash
# 토픽 확인
rostopic list | grep joint_states

# 토픽 모니터링
rostopic echo /dsr01a0912/joint_states

# 수동으로 토픽 발행 (테스트용)
rostopic pub /dsr01a0912/joint_states sensor_msgs/JointState "..."
```

### 3. 컴파일 에러

**증상:**
```
fatal error: dsr_msgs/RobotState.h: No such file or directory
```

**해결 방법:**
```bash
# 의존성 빌드
cd ~/robot_ws
catkin_make --pkg dsr_msgs

# 전체 재빌드
catkin_make clean
catkin_make

# 환경변수 재설정
source devel/setup.bash
```

### 4. Docker 환경에서 실행

Docker 컨테이너 내에서 실행하는 경우:

```bash
# 컨테이너 진입
docker exec -it my_noetic_ws bash

# 워크스페이스로 이동
cd /root/robot_ws  # 또는 해당 경로

# 환경변수 설정
source devel/setup.bash

# 실행
roslaunch doosan_robot_control doosan_robot_control.launch
```

## 🤝 확장 및 커스터마이징

### 새로운 동작 추가

`DoosanRobotController` 클래스를 상속하여 기능 확장:

```cpp
class MyCustomController : public DoosanRobotController {
public:
    MyCustomController(ros::NodeHandle& nh, const RobotConfig& config)
        : DoosanRobotController(nh, config) {}
    
    // 커스텀 동작 추가
    bool myCustomMotion() {
        // 구현
        return true;
    }
};
```

### 콜백 오버라이드

```cpp
protected:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) override {
        // 기본 동작
        DoosanRobotController::jointStateCallback(msg);
        
        // 추가 로직
        // ...
    }
```

## 📝 라이선스

MIT License

## 👥 저자

KATECH Robotics Team

## 📧 문의

robotics@katech.re.kr

---

**행복한 코딩 되세요! 🎉**

