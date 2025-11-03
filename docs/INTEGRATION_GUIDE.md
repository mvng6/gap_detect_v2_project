# 통합 관제 시스템 개발 가이드

**Mobile-Cobot Integrated Control System - Development Guide**

---

## 📋 문서 정보

**작성자**: LDJ (Dongjun Lee)  
**이메일**: djlee2@katech.re.kr  
**소속**: KATECH 스마트제조기술연구센터  
**최종 수정일**: 2025-11-03  
**버전**: 1.0.0

---

## 📝 목차

1. [개요](#1-개요)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [패키지 구조](#3-패키지-구조)
4. [개발 과정](#4-개발-과정)
5. [시퀀스 설명](#5-시퀀스-설명)
6. [확장 가이드](#6-확장-가이드)
7. [문제 해결](#7-문제-해결)

---

## 1. 개요

### 1.1 프로젝트 목표

본 프로젝트는 **모바일 로봇**과 **두산 협동 로봇(Doosan A0912)**을 하나의 중앙 관제 시스템으로 통합하여 순차적 협업 작업을 수행하는 것을 목표로 합니다.

### 1.2 핵심 요구사항

- ✅ **순차 제어**: 모바일 → 협동로봇 → 모바일 → 협동로봇 순서로 동작
- ✅ **상태 모니터링**: 각 로봇의 실시간 상태 확인
- ✅ **자동 초기화**: 로봇 연결 및 준비 상태 자동 감지
- ✅ **왕복 시퀀스**: 전진 → 작업 → 복귀 → 홈 사이클
- ✅ **에러 처리**: 연결 실패, 타임아웃 등 자동 복구

### 1.3 설계 원칙

| 원칙 | 설명 |
|------|------|
| **기존 코드 재사용** | 두산 로봇 드라이버 및 모바일 SDK 활용 |
| **단순성** | Topic 기반 통신으로 구현 복잡도 최소화 |
| **안전성** | 상태 확인 로직으로 로봇 간 충돌 방지 |
| **확장성** | 새로운 자세 및 시퀀스 쉽게 추가 가능 |

---

## 2. 시스템 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────┐
│          Central Coordinator Node                   │
│         (중앙 관제 노드 - Python)                     │
│                                                      │
│  • 로봇 초기화 및 상태 감지                            │
│  • 순차 제어 로직                                     │
│  • 상태 모니터링 및 에러 처리                          │
└─────────────────────────────────────────────────────┘
         │                              │
         │ Topic                        │ Topic
         ↓                              ↓
/mobile/cmd                    /katech/robot_command
(Float64MultiArray)            (Int32)
         │                              │
┌────────────────────┐        ┌───────────────────────┐
│ Mobile Robot Node  │        │ Doosan Robot Node     │
│ (Python - asyncio) │        │ (C++ - roscpp)        │
│                    │        │                       │
│ • SDK 래퍼         │        │ • Topic → Service     │
│ • 거리 제어        │        │ • 자세 변경           │
│ • 상태 발행        │        │ • 상태 발행           │
└────────────────────┘        └───────────────────────┘
         ↑                              ↑
         │ Topic                        │ Topic
         │                              │
    /mobile/status                 /doosan/status
    (String)                       (String)
         │                              │
         └──────────────┬───────────────┘
                        │ Subscribe
                        ↓
            [Central Coordinator]
```

### 2.2 통신 프로토콜

#### 명령 토픽

| 토픽 | 타입 | 방향 | 내용 |
|-----|------|------|------|
| `/mobile/cmd` | `std_msgs/Float64MultiArray` | Coordinator → Mobile | `[distance, speed]` |
| `/katech/robot_command` | `std_msgs/Int32` | Coordinator → Doosan | `0, 1, 99` (자세 ID) |

#### 상태 토픽

| 토픽 | 타입 | 방향 | 값 |
|-----|------|------|-----|
| `/mobile/status` | `std_msgs/String` | Mobile → Coordinator | `IDLE`, `MOVING`, `COMPLETED`, `ERROR` |
| `/doosan/status` | `std_msgs/String` | Doosan → Coordinator | `IDLE`, `MOVING`, `COMPLETED`, `ERROR` |
| `/dsr01a0912/state` | `dsr_msgs/RobotState` | Driver → Coordinator | 시스템 상태 (SAFE_OFF, STANDBY 등) |

---

## 3. 패키지 구조

### 3.1 패키지 개요

```
robot_ws/src/
├── central_coordinator/     # 중앙 관제 시스템
│   ├── src/
│   │   └── coordinator_node.py
│   └── launch/
│       └── integrated_system.launch
│
├── doosan_helper/          # 두산 로봇 제어
│   └── src/
│       ├── move_robot_node.cpp
│       ├── trigger_home_node.cpp
│       ├── trigger_one_node.cpp
│       └── trigger_zero_node.cpp
│
└── mobile_robot_control/   # 모바일 로봇 제어
    └── src/
        ├── move_mobile_robot_node.py
        └── mobile_robot_twist_control.py
```

### 3.2 의존성

```
central_coordinator
├─> rospy
├─> std_msgs
├─> dsr_msgs
├─> doosan_helper (move_robot_node)
└─> mobile_robot_control (move_mobile_robot_node)

doosan_helper
├─> roscpp
├─> std_msgs
├─> dsr_msgs
└─> dsr_control (driver)

mobile_robot_control
├─> rospy
├─> std_msgs
└─> asyncio (Python)
```

---

## 4. 개발 과정

### 4.1 Step 1: 두산 로봇 노드 수정

**파일**: `src/doosan_helper/src/move_robot_node.cpp`

**수정 사항**:
1. 상태 발행 기능 추가 (`/doosan/status`)
2. 자세 ID 정의 (0, 1, 99)
3. 서비스 호출 결과에 따른 상태 변경

**핵심 코드**:
```cpp
// 전역 Publisher
ros::Publisher status_pub;

// 콜백 함수
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    // 상태 발행: MOVING
    std_msgs::String status;
    status.data = "MOVING";
    status_pub.publish(status);
    
    // 서비스 호출
    if (client.call(srv) && srv.response.success) {
        status.data = "COMPLETED";
    } else {
        status.data = "ERROR";
    }
    status_pub.publish(status);
}
```

### 4.2 Step 2: 모바일 로봇 노드 작성

**파일**: `src/mobile_robot_control/src/move_mobile_robot_node.py`

**주요 기능**:
1. 기존 `MobileRobotTwistController` 래핑
2. ROS Topic 통합
3. 비동기 실행 (threading + asyncio)
4. 상태 발행

**핵심 구조**:
```python
class MobileRobotROSNode:
    def command_callback(self, msg):
        # Topic 수신
        distance, speed = msg.data[0], msg.data[1]
        
        # 비동기 실행
        thread = threading.Thread(target=self.execute_movement, args=(distance, speed))
        thread.start()
    
    async def _async_move(self, distance, speed):
        self.publish_status("MOVING")
        
        controller = MobileRobotTwistController(init_node=False)
        success = await controller.move_distance(distance, speed)
        
        if success:
            self.publish_status("COMPLETED")
        else:
            self.publish_status("ERROR")
```

### 4.3 Step 3: 중앙 관제 노드 작성

**파일**: `src/central_coordinator/src/coordinator_node.py`

**핵심 로직**:

```python
class CentralCoordinator:
    def initialize_robots(self):
        # 1. 두산 STANDBY 대기
        self.wait_for_doosan_ready(timeout=60.0)
        
        # 2. 홈 위치 이동
        self.send_doosan_command(99)
        self.wait_for_status("doosan", "COMPLETED")
    
    def run_sequence(self):
        while not rospy.is_shutdown():
            # 1. 모바일 전진
            self.send_mobile_command(distance, speed)
            self.wait_for_status("mobile", "COMPLETED")
            
            # 2. 두산 작업
            self.send_doosan_command(1)
            self.wait_for_status("doosan", "COMPLETED")
            
            # 3. 모바일 후진
            self.send_mobile_command(distance, -speed)
            self.wait_for_status("mobile", "COMPLETED")
            
            # 4. 두산 홈
            self.send_doosan_command(99)
            self.wait_for_status("doosan", "COMPLETED")
            
            # 5. 대기
            rospy.sleep(cycle_delay)
```

### 4.4 Step 4: Launch 파일 작성

**파일**: `src/central_coordinator/launch/integrated_system.launch`

```xml
<launch>
    <!-- 파라미터 -->
    <arg name="mobile_distance" default="0.3"/>
    <arg name="mobile_speed" default="0.2"/>
    <arg name="cycle_delay" default="5.0"/>
    
    <!-- 두산 드라이버 -->
    <include file="$(find dsr_launcher)/launch/single_robot.launch">
        <arg name="model" value="a0912"/>
        <arg name="mode" value="real"/>
        <arg name="host" value="192.168.137.100"/>
    </include>
    
    <!-- 두산 제어 노드 -->
    <node name="move_robot_node" pkg="doosan_helper" type="move_robot_node"/>
    
    <!-- 모바일 제어 노드 -->
    <node name="mobile_robot_ros_node" pkg="mobile_robot_control" type="move_mobile_robot_node.py"/>
    
    <!-- 중앙 관제 노드 -->
    <node name="central_coordinator" pkg="central_coordinator" type="coordinator_node.py">
        <param name="mobile_distance" value="$(arg mobile_distance)"/>
        <param name="mobile_speed" value="$(arg mobile_speed)"/>
        <param name="cycle_delay" value="$(arg cycle_delay)"/>
    </node>
</launch>
```

---

## 5. 시퀀스 설명

### 5.1 초기화 시퀀스

```
[프로그램 시작]
    │
    ├─> ROS 노드 초기화
    │   ├─ central_coordinator
    │   ├─ move_robot_node
    │   └─ mobile_robot_ros_node
    │
    ├─> 토픽 구독 시작
    │   ├─ /dsr01a0912/state (두산 시스템 상태)
    │   ├─ /doosan/status (두산 동작 상태)
    │   └─ /mobile/status (모바일 상태)
    │
    ├─> 두산 로봇 STANDBY 대기
    │   ├─ robot_state == 1 확인
    │   ├─ SAFE_OFF(3) 감지 시 → 서보 온 안내
    │   └─ 타임아웃: 60초
    │
    ├─> 두산 홈 위치 이동
    │   ├─ 명령: /katech/robot_command = 99
    │   ├─ 대기: /doosan/status == "COMPLETED"
    │   └─ 타임아웃: 60초
    │
    └─> [초기화 완료] → 메인 사이클 시작
```

### 5.2 메인 사이클

```
[사이클 N 시작]
    │
    ├─> [1/4] 모바일 로봇 전진
    │   ├─ 발행: /mobile/cmd = [0.3, 0.2]
    │   │         (0.3m 전진, 0.2m/s 속도)
    │   ├─ 대기: /mobile/status == "COMPLETED"
    │   └─ 타임아웃: 60초
    │
    ├─> [2/4] 두산 로봇 작업 자세
    │   ├─ 발행: /katech/robot_command = 1
    │   │         (자세: [-90, 0, 90, 0, 90, -90])
    │   ├─ 대기: /doosan/status == "COMPLETED"
    │   └─ 타임아웃: 60초
    │
    ├─> [3/4] 모바일 로봇 후진 (복귀)
    │   ├─ 발행: /mobile/cmd = [0.3, -0.2]
    │   │         (0.3m 후진, 0.2m/s 속도)
    │   ├─ 대기: /mobile/status == "COMPLETED"
    │   └─ 타임아웃: 60초
    │
    ├─> [4/4] 두산 로봇 홈 위치
    │   ├─ 발행: /katech/robot_command = 99
    │   │         (자세: [0, 0, 0, 0, 0, 0])
    │   ├─ 대기: /doosan/status == "COMPLETED"
    │   └─ 타임아웃: 60초
    │
    ├─> [사이클 완료]
    │
    ├─> cycle_delay (5초) 대기
    │
    └─> [사이클 N+1 시작]
```

### 5.3 에러 처리

```
[단계 실행 중]
    │
    ├─> 타임아웃 발생?
    │   └─> YES → 에러 로그 출력 → 10초 대기 → 재시도
    │
    ├─> 상태 = "ERROR"?
    │   └─> YES → 에러 로그 출력 → 10초 대기 → 재시도
    │
    └─> Ctrl+C (사용자 중단)?
        └─> YES → 안전하게 종료
```

---

## 6. 확장 가이드

### 6.1 새로운 자세 추가

#### 두산 로봇 자세 추가

**파일**: `src/doosan_helper/src/move_robot_node.cpp`

```cpp
// commandCallback 함수에 추가
else if (msg->data == 2)  // 새 명령 ID
{
    ROS_INFO("Trigger message '2' received.");
    srv.request.pos = {45.0, -45.0, 90.0, 0.0, 45.0, 0.0};  // 새 자세
    should_call_service = true;
}
```

**Trigger 노드 작성** (선택사항):

```cpp
// src/doosan_helper/src/trigger_two_node.cpp
#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger_two_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("/katech/robot_command", 1, true);
    ros::Duration(0.5).sleep();
    
    std_msgs::Int32 msg;
    msg.data = 2;
    pub.publish(msg);
    
    ros::Duration(0.5).sleep();
    return 0;
}
```

**CMakeLists.txt 수정**:
```cmake
add_executable(trigger_two_node src/trigger_two_node.cpp)
target_link_libraries(trigger_two_node ${catkin_LIBRARIES})
```

### 6.2 시퀀스 변경

**예시**: 작업 자세를 2번 반복

```python
# coordinator_node.py
def run_sequence(self):
    while not rospy.is_shutdown():
        # 1. 모바일 전진
        self.send_mobile_command(self.mobile_distance, self.mobile_speed)
        self.wait_for_status("mobile", "COMPLETED")
        
        # 2-1. 두산 작업 자세 (첫 번째)
        self.send_doosan_command(1)
        self.wait_for_status("doosan", "COMPLETED")
        rospy.sleep(2.0)  # 작업 시간
        
        # 2-2. 두산 작업 자세 (두 번째)
        self.send_doosan_command(2)  # 다른 자세
        self.wait_for_status("doosan", "COMPLETED")
        
        # 3. 모바일 후진
        # ... (동일)
```

### 6.3 다른 모바일 로봇 연결

**파일**: `src/mobile_robot_control/src/mobile_robot_twist_control.py`

```python
# __init__ 함수에서 IP 변경
self.robot_ip = rospy.get_param('~robot_ip', '192.168.1.100')  # 새 IP
self.robot_port = rospy.get_param('~robot_port', 8080)  # 새 포트
```

**Launch 파일에서 파라미터 전달**:
```xml
<node name="mobile_robot_ros_node" pkg="mobile_robot_control" type="move_mobile_robot_node.py">
    <param name="robot_ip" value="192.168.1.100"/>
    <param name="robot_port" value="8080"/>
</node>
```

---

## 7. 문제 해결

### 7.1 일반적인 문제

#### 빌드 오류

```bash
# 의존성 재설치
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
catkin_make clean
catkin_make
source devel/setup.bash
```

#### 노드 실행 오류

```bash
# 환경 변수 확인
echo $ROS_MASTER_URI
echo $ROS_IP

# 환경 재설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### 7.2 로봇별 문제

#### 두산 로봇

| 문제 | 해결 |
|------|------|
| SAFE_OFF에서 멈춤 | 티치 펜던트에서 서보 온 |
| 서비스 호출 실패 | 드라이버 재실행 |
| 목표 도달 실패 | 자세 충돌 확인, 특이점 회피 |

#### 모바일 로봇

| 문제 | 해결 |
|------|------|
| 연결 실패 | IP/포트 확인, Wi-Fi 연결 |
| Import 오류 | `importlib` 사용 (이미 적용됨) |
| 거리 부정확 | 가감속 파라미터 조정 |
| 배터리 부족 | `rosrun mobile_robot_control battery_check.py` 실행 |

---

## 8. 유틸리티

### 8.1 배터리 체크

모바일 로봇의 배터리 잔량을 확인하는 도구입니다.

```bash
# 실행
rosrun mobile_robot_control battery_check.py
```

**출력 예시**:
```
============================================================
📊 모바일 로봇 배터리 상태
============================================================

🪫  배터리 잔량: 37% (주의)

[███████████████░░░░░░░░░░░░░░░░░░░░░░░░░] 37%

============================================================
```

**배터리 레벨 표시**:
- 80% 이상: 🔋 초록색 (충분)
- 50~79%: 🔋 청록색 (보통)
- 20~49%: 🪫 노란색 (주의)
- 20% 미만: 🪫 빨간색 (위험)

---

## 📝 라이선스 및 저작권

**Copyright © 2025 KATECH (Korea Automotive Technology Institute)**  
**Smart Manufacturing Technology Research Center**

**Author**: LDJ (Dongjun Lee)  
**Email**: djlee2@katech.re.kr

---

## 🔗 관련 문서

- [메인 README](../README.md) - 프로젝트 개요 및 실행
- [환경 구축 가이드](ENVIRONMENT_SETUP.md) - Docker 및 ROS 설정
- [central_coordinator](../src/central_coordinator/README.md) - 중앙 관제 노드
- [doosan_helper](../src/doosan_helper/README.md) - 두산 로봇 제어
- [mobile_robot_control](../src/mobile_robot_control/README.md) - 모바일 로봇 제어

---

**Integration Guide for Mobile-Cobot Integrated Control System**  
**Built by KATECH Smart Manufacturing Technology Research Center**
