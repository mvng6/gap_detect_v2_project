# Doosan Helper Package

**두산 협동 로봇 제어 패키지**

---

## 📋 패키지 정보

**패키지명**: `doosan_helper`  
**작성자**: LDJ (Dongjun Lee)  
**이메일**: djlee2@katech.re.kr  
**소속**: KATECH 스마트제조기술연구센터  
**최종 수정일**: 2025-11-03  
**버전**: 1.0.0

---

## 📝 개요

`doosan_helper`는 두산 협동 로봇(Doosan A0912)을 제어하기 위한 ROS 노드 모음 패키지입니다. Topic 기반 통신으로 로봇의 자세를 변경하고, 상태를 모니터링할 수 있습니다.

### 주요 기능

- ✅ **Topic 기반 제어**: 간단한 정수 메시지로 로봇 자세 변경
- ✅ **상태 발행**: 로봇 동작 상태 실시간 발행 (IDLE, MOVING, COMPLETED, ERROR)
- ✅ **사전 정의 자세**: 홈, 작업 자세 등 미리 정의된 관절 각도
- ✅ **Trigger 노드**: 원샷 실행으로 특정 자세 명령 발행

---

## 🏗️ 패키지 구조

### 노드 목록

| 노드명 | 타입 | 역할 |
|-------|------|------|
| `move_robot_node` | Server | 명령 수신 및 로봇 제어 (상시 실행) |
| `trigger_home_node` | Client | 홈 자세 명령 발행 (원샷) |
| `trigger_one_node` | Client | 자세 1 명령 발행 (원샷) |
| `trigger_zero_node` | Client | 자세 0 명령 발행 (원샷) |

### 자세 정의

| 명령 ID | 자세명 | 관절 각도 [deg] |
|--------|--------|----------------|
| 99 | 홈 (Home) | `[0, 0, 0, 0, 0, 0]` |
| 1 | 자세 1 | `[-90, 0, 90, 0, 90, -90]` |
| 0 | 자세 0 | `[90, 0, 90, 0, 90, -90]` |

---

## 📁 파일 구조

```
doosan_helper/
├── CMakeLists.txt              # 빌드 설정
├── package.xml                 # 패키지 메타데이터
├── README.md                   # 본 문서
├── src/
│   ├── move_robot_node.cpp     # 메인 제어 노드
│   ├── trigger_home_node.cpp   # 홈 자세 트리거
│   ├── trigger_one_node.cpp    # 자세 1 트리거
│   └── trigger_zero_node.cpp   # 자세 0 트리거
├── launch/                     # (비어있음)
└── include/                    # (비어있음)
```

---

## 🚀 사용 방법

### 1. 메인 제어 노드 실행

```bash
# 환경 설정
source ~/catkin_ws/devel/setup.bash

# 노드 실행 (명령 대기 모드)
rosrun doosan_helper move_robot_node
```

### 2. Trigger 노드 실행 (수동 제어)

```bash
# 홈 자세로 이동
rosrun doosan_helper trigger_home_node

# 자세 1로 이동
rosrun doosan_helper trigger_one_node

# 자세 0으로 이동
rosrun doosan_helper trigger_zero_node
```

### 3. Topic으로 직접 제어

```bash
# 홈 자세 (99)
rostopic pub /katech/robot_command std_msgs/Int32 "data: 99" -1

# 자세 1
rostopic pub /katech/robot_command std_msgs/Int32 "data: 1" -1

# 자세 0
rostopic pub /katech/robot_command std_msgs/Int32 "data: 0" -1
```

---

## 📡 ROS 인터페이스

### move_robot_node

#### Subscribed Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/katech/robot_command` | `std_msgs/Int32` | 자세 명령 (0, 1, 99) |

#### Published Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/doosan/status` | `std_msgs/String` | 동작 상태 (IDLE, MOVING, COMPLETED, ERROR) |

#### Service Clients

| 서비스 | 타입 | 설명 |
|-------|------|------|
| `/dsr01a0912/motion/move_joint` | `dsr_msgs/MoveJoint` | 관절 이동 서비스 |

---

## 🔄 동작 흐름

### move_robot_node

```
[시작]
  │
  ├─> ROS 노드 초기화
  │
  ├─> /doosan/status Publisher 생성
  │
  ├─> 초기 상태 발행: "IDLE"
  │
  ├─> /dsr01a0912/motion/move_joint 서비스 대기
  │
  ├─> /katech/robot_command Topic 구독 시작
  │
  └─> [명령 대기]
      │
      ├─> 명령 수신 (0, 1, 또는 99)
      │
      ├─> 상태 발행: "MOVING"
      │
      ├─> move_joint 서비스 호출
      │   ├─ 속도: 30 deg/s
      │   ├─ 가속도: 60 deg/s²
      │   └─ 모드: 절대 위치
      │
      ├─> 결과 확인
      │   ├─ 성공 → 상태 발행: "COMPLETED"
      │   └─ 실패 → 상태 발행: "ERROR"
      │
      └─> [명령 대기]
```

### trigger_*_node

```
[실행]
  │
  ├─> ROS 노드 초기화
  │
  ├─> /katech/robot_command Publisher 생성
  │
  ├─> 명령 발행 (0.5초 대기)
  │   └─ data: [99 | 1 | 0]
  │
  └─> [종료]
```

---

## ⚙️ 소스 코드 설명

### move_robot_node.cpp

**핵심 함수**:

```cpp
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    // 명령에 따라 자세 설정
    if (msg->data == 99) {
        // 홈: [0, 0, 0, 0, 0, 0]
    } else if (msg->data == 1) {
        // 자세 1: [-90, 0, 90, 0, 90, -90]
    } else if (msg->data == 0) {
        // 자세 0: [90, 0, 90, 0, 90, -90]
    }
    
    // 상태 발행: MOVING
    // move_joint 서비스 호출
    // 상태 발행: COMPLETED or ERROR
}
```

**주요 변수**:
- `status_pub`: 상태 발행용 Publisher (전역)
- `move_client`: move_joint 서비스 클라이언트
- `srv.request.vel`: 속도 (30 deg/s)
- `srv.request.acc`: 가속도 (60 deg/s²)
- `srv.request.mode`: 이동 모드 (0 = 절대 위치)

---

## 🐛 디버깅

### 로그 확인

```bash
# 노드 정보
rosnode info /move_robot_node

# 상태 실시간 확인
rostopic echo /doosan/status

# 명령 모니터링
rostopic echo /katech/robot_command
```

### 서비스 수동 호출

```bash
# move_joint 서비스 직접 호출
rosservice call /dsr01a0912/motion/move_joint \
  "pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
   vel: 30.0
   acc: 60.0
   time: 0.0
   radius: 0.0
   mode: 0
   blendType: 0
   syncType: 0"
```

---

## ⚠️ 주의사항

### 안전 수칙

1. **작업 공간**: 로봇 동작 범위 내에 사람이나 장애물이 없는지 확인
2. **비상 정지**: 티치 펜던트의 비상 정지 버튼 위치 숙지
3. **속도 제한**: 처음 테스트 시 속도를 낮춰서 실행
4. **서보 온**: 로봇이 STANDBY 상태인지 확인

### 알려진 제한사항

- 명령 실행 중 새 명령을 보내면 이전 명령이 취소되지 않음
- 에러 발생 시 수동으로 로봇을 재설정해야 함
- 충돌 감지 기능 없음 (안전 거리 유지 필요)

### 문제 해결

#### 서비스 연결 실패
```
Failed to call service /dsr01a0912/motion/move_joint
```
**해결**: 두산 드라이버가 실행 중인지 확인
```bash
rosnode list | grep dsr01a0912
rosservice list | grep move_joint
```

#### 상태가 ERROR로 변경됨
**원인**: 로봇이 목표 자세에 도달할 수 없음 (특이점, 충돌 등)  
**해결**: 티치 펜던트에서 수동으로 안전한 자세로 이동 후 재시도

---

## 🔧 빌드 및 설치

### 의존성

```xml
<!-- package.xml -->
<depend>roscpp</depend>
<depend>std_msgs</depend>
<depend>dsr_msgs</depend>
```

### 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📝 라이선스 및 저작권

**Copyright © 2025 KATECH (Korea Automotive Technology Institute)**  
**Smart Manufacturing Technology Research Center**

**Author**: LDJ (Dongjun Lee)  
**Email**: djlee2@katech.re.kr

---

## 🔗 관련 문서

- [메인 README](../../README.md) - 프로젝트 개요
- [중앙 관제 노드](../central_coordinator/README.md) - 통합 제어 시스템
- [모바일 로봇 제어](../mobile_robot_control/README.md) - 모바일 로봇 패키지

---

**Doosan Helper Package for Mobile-Cobot Integrated Control System**  
**Built by KATECH Smart Manufacturing Technology Research Center**

