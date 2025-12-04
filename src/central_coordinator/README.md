# Central Coordinator Package

**중앙 관제 노드 패키지**

---

## 📋 패키지 정보

**패키지명**: `central_coordinator`  
**작성자**: LDJ (Dongjun Lee)  
**이메일**: djlee2@katech.re.kr  
**소속**: KATECH 스마트제조기술연구센터  
**최종 수정일**: 2025-11-03  
**버전**: 1.0.0

---

## 📝 개요

`central_coordinator`는 모바일 로봇과 두산 협동 로봇을 통합 제어하는 중앙 관제 시스템 패키지입니다. ROS Topic 기반 통신을 통해 두 로봇의 순차적 협업 작업을 관리하며, 자동 초기화 및 상태 모니터링 기능을 제공합니다.

### 주요 기능

- ✅ **자동 초기화**: 두산 로봇 STANDBY 상태 자동 감지 및 홈 위치 이동
- ✅ **순차 제어**: 모바일 전진 → 협동로봇 작업 → 모바일 복귀 → 협동로봇 홈
- ✅ **상태 감지**: 실시간 로봇 상태 모니터링 및 에러 처리
- ✅ **파라미터 제어**: 런타임 시 이동 거리, 속도 등 조정 가능
- ✅ **사이클 관리**: 무한 반복 또는 지정 횟수 실행

---

## 🏗️ 아키텍처

### 노드 구조

```python
CentralCoordinator
├── __init__()              # 초기화 및 ROS 설정
├── initialize_robots()     # 로봇 초기화 시퀀스
│   ├── wait_for_doosan_ready()  # 두산 STANDBY 대기
│   └── send_doosan_command(99)  # 홈 위치 이동
└── run_sequence()          # 메인 사이클 실행
    ├── send_mobile_command()    # 모바일 전진
    ├── wait_for_status()        # 완료 대기
    ├── send_doosan_command(1)   # 협동로봇 작업
    ├── send_mobile_command()    # 모바일 후진
    └── send_doosan_command(99)  # 협동로봇 홈
```

### 통신 구조

```
[CentralCoordinator]
    │
    ├──> /mobile/cmd (발행)
    │    └── std_msgs/Float64MultiArray: [distance, speed]
    │
    ├──> /katech/robot_command (발행)
    │    └── std_msgs/Int32: 자세 명령
    │
    ├──< /mobile/status (구독)
    │    └── std_msgs/String: IDLE, MOVING, COMPLETED, ERROR
    │
    ├──< /doosan/status (구독)
    │    └── std_msgs/String: IDLE, MOVING, COMPLETED, ERROR
    │
    └──< /dsr01a0912/state (구독)
         └── dsr_msgs/RobotState: 로봇 시스템 상태
```

---

## 📁 파일 구조

```
central_coordinator/
├── CMakeLists.txt          # 빌드 설정
├── package.xml             # 패키지 메타데이터
├── README.md               # 본 문서
├── src/
│   └── coordinator_node.py # 중앙 관제 노드 (메인)
├── launch/
│   └── integrated_system.launch  # 통합 시스템 실행
└── include/                # (비어있음 - Python 패키지)
```

---

## 🚀 사용 방법

### 1. 기본 실행

```bash
# 환경 설정
source ~/catkin_ws/devel/setup.bash

# 실행
roslaunch central_coordinator integrated_system.launch
```

### 2. 파라미터 변경

```bash
# 이동 거리 변경 (기본: 0.3m)
roslaunch central_coordinator integrated_system.launch mobile_distance:=0.5

# 속도 변경 (기본: 0.2m/s)
roslaunch central_coordinator integrated_system.launch mobile_speed:=0.3

# 사이클 간 대기 시간 변경 (기본: 5.0초)
roslaunch central_coordinator integrated_system.launch cycle_delay:=10.0

# 복합 설정
roslaunch central_coordinator integrated_system.launch \
    mobile_distance:=0.5 \
    mobile_speed:=0.3 \
    cycle_delay:=10.0
```

---

## ⚙️ 파라미터 설명

### Launch 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `doosan_model` | string | `a0912` | 두산 로봇 모델명 |
| `doosan_host` | string | `192.168.137.100` | 두산 컨트롤러 IP |
| `mobile_distance` | float | `0.3` | 모바일 이동 거리 (m) |
| `mobile_speed` | float | `0.2` | 모바일 이동 속도 (m/s) |
| `cycle_delay` | float | `5.0` | 사이클 간 대기 시간 (초) |

### 노드 파라미터

노드 실행 시 `~` 네임스페이스로 전달됩니다:
- `~mobile_distance`: 모바일 로봇 이동 거리
- `~mobile_speed`: 모바일 로봇 속도
- `~cycle_delay`: 사이클 간 대기 시간

---

## 📡 ROS 인터페이스

### Published Topics

| 토픽 | 타입 | 주기 | 설명 |
|-----|------|-----|------|
| `/mobile/cmd` | `std_msgs/Float64MultiArray` | Event | 모바일 로봇 이동 명령 |
| `/katech/robot_command` | `std_msgs/Int32` | Event | 두산 로봇 자세 명령 |

### Subscribed Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/mobile/status` | `std_msgs/String` | 모바일 로봇 상태 |
| `/doosan/status` | `std_msgs/String` | 두산 로봇 동작 상태 |
| `/dsr01a0912/state` | `dsr_msgs/RobotState` | 두산 로봇 시스템 상태 |

---

## 🔄 시퀀스 설명

### 초기화 단계

```
[START]
  │
  ├─> 두산 로봇 상태 구독 시작
  │
  ├─> STANDBY 상태 대기 (최대 60초)
  │   ├─ SAFE_OFF 감지 시 → 서보 온 안내 메시지 출력
  │   └─ 타임아웃 시 → 에러 및 종료
  │
  ├─> 두산 로봇 홈 위치 이동 (명령 99)
  │
  └─> [초기화 완료]
```

### 메인 사이클

```
[CYCLE START]
  │
  ├─> [1/4] 모바일 전진
  │   ├─ 명령: /mobile/cmd [distance, speed]
  │   └─ 대기: /mobile/status == "COMPLETED"
  │
  ├─> [2/4] 두산 작업 자세
  │   ├─ 명령: /katech/robot_command = 1
  │   └─ 대기: /doosan/status == "COMPLETED"
  │
  ├─> [3/4] 모바일 후진 (복귀)
  │   ├─ 명령: /mobile/cmd [distance, -speed]
  │   └─ 대기: /mobile/status == "COMPLETED"
  │
  ├─> [4/4] 두산 홈 위치
  │   ├─ 명령: /katech/robot_command = 99
  │   └─ 대기: /doosan/status == "COMPLETED"
  │
  ├─> [CYCLE COMPLETE]
  │
  └─> cycle_delay 초 대기 → [CYCLE START]
```

---

## 🐛 디버깅

### 로그 확인

```bash
# ROS 로그 확인
rosnode info /central_coordinator

# 실시간 로그 확인
rostopic echo /rosout | grep central_coordinator
```

### 수동 테스트

```bash
# 모바일 로봇 수동 명령
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.3, 0.2]"

# 두산 로봇 수동 명령
rostopic pub /katech/robot_command std_msgs/Int32 "data: 99"

# 상태 확인
rostopic echo /mobile/status
rostopic echo /doosan/status
```

---

## ⚠️ 주의사항

### 안전 수칙

1. **작업 공간 확보**: 로봇 이동 경로에 장애물이 없는지 확인
2. **비상 정지 준비**: 비상 시 즉시 정지 버튼을 누를 수 있도록 준비
3. **서보 온 확인**: 두산 로봇이 STANDBY 상태인지 확인

### 문제 해결

#### 초기화 실패 (SAFE_OFF)
```
⚠️ 티치 펜던트에서 '서보 온' 버튼을 눌러주세요
```

#### 모바일 로봇 응답 없음
```bash
# 노드 상태 확인
rosnode list | grep mobile_robot_ros_node

# 토픽 발행 확인
rostopic hz /mobile/status
```

#### 두산 로봇 응답 없음
```bash
# 드라이버 실행 확인
rosnode list | grep dsr01a0912

# 서비스 확인
rosservice list | grep move_joint
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
- [환경 구축 가이드](../../docs/ENVIRONMENT_SETUP.md) - 개발 환경 설정
- [통합 개발 가이드](../../docs/INTEGRATION_GUIDE.md) - 시스템 개발 방법
- [doosan_helper](../doosan_helper/README.md) - 두산 로봇 제어 패키지
- [mobile_robot_control](../mobile_robot_control/README.md) - 모바일 로봇 제어 패키지

---

**Central Coordinator Package for Mobile-Cobot Integrated Control System**  
**Built by KATECH Smart Manufacturing Technology Research Center**

