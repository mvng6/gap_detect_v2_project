# Mobile-Cobot Integrated Control System

**모바일 로봇과 협동 로봇의 통합 제어 시스템**

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-KATECH-orange)](https://www.katech.re.kr)

---

## 📋 프로젝트 개요

본 프로젝트는 **모바일 로봇**과 **두산 협동 로봇(Doosan A0912)**을 중앙 관제 시스템을 통해 순차적으로 제어하는 통합 시스템입니다. ROS 1 Noetic 기반으로 Topic 통신을 활용하여 두 로봇의 협업 작업을 수행합니다.

### 주요 기능

- ✅ **중앙 관제 시스템**: 단일 노드에서 두 로봇을 순차적으로 제어
- ✅ **자동 초기화**: 로봇 연결 및 서보 온 상태 자동 감지
- ✅ **왕복 시퀀스**: 모바일 전진 → 협동로봇 작업 → 모바일 복귀 → 협동로봇 홈
- ✅ **상태 모니터링**: 실시간 로봇 상태 확인 및 에러 처리
- ✅ **파라미터 제어**: 이동 거리, 속도 등 런타임 설정 가능

---

## 🏗️ 시스템 구조

```
┌─────────────────────────────────────────────────┐
│           Central Coordinator Node              │
│         (중앙 관제 노드)                          │
│                                                 │
│  [초기화] 두산 로봇 STANDBY 확인 → 홈 위치        │
│  [사이클]                                        │
│    1. 모바일 전진 (0.3m)                         │
│    2. 두산 작업 자세                              │
│    3. 모바일 후진 (복귀)                          │
│    4. 두산 홈 위치                                │
└─────────────────────────────────────────────────┘
         │                            │
         ↓ (Topic)                    ↓ (Topic)
    /mobile/cmd                  /katech/robot_command
         │                            │
┌────────────────────┐      ┌───────────────────────┐
│ Mobile Robot Node  │      │ Doosan Robot Node     │
│                    │      │                       │
│ - 비동기 제어       │      │ - 서비스 클라이언트    │
│ - Odometry 피드백   │      │ - 상태 발행           │
└────────────────────┘      └───────────────────────┘
         ↑                            ↑
    /mobile/status             /doosan/status
         └────────────┬───────────────┘
                      │
              (상태 구독)
```

---

## 🚀 빠른 시작

### 1. 환경 요구사항

- **Host OS**: Ubuntu 24.04 LTS
- **ROS 환경**: Docker + ROS Noetic
- **로봇 하드웨어**:
  - Doosan A0912 (IP: 192.168.137.100)
  - Mobile Robot (IP: 169.254.128.2)

### 2. 실행

```bash
# Docker 컨테이너 접속
docker exec -it my_noetic_ws bash

# 환경 설정
cd ~/catkin_ws
source devel/setup.bash

# 통합 시스템 실행
roslaunch central_coordinator integrated_system.launch
```

### 3. 파라미터 변경

```bash
# 이동 거리 및 속도 변경
roslaunch central_coordinator integrated_system.launch \
    mobile_distance:=0.5 \
    mobile_speed:=0.3 \
    cycle_delay:=10.0
```

---

## 📦 패키지 구조

```
robot_ws/
├── src/
│   ├── central_coordinator/    # 중앙 관제 노드
│   │   ├── src/
│   │   │   └── coordinator_node.py
│   │   └── launch/
│   │       └── integrated_system.launch
│   │
│   ├── doosan_helper/          # 두산 로봇 제어
│   │   ├── src/
│   │   │   ├── move_robot_node.cpp
│   │   │   ├── trigger_home_node.cpp
│   │   │   ├── trigger_one_node.cpp
│   │   │   └── trigger_zero_node.cpp
│   │   └── README.md
│   │
│   └── mobile_robot_control/   # 모바일 로봇 제어
│       ├── src/
│       │   ├── move_mobile_robot_node.py
│       │   └── mobile_robot_twist_control.py
│       └── README.md
│
└── docs/                       # 문서
    ├── ENVIRONMENT_SETUP.md    # 환경 구축 가이드
    └── INTEGRATION_GUIDE.md    # 통합 개발 가이드
```

---

## 📖 문서

### 사용자 가이드
- [환경 구축 가이드](docs/ENVIRONMENT_SETUP.md) - Docker 및 ROS 환경 설정
- [통합 개발 가이드](docs/INTEGRATION_GUIDE.md) - 시스템 개발 및 확장 방법

### 패키지별 문서
- [central_coordinator](src/central_coordinator/README.md) - 중앙 관제 노드
- [doosan_helper](src/doosan_helper/README.md) - 두산 로봇 제어
- [mobile_robot_control](src/mobile_robot_control/README.md) - 모바일 로봇 제어

---

## 🔧 주요 ROS 토픽

| 토픽 이름 | 메시지 타입 | 설명 |
|----------|-----------|------|
| `/mobile/cmd` | `std_msgs/Float64MultiArray` | 모바일 로봇 이동 명령 [distance, speed] |
| `/mobile/status` | `std_msgs/String` | 모바일 로봇 상태 (IDLE, MOVING, COMPLETED, ERROR) |
| `/katech/robot_command` | `std_msgs/Int32` | 두산 로봇 자세 명령 (0, 1, 99) |
| `/doosan/status` | `std_msgs/String` | 두산 로봇 상태 (IDLE, MOVING, COMPLETED, ERROR) |
| `/dsr01a0912/state` | `dsr_msgs/RobotState` | 두산 로봇 시스템 상태 (드라이버 발행) |

---

## 🔋 유틸리티

### 배터리 잔량 확인

```bash
# 모바일 로봇 배터리 확인
rosrun mobile_robot_control battery_check.py
```

출력: 배터리 잔량을 색상 막대 그래프로 표시 (80%↑: 초록, 50~79%: 청록, 20~49%: 노란색, 20%↓: 빨강)

---

## 🛠️ 개발 환경

### 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 디버깅
```bash
# 상태 모니터링
rostopic echo /mobile/status
rostopic echo /doosan/status

# 수동 명령 테스트
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.3, 0.2]"
rostopic pub /katech/robot_command std_msgs/Int32 "data: 99"

# 노드 그래프 확인
rqt_graph
```

---

## 🐛 문제 해결

### 두산 로봇이 SAFE_OFF 상태에서 멈춤
```
⚠️ 티치 펜던트에서 '서보 온(Servo On)' 버튼을 눌러주세요
```

### 모바일 로봇 연결 실패
```bash
# IP 확인
ping 169.254.128.2

# 로그 확인
rosnode info /mobile_robot_ros_node
```

---

## 📝 라이선스 및 저작권

**Copyright © 2025 KATECH (Korea Automotive Technology Institute)**  
**Smart Manufacturing Technology Research Center**

**Author**: LDJ (Dongjun Lee)  
**Email**: djlee2@katech.re.kr

본 소프트웨어는 한국자동차연구원 스마트제조기술연구센터에서 개발되었습니다.  
상업적 사용 및 재배포 시 사전 승인이 필요합니다.

---

## 🤝 기여

본 프로젝트는 KATECH 내부 프로젝트입니다.  
문의 사항은 djlee2@katech.re.kr로 연락주시기 바랍니다.

---

## 📈 버전 히스토리

### v1.0.0 (2025-11-03)
- ✅ 중앙 관제 시스템 구축 완료
- ✅ 모바일-협동로봇 왕복 시퀀스 구현
- ✅ 자동 초기화 및 상태 감지 기능
- ✅ ROS Topic 기반 통신 구조 확립

---

**Built with ❤️ by KATECH Smart Manufacturing Technology Research Center**
