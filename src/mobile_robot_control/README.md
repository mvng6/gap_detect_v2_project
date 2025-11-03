# Mobile Robot Control Package

**모바일 로봇 제어 패키지**

---

## 📋 패키지 정보

**패키지명**: `mobile_robot_control`  
**작성자**: LDJ (Dongjun Lee)  
**이메일**: djlee2@katech.re.kr  
**소속**: KATECH 스마트제조기술연구센터  
**최종 수정일**: 2025-11-03  
**버전**: 1.0.0

---

## 📝 개요

`mobile_robot_control`은 모바일 로봇의 정밀한 거리 제어를 위한 ROS 패키지입니다. Twist 방식의 속도 제어와 Odometry 피드백을 결합하여 목표 거리까지 정확하게 이동하며, 가감속 프로파일을 적용하여 부드러운 동작을 구현합니다.

### 주요 기능

- ✅ **정밀 거리 제어**: Odometry 피드백 기반 목표 거리 도달
- ✅ **가감속 프로파일**: 부드러운 출발과 정지
- ✅ **ROS Topic 통합**: 명령 수신 및 상태 발행
- ✅ **비동기 제어**: asyncio 기반 non-blocking 동작
- ✅ **에러 처리**: 연결 실패, 타임아웃 등 자동 처리

---

## 🏗️ 패키지 구조

### 노드 목록

| 노드명 | 타입 | 파일 | 역할 |
|-------|------|------|------|
| `mobile_robot_ros_node` | Server | `move_mobile_robot_node.py` | ROS Topic 통합 제어 노드 |
| `battery_check` | Utility | `battery_check.py` | 배터리 잔량 확인 도구 |
| (내부 모듈) | Library | `mobile_robot_twist_control.py` | 모바일 로봇 SDK 제어 클래스 |

### 제어 흐름

```
[ROS Topic 명령] 
    ↓
[MobileRobotROSNode]
    ├─> threading.Thread (비동기 처리)
    └─> asyncio.run
        ↓
[MobileRobotTwistController]
    ├─> connect() - 로봇 연결
    ├─> move_distance() - 거리 제어
    │   ├─> 가속 구간
    │   ├─> 등속 구간  
    │   └─> 감속 구간
    └─> stop() - 연결 종료
```

---

## 📁 파일 구조

```
mobile_robot_control/
├── CMakeLists.txt                      # 빌드 설정
├── package.xml                         # 패키지 메타데이터
├── README.md                           # 본 문서
└── src/
    ├── move_mobile_robot_node.py       # ROS 통합 노드 ⭐
    ├── mobile_robot_twist_control.py   # SDK 제어 모듈
    └── battery_check.py                # 배터리 체크 도구
```

---

## 🚀 사용 방법

### 1. ROS 노드 실행

```bash
# 환경 설정
source ~/catkin_ws/devel/setup.bash

# 노드 실행 (명령 대기 모드)
rosrun mobile_robot_control move_mobile_robot_node.py
```

### 2. Topic으로 제어

```bash
# 전진: 0.3m를 0.2m/s로 이동
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.3, 0.2]"

# 후진: 0.3m를 -0.2m/s로 이동
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.3, -0.2]"

# 상태 확인
rostopic echo /mobile/status
```

### 3. 독립 실행 (ROS 없이)

```bash
# 직접 실행 (원샷 모드)
cd ~/catkin_ws/src/mobile_robot_control/src
python3 mobile_robot_twist_control.py --distance 0.5 --speed 0.2
```

---

## 📡 ROS 인터페이스

### move_mobile_robot_node

#### Subscribed Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/mobile/cmd` | `std_msgs/Float64MultiArray` | 이동 명령 `[distance, speed]` |

**메시지 형식**:
```python
data: [distance, speed]
# distance: 이동 거리 (m, 양수=전진, 음수는 speed를 음수로)
# speed: 이동 속도 (m/s, 음수=후진)
```

#### Published Topics

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/mobile/status` | `std_msgs/String` | 로봇 상태 |

**상태 값**:
- `IDLE`: 대기 중
- `MOVING`: 이동 중
- `COMPLETED`: 이동 완료
- `ERROR`: 오류 발생

---

## ⚙️ 주요 파라미터

### 로봇 연결 설정

```python
# mobile_robot_twist_control.py 내부
self.robot_ip = '169.254.128.2'      # 모바일 로봇 IP
self.robot_port = 5480               # 포트
self.robot_identity = 'twist_controller'
```

### 가감속 프로파일

```python
# move_distance() 함수 파라미터
accel_distance = 0.15   # 가속 구간 거리 (m)
decel_distance = 0.2    # 감속 구간 거리 (m)
```

### 제어 파라미터

```python
distance_tolerance = 0.01    # 거리 허용 오차 (m)
velocity_scale_accel = 0.3   # 가속 시작 속도 비율
velocity_scale_decel = 0.5   # 감속 시작 속도 비율
timeout = 60.0               # 타임아웃 (초)
```

---

## 🔄 동작 알고리즘

### 1. 가감속 프로파일

```
속도
 ^
 │     등속 구간
 │   ┌─────────┐
 │  ╱           ╲
 │ ╱ 가속        ╲ 감속
 │╱               ╲
 └────────────────────> 거리
   <─accel─><───><─decel─>
```

### 2. 상태 전이

```
[IDLE]
  │ 명령 수신
  ↓
[MOVING]
  ├─> connect()
  ├─> move_distance()
  │   ├─> 가속 (0.3 × speed → speed)
  │   ├─> 등속 (speed)
  │   └─> 감속 (speed → 0.5 × speed → 0)
  └─> stop()
  ↓
[COMPLETED] or [ERROR]
  ↓
[IDLE]
```

### 3. 거리 제어 로직

```python
async def move_distance(target_distance, speed):
    current_distance = 0
    
    while current_distance < target_distance:
        # 남은 거리 계산
        remaining = target_distance - current_distance
        
        # 속도 프로파일 적용
        if current_distance < accel_distance:
            # 가속 구간
            velocity = speed * (0.3 + 0.7 * current_distance / accel_distance)
        elif remaining < decel_distance:
            # 감속 구간
            velocity = speed * max(0.5, remaining / decel_distance)
        else:
            # 등속 구간
            velocity = speed
        
        # Twist 명령 전송
        await robot.set_wheel_velocity(velocity)
        
        # Odometry 업데이트
        current_distance = odometry.distance
```

---

## 🐛 디버깅

### 로그 확인

```bash
# ROS 노드 정보
rosnode info /mobile_robot_ros_node

# 상태 실시간 확인
rostopic echo /mobile/status

# 명령 모니터링
rostopic echo /mobile/cmd

# 상세 로그 (노드 실행 시)
rosrun mobile_robot_control move_mobile_robot_node.py __log_level:=debug
```

### 직접 SDK 테스트

```bash
# Python 인터프리터
cd ~/catkin_ws/src/mobile_robot_control/src
python3

>>> from mobile_robot_twist_control import MobileRobotTwistController
>>> import asyncio
>>> controller = MobileRobotTwistController()
>>> asyncio.run(controller.connect())
>>> # 테스트...
>>> asyncio.run(controller.stop())
```

### 배터리 잔량 확인

```bash
# ROS 명령어로 배터리 체크
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
- **80% 이상**: 🔋 초록색 (충분)
- **50~79%**: 🔋 청록색 (보통)
- **20~49%**: 🪫 노란색 (주의)
- **20% 미만**: 🪫 빨간색 (위험)

---

## ⚠️ 주의사항

### 안전 수칙

1. **작업 공간**: 모바일 로봇 이동 경로에 장애물이 없는지 확인
2. **속도 제한**: 처음 테스트 시 낮은 속도(0.1 m/s)로 시작
3. **거리 제한**: 작은 거리(0.1 m)부터 테스트
4. **비상 정지**: 비상 시 로봇 전원을 끌 수 있도록 준비

### 알려진 제한사항

- 동시에 여러 명령을 보내면 이전 명령이 무시됨
- Wi-Fi 연결 불안정 시 제어 지연 발생 가능
- 장애물 감지 기능 없음 (수동 확인 필요)
- 배터리 부족 시 속도가 제한될 수 있음

### 문제 해결

#### 연결 실패
```
ERROR: 로봇 연결 실패: Connection refused
```
**해결**:
```bash
# 로봇 IP 확인
ping 169.254.128.2

# 로봇 전원 및 Wi-Fi 연결 확인
```

#### Import 오류
```
ImportError: cannot import name 'MobileRobotTwistController'
```
**해결**: 이미 `move_mobile_robot_node.py`에서 `importlib`로 해결됨

#### 이동 거리 부정확
**원인**: Odometry 드리프트, 바닥 미끄러짐  
**해결**: 가감속 파라미터 조정 또는 바닥 상태 개선

---

## 🔧 커스터마이징

### 속도 프로파일 변경

`mobile_robot_twist_control.py` 수정:

```python
# 가속 시작 속도 비율 변경 (기본: 0.3)
velocity_scale_accel = 0.5  # 더 빠른 출발

# 감속 최소 속도 비율 변경 (기본: 0.5)
velocity_scale_decel = 0.3  # 더 부드러운 정지
```

### 로봇 IP 변경

```python
# ROS 파라미터로 설정
rospy.set_param('/mobile_robot_ros_node/robot_ip', '192.168.1.100')
```

또는 코드 직접 수정:

```python
# mobile_robot_twist_control.py
self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
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
- [두산 로봇 제어](../doosan_helper/README.md) - 협동로봇 패키지

---

**Mobile Robot Control Package for Mobile-Cobot Integrated Control System**  
**Built by KATECH Smart Manufacturing Technology Research Center**

