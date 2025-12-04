# 모바일-협동로봇 협업 시스템

## 📋 개요

이 시스템은 모바일 로봇(TR200)과 두산 협동로봇 간의 순차적 협업을 구현합니다.

### 동작 시퀀스

```
[모바일 로봇]
    ↓ 1. 특정 거리 이동
    ↓ 2. 이동 완료 → /katech/mobile_ready 발행
    
[코디네이터]
    ↓ 3. 모바일 로봇 완전 정지 확인
    ↓ 4. /katech/robot_command 발행 (data=0)
    
[협동로봇]
    ↓ 5. 지정된 자세로 이동
    ↓ 6. 동작 완료
    
[코디네이터]
    ↓ 7. 동작 완료 확인
    ↓ 8. /katech/cobot_done 발행
    
[모바일 로봇]
    ↓ 9. 완료 메시지 수신
    ↓ 10. 협동로봇 관절 상태 출력
```

## 🔧 시스템 구성

### 노드 구성

1. **mobile_robot_control_node.py** (Python)
   - 모바일 로봇 이동 제어
   - 이동 완료 시 협동로봇에 알림
   - 협동로봇 완료 대기 및 관절 상태 출력

2. **cobot_coordinator_node** (C++)
   - 협업 시퀀스 조율
   - 모바일 로봇 정지 확인
   - 협동로봇 동작 트리거
   - 협동로봇 완료 확인

3. **move_robot_node** (C++)
   - 협동로봇 모션 제어
   - `/katech/robot_command` 토픽 수신
   - 지정된 자세로 이동

### 토픽 구조

| 토픽 이름 | 메시지 타입 | 발행자 | 구독자 | 설명 |
|----------|-----------|--------|--------|------|
| `/katech/mobile_ready` | `std_msgs/Bool` | mobile_robot_control | cobot_coordinator | 모바일 이동 완료 |
| `/katech/cobot_done` | `std_msgs/Bool` | cobot_coordinator | mobile_robot_control | 협동로봇 완료 |
| `/katech/robot_command` | `std_msgs/Int32` | cobot_coordinator | move_robot_node | 협동로봇 명령 |
| `/woosh/twist` | `geometry_msgs/Twist` | woosh_robot | cobot_coordinator | 모바일 속도 모니터링 |
| `/dsr01a0912/joint_states` | `sensor_msgs/JointState` | doosan_robot | cobot_coordinator, mobile_robot_control | 관절 상태 |

## 🚀 사용 방법

### 1단계: 빌드

```bash
cd /home/katech/robot_ws
catkin_make
source devel/setup.bash
```

### 2단계: 협동로봇 시스템 실행

```bash
# 터미널 1: 협업 시스템 노드 실행
roslaunch doosan_helper mobile_cobot_collaboration.launch
```

이 명령은 다음 노드들을 실행합니다:
- `move_robot_node`: 협동로봇 제어
- `cobot_coordinator_node`: 협업 코디네이터

### 3단계: 모바일 로봇 실행

```bash
# 터미널 2: 모바일 로봇 제어 노드 실행
cd /home/katech/robot_ws/src/mobile_robot_control/src
python3 mobile_robot_control_node.py --distance 1.0 --speed 0.2
```

### 예상 출력

#### 터미널 2 (모바일 로봇):
```
🤖 Mobile Robot Controller 초기화 완료
   연결 대상: 169.254.128.2:5480
📡 토픽 통신 준비 완료
   - 발행: /katech/mobile_ready
   - 구독: /katech/cobot_done
   - 구독: /dsr01a0912/joint_states
✅ 로봇 연결 성공!
🔋 배터리 잔량: 85%
📍 초기 위치: X=0.000, Y=0.000, Theta=0.000

🎯 정밀 이동 모드 (Odometry 피드백)
=============================================================
🎯 목표: 1.000m 이동 (최대 속도: 0.20m/s)
   가속 구간: 0.15m | 감속 구간: 0.20m
   제어 주기: 20Hz (50ms마다 명령 전송)
=============================================================
📍 시작 위치: X=0.000, Y=0.000
🚀 이동 시작!
📊 🚀 가속 | 0.078m / 1.000m | 속도: 0.11m/s | 남은: 0.922m
📊 ⚡ 등속 | 0.523m / 1.000m | 속도: 0.20m/s | 남은: 0.477m
📊 🛑 감속 | 0.853m / 1.000m | 속도: 0.15m/s | 남은: 0.147m
✅ 목표 거리 도달! (실제: 1.002m)
🛑 정지 중...
🛑 정지 완료

=============================================================
📊 최종 결과:
   목표 거리: 1.000m
   실제 거리: 1.002m
   오차: 0.002m (0.2%)
   소요 시간: 8.3초
=============================================================

=============================================================
🤝 협업 시퀀스 시작
=============================================================
📢 협동로봇에 이동 완료 알림 발행...
✅ 메시지 발행 완료: /katech/mobile_ready
⏳ 협동로봇 작업 완료 대기 중...
✅ 협동로봇 작업 완료 메시지 수신!
✅ 협동로봇 작업 완료 확인됨
🔍 협동로봇 관절 상태 확인 중...
=============================================================
🤖 협동로봇 현재 관절 상태:
   관절 이름: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
   관절 위치 (도):
     joint1: 90.00°
     joint2: 0.00°
     joint3: 90.00°
     joint4: 0.00°
     joint5: 90.00°
     joint6: -90.00°
=============================================================

=============================================================
🎉 전체 협업 시퀀스 완료!
=============================================================

✅ 작업 완료! Ctrl+C로 종료하세요.
```

#### 터미널 1 (협동로봇 시스템):
```
==================================================
🤝 Cobot Coordinator Node 시작
==================================================
파라미터:
  - 속도 임계값: 0.010 m/s
  - 위치 허용 오차: 0.020 rad (1.1도)
  - 안정화 시간: 1.0초
  - 로봇 명령: 0
==================================================
토픽:
  - 구독: /katech/mobile_ready
  - 구독: /woosh/twist
  - 구독: /dsr01a0912/joint_states
  - 발행: /katech/robot_command
  - 발행: /katech/cobot_done
==================================================
⏳ 모바일 로봇 이동 완료 대기 중...

==================================================
✅ 모바일 로봇 이동 완료 메시지 수신!
==================================================
🔍 모바일 로봇 정지 상태 확인 중...
   (1.0초 동안 속도 < 0.010 m/s 유지 필요)
   현재 속도: 0.000 m/s | 경과: 0.0초
✅ 모바일 로봇 완전 정지 확인! (1.1초 경과)

==================================================
🤖 협동로봇 동작 시작
==================================================
명령 전송: /katech/robot_command = 0
✅ 명령 발행 완료
⏳ 협동로봇 동작 완료 대기 중...
   동작 진행 중... (5.0초 경과)
✅ 협동로봇 목표 자세 도달! (7.3초 경과)

==================================================
📢 모바일 로봇에 작업 완료 알림 전송
==================================================
✅ 완료 메시지 발행: /katech/cobot_done

==================================================
⏳ 다음 협업 시퀀스 대기 중...
==================================================
```

## ⚙️ 파라미터 설정

launch 파일에서 다음 파라미터를 조정할 수 있습니다:

```xml
<arg name="velocity_threshold" default="0.01" />
<!-- 모바일 로봇 정지 판단 임계값 (m/s) -->

<arg name="position_tolerance" default="0.02" />
<!-- 협동로봇 관절 위치 허용 오차 (rad, 약 1도) -->

<arg name="stability_duration" default="1.0" />
<!-- 정지 안정화 시간 (초) -->

<arg name="robot_command" default="0" />
<!-- 협동로봇 명령: 0, 1, 99(home) -->
```

사용 예시:
```bash
roslaunch doosan_helper mobile_cobot_collaboration.launch \
    velocity_threshold:=0.005 \
    robot_command:=1
```

## 🐛 문제 해결

### 1. "로봇 연결 실패"
- 모바일 로봇 IP가 올바른지 확인: `169.254.128.2`
- 네트워크 연결 확인: `ping 169.254.128.2`

### 2. "협동로봇 응답 없음"
- 두산 로봇 컨트롤러가 실행 중인지 확인
- 토픽 확인: `rostopic list | grep dsr01`
- move_robot_node가 실행 중인지 확인

### 3. "타임아웃 발생"
- 파라미터 조정: `stability_duration` 값 증가
- 로그 확인: 어느 단계에서 멈췄는지 파악

### 4. "관절 상태 출력 안됨"
- JointState 토픽 확인: `rostopic echo /dsr01a0912/joint_states`
- 협동로봇이 동작 완료했는지 확인

## 📊 토픽 모니터링

```bash
# 모든 협업 관련 토픽 확인
rostopic list | grep katech

# 모바일 준비 메시지 모니터링
rostopic echo /katech/mobile_ready

# 협동로봇 완료 메시지 모니터링
rostopic echo /katech/cobot_done

# 협동로봇 명령 모니터링
rostopic echo /katech/robot_command

# 관절 상태 확인
rostopic echo /dsr01a0912/joint_states
```

## 📝 커스터마이징

### 다른 협동로봇 자세 사용

`cobot_coordinator_node.cpp`에서 목표 관절 자세 수정:

```cpp
// 목표 관절 자세 (data=0에 대응하는 자세)
target_joint_positions_ = {
    90.0 * M_PI / 180.0,   // Joint 0: 90도
    0.0,                    // Joint 1: 0도
    90.0 * M_PI / 180.0,   // Joint 2: 90도
    0.0,                    // Joint 3: 0도
    90.0 * M_PI / 180.0,   // Joint 4: 90도
    -90.0 * M_PI / 180.0   // Joint 5: -90도
};
```

### 다른 이동 거리 설정

모바일 로봇 실행 시 파라미터 변경:

```bash
python3 mobile_robot_control_node.py \
    --distance 2.0 \
    --speed 0.3 \
    --accel 0.2 \
    --decel 0.25
```

## 👨‍💻 개발자 정보

- **Author**: KATECH Robotics Team
- **License**: MIT
- **Contact**: zinith7@naver.com

## 🔗 관련 문서

- [모바일 로봇 제어 가이드](../../docs/mobile_robot/)
- [두산 로봇 API 문서](../doosan-robot/README.md)

