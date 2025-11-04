# 중앙 관제 노드 학습 가이드: `coordinator_node.py`

## 📚 학습 목표
- 중앙 관제 노드가 전체 협업 사이클을 어떻게 오케스트레이션 하는지 이해한다.
- 주요 함수(초기화, 대기, 명령 전송, 메인 루프)의 동작 원리를 익힌다.
- ROS 토픽/서비스 흐름을 코드와 연결해 설명할 수 있다.
- 직접 토픽을 발행하며 로직을 검증할 수 있는 연습 문제를 수행한다.

## 1. 파일 개요
- 위치: `src/central_coordinator/src/coordinator_node.py`
- 역할: 두산 협동 로봇과 모바일 로봇을 순차적으로 제어하며 협업 작업을 반복 실행하는 상위 제어 노드
- 통신 개요:
  - 발행 토픽: `/mobile/cmd`, `/dsr_robot/robot_cmd`
  - 구독 토픽: `/mobile/status`, `/doosan/status`, `/dsr01a0912/state`
  - 사용 메시지: `std_msgs/String`, `std_msgs/Int32`, `std_msgs/Float64MultiArray`, `dsr_msgs/RobotState`

## 2. 클래스 구조 살펴보기

```12:345:src/central_coordinator/src/coordinator_node.py
class CentralCoordinator:
    def __init__(self):
        ...
    def mobile_status_callback(self, msg):
        ...
    def doosan_status_callback(self, msg):
        ...
    def doosan_robot_state_callback(self, msg):
        ...
    def wait_for_doosan_ready(self, timeout=60.0):
        ...
    def wait_for_status(self, robot_name, target_status, timeout=60.0):
        ...
    def send_mobile_command(self, distance, speed):
        ...
    def send_doosan_command(self, command_id):
        ...
    def initialize_robots(self):
        ...
    def run_sequence(self):
        ...
```

### 핵심 모듈
1. **초기화 구문(`__init__`)**: 노드 초기화, 퍼블리셔/서브스크립션 생성, 파라미터 로드, 상태 변수 준비
2. **콜백 함수**: 각 로봇 상태 토픽을 수신할 때 상태 변수를 최신으로 유지
3. **대기 함수**: 특정 조건(두산 STANDBY, 상태 토픽 값 등)이 만족될 때까지 폴링
4. **명령 함수**: 모바일 로봇과 두산 로봇에 실제 명령 토픽을 발행
5. **초기화 & 메인 루프**: 협업 시퀀스를 반복 실행하는 고수준 로직

## 3. 초기화 단계 상세

### 3.1 노드 초기화

```52:108:src/central_coordinator/src/coordinator_node.py
rospy.init_node('central_coordinator', anonymous=False)
self.mobile_cmd_pub = rospy.Publisher('/mobile/cmd', Float64MultiArray, queue_size=1)
self.doosan_cmd_pub = rospy.Publisher('/dsr_robot/robot_cmd', Int32, queue_size=1)
self.mobile_status_sub = rospy.Subscriber('/mobile/status', String, self.mobile_status_callback)
self.doosan_status_sub = rospy.Subscriber('/doosan/status', String, self.doosan_status_callback)
self.doosan_robot_state_sub = rospy.Subscriber('/dsr01a0912/state', RobotState, self.doosan_robot_state_callback)
self.mobile_distance = rospy.get_param('~mobile_distance', 0.3)
self.mobile_speed = rospy.get_param('~mobile_speed', 0.2)
self.cycle_delay = rospy.get_param('~cycle_delay', 5.0)
```

**설명**
- `rospy.init_node`: ROS 노드 이름을 `central_coordinator`로 고정 (anonymous=False)
- 퍼블리셔 2개: 모바일 명령, 두산 명령용
- 서브스크립션 3개: 모바일 상태, 두산 상태(커스텀), 두산 실제 상태(드라이버)
- 파라미터 세트: 런치 파일에서 전달된 값을 사용하고, 없으면 기본값으로 동작

### 3.2 상태 변수 & 초기 로그
- `self.mobile_status`, `self.doosan_status`, `self.doosan_robot_state` 등 상태 변수로 최신 상태를 저장
- 초기 2초 슬립으로 다른 노드들이 기동할 시간 확보
- 로그로 현재 설정값을 출력해 디버깅 편의 제공

## 4. 상태 콜백 이해

각 콜백은 단순히 상태 변수 업데이트와 로그 출력 역할을 담당한다. 특히 두산 실제 상태 콜백에서는 이전 상태와 비교해 변화가 있을 때만 로그를 남겨 잡음 감소.

```67:85:src/central_coordinator/src/coordinator_node.py
def doosan_robot_state_callback(self, msg):
    self.doosan_robot_state = msg.robot_state
    self.doosan_robot_state_str = msg.robot_state_str
    if hasattr(self, '_last_robot_state') and self._last_robot_state != msg.robot_state:
        rospy.loginfo(f"🤖 두산 시스템 상태: {msg.robot_state} ({msg.robot_state_str})")
    self._last_robot_state = msg.robot_state
```

**핵심 포인트**
- 상태 ID와 문자열을 동시에 저장해 로직과 로그에 활용
- `_last_robot_state`를 사용하여 상태 변경 감지

## 5. 대기 함수 분석

### 5.1 `wait_for_doosan_ready`
- 목표: `/dsr01a0912/state`가 STANDBY(상수 1)일 때까지 대기
- 동작: 5Hz 루프로 상태 확인, SAFE_OFF 상태에서 일정 시간 이상 머무르면 경고, 타임아웃 시 false 반환
- 로그: 경과 시간, 남은 시간, 현재 상태 문자열을 주기적으로 출력

```100:129:src/central_coordinator/src/coordinator_node.py
while not rospy.is_shutdown():
    if self.doosan_robot_state == self.STATE_STANDBY:
        return True
    if self.doosan_robot_state == self.STATE_SAFE_OFF and elapsed > 5.0 and not safe_off_warning_shown:
        rospy.logwarn("⚠️  두산 로봇이 SAFE_OFF 상태입니다!...)
    if elapsed > timeout:
        rospy.logerr("❌ 타임아웃...")
        return False
    rate.sleep()
```

### 5.2 `wait_for_status`
- 모바일/두산 커스텀 상태 토픽을 활용해 명령 완료 여부 판단
- 에러 상태(`ERROR`) 감지 시 즉시 실패 처리
- 타임아웃으로 무한 대기 방지

## 6. 명령 전송 함수

### 6.1 모바일 명령
- `Float64MultiArray` 데이터에 `[distance, speed]` 두 값을 담아 발행
- 0.1초 간격으로 3회 반복 발행하여 신뢰성 향상

```186:208:src/central_coordinator/src/coordinator_node.py
cmd = Float64MultiArray()
cmd.data = [distance, speed]
for _ in range(3):
    self.mobile_cmd_pub.publish(cmd)
    rospy.sleep(0.1)
```

### 6.2 두산 명령
- 정수 명령 ID(0,1,99 등)를 발행
- 명령 ID에 따라 로그용 설명 문자열을 설정
- 동일하게 3회 반복 발행

## 7. 초기화 절차

`initialize_robots()`에서 두 단계 수행:
1. `wait_for_doosan_ready`로 STANDBY 대기 후 성공 시 홈 명령(99) 발행
2. `/doosan/status` 토픽이 `COMPLETED`가 될 때까지 기다림

실패 시 상세 로그와 함께 false 반환 → 메인 루프에서 실행 중단

## 8. 메인 사이클 로직

```248:336:src/central_coordinator/src/coordinator_node.py
while not rospy.is_shutdown():
    # 1) 모바일 전진
    self.send_mobile_command(self.mobile_distance, self.mobile_speed)
    if not self.wait_for_status("mobile", "COMPLETED", timeout=60.0):
        rospy.sleep(10.0)
        continue

    # 2) 두산 작업 자세
    self.send_doosan_command(1)
    if not self.wait_for_status("doosan", "COMPLETED", timeout=60.0):
        rospy.sleep(10.0)
        continue

    # 3) 모바일 후진
    self.send_mobile_command(self.mobile_distance, -self.mobile_speed)
    ...

    # 4) 두산 홈 복귀
    self.send_doosan_command(99)
    ...

    rospy.sleep(self.cycle_delay)
```

**주요 특징**
- 각 단계 실패 시 10초 대기 후 사이클을 처음부터 다시 시도
- 모바일 후진 시 속도를 음수로 반전하여 동일한 거리만큼 되돌림
- 성공 시 사이클 카운터 증가 및 휴지 시간 확보

## 9. 안전/예외 처리 포인트
- 두산 SAFE_OFF 상태 감지 시 사용자 안내 로그 출력 (Servo On 지시)
- 각 대기 함수에 타임아웃 적용 → 무한 루프 방지
- 에러 상태 감지 시 즉시 로그와 함께 실패 처리
- 명령 발행 반복으로 토픽 누락 가능성 최소화

## 10. 실습 과제

### 과제 1: 토픽 흐름 추적
1. Launch 파일 실행 후 아래 명령을 동시에 실행하여 상태 변화를 관찰하세요.
   ```bash
   rostopic echo /mobile/status
   rostopic echo /doosan/status
   rostopic echo /dsr01a0912/state
   ```
2. 로그를 바탕으로 각 단계에서 어떤 상태 값이 출력되는지 시간 순으로 정리하세요.

### 과제 2: 수동 명령 주입 실험
1. 관제 노드 실행 전, 모바일/두산 상태 토픽에 직접 메시지를 발행하여 콜백 로그를 확인하세요.
   ```bash
   rostopic pub /mobile/status std_msgs/String "data: 'COMPLETED'" -1
   rostopic pub /doosan/status std_msgs/String "data: 'COMPLETED'" -1
   ```
2. 관제 노드 실행 후, `/dsr_robot/robot_cmd` 토픽을 모니터링하며 실제 발행되는 명령 ID를 캡처하세요.

### 과제 3: 파라미터 튜닝 시나리오 작성
1. 아래와 같이 런치 파라미터를 변경하여 실행하고, 각 설정이 로봇 동작에 미치는 영향을 기록하세요.
   ```bash
   roslaunch central_coordinator integrated_system.launch \
       mobile_distance:=0.2 \
       mobile_speed:=0.15 \
       cycle_delay:=3.0
   ```
2. 파라미터 조합별로 예상되는 협업 사이클 시간을 계산하고, 실제 로그와 비교하세요.

## 11. 이해도 체크리스트
- [ ] 각 퍼블리셔와 서브스크립션이 담당하는 토픽의 목적을 설명할 수 있다.
- [ ] `wait_for_doosan_ready`와 `wait_for_status`의 차이를 구분할 수 있다.
- [ ] 명령 발행 함수가 반복 발행을 사용하는 이유를 설명할 수 있다.
- [ ] 사이클 실패 시 왜 10초 대기 후 재시도하는지 이해했다.
- [ ] 파라미터 변경이 협업 시나리오에 미치는 영향을 예시로 들 수 있다.

## 12. 다음 학습 단계 예고
- `doosan_helper/move_robot_node.cpp` 분석: 토픽 명령이 실제 `MoveJoint` 서비스로 연결되는 흐름 이해
- 준비사항: C++ ROS 노드 기본 구조, `dsr_msgs::MoveJoint` 서비스 정의 미리 확인

---

**작성자**: AI Assistant  
**최종 수정일**: 2025-01-XX  
**대상 독자**: ROS 초보자


