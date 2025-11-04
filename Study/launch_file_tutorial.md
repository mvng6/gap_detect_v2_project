# ROS Launch 파일 기초 가이드: integrated_system.launch 상세 설명

## 📚 목차
1. [ROS Launch 파일이란?](#1-ros-launch-파일이란)
2. [파일 구조 개요](#2-파일-구조-개요)
3. [섹션별 상세 설명](#3-섹션별-상세-설명)
4. [실행 순서와 데이터 흐름](#4-실행-순서와-데이터-흐름)
5. [실제 실행 방법](#5-실제-실행-방법)

---

## 1. ROS Launch 파일이란?

### 1.1 기본 개념
ROS Launch 파일은 **여러 개의 ROS 노드(프로그램)를 한 번에 실행하기 위한 설정 파일**입니다.

**비유로 이해하기:**
- **일반 실행**: 각 프로그램을 하나씩 터미널에서 수동으로 실행
  ```bash
  # 터미널 1
  rosrun package1 node1.py
  
  # 터미널 2  
  rosrun package2 node2.py
  
  # 터미널 3
  rosrun package3 node3.py
  ```

- **Launch 파일 사용**: 한 번의 명령으로 모든 프로그램을 자동 실행
  ```bash
  roslaunch central_coordinator integrated_system.launch
  ```

### 1.2 Launch 파일의 장점
1. **편의성**: 여러 노드를 한 번에 실행
2. **설정 관리**: 파라미터(IP 주소, 속도 등)를 한 곳에서 관리
3. **재현성**: 항상 동일한 설정으로 실행 가능
4. **문서화**: 어떤 노드가 실행되는지 명확히 기록

---

## 2. 파일 구조 개요

### 2.1 XML 형식
Launch 파일은 XML 형식으로 작성됩니다. 마치 HTML처럼 태그(`<tag>`)를 사용합니다.

```xml
<?xml version="1.0"?>  <!-- XML 버전 선언 -->
<launch>               <!-- Launch 파일의 시작 -->
    <!-- 내용 -->
</launch>              <!-- Launch 파일의 끝 -->
```

### 2.2 integrated_system.launch의 전체 구조

```
integrated_system.launch
│
├─ [1] 파라미터 설정 (arg 태그)
│   ├─ 두산 로봇 설정
│   ├─ 모바일 로봇 설정
│   └─ 사이클 설정
│
├─ [2] 두산 로봇 시스템
│   ├─ 드라이버 실행 (include)
│   └─ 명령 노드 실행 (node)
│
├─ [3] 모바일 로봇 시스템
│   └─ 제어 노드 실행 (node)
│
└─ [4] 중앙 관제 노드
    └─ 관제 노드 실행 (node)
```

---

## 3. 섹션별 상세 설명

### 3.1 파라미터 설정 섹션 (3-14번 줄)

#### 3.1.1 `<arg>` 태그란?
`<arg>`는 **변수처럼 사용할 수 있는 값**을 정의합니다. 실행할 때 값을 바꿀 수 있어요.

```xml
<arg name="변수명" default="기본값" doc="설명"/>
```

**예시:**
```xml
<arg name="doosan_model" default="a0912" doc="두산 로봇 모델명"/>
```

- `name`: 변수 이름 (`doosan_model`)
- `default`: 기본값 (`a0912` - 로봇 모델명)
- `doc`: 설명 (나중에 `roslaunch --help`로 볼 수 있음)

#### 3.1.2 두산 로봇 설정 (6-7번 줄)

```xml
<arg name="doosan_model" default="a0912" doc="두산 로봇 모델명"/>
<arg name="doosan_host" default="192.168.137.100" doc="두산 컨트롤러 IP"/>
```

**설명:**
- `doosan_model`: 사용할 두산 로봇 모델 (기본값: `a0912`)
- `doosan_host`: 두산 로봇 컨트롤러의 IP 주소 (기본값: `192.168.137.100`)

**실제 사용 예시:**
```bash
# 기본값 사용
roslaunch central_coordinator integrated_system.launch

# 다른 모델과 IP로 실행
roslaunch central_coordinator integrated_system.launch \
    doosan_model:=m1013 \
    doosan_host:=192.168.1.100
```

#### 3.1.3 모바일 로봇 설정 (10-11번 줄)

```xml
<arg name="mobile_distance" default="0.3" doc="모바일 이동 거리 (m)"/>
<arg name="mobile_speed" default="0.2" doc="모바일 이동 속도 (m/s)"/>
```

**설명:**
- `mobile_distance`: 모바일 로봇이 한 번에 이동할 거리 (기본값: 0.3미터)
- `mobile_speed`: 모바일 로봇의 이동 속도 (기본값: 초당 0.2미터)

**실제 사용 예시:**
```bash
# 더 빠르고 멀리 이동하도록 설정
roslaunch central_coordinator integrated_system.launch \
    mobile_distance:=0.5 \
    mobile_speed:=0.3
```

#### 3.1.4 사이클 설정 (14번 줄)

```xml
<arg name="cycle_delay" default="5.0" doc="사이클 간 대기 시간 (초)"/>
```

**설명:**
- `cycle_delay`: 한 작업 사이클이 끝난 후 다음 사이클을 시작하기 전 대기 시간 (기본값: 5초)

**왜 필요할까?**
작업 사이클이 끝나면 잠시 쉬어야 다음 작업을 안전하게 시작할 수 있습니다.

---

### 3.2 두산 로봇 시스템 섹션 (17-28번 줄)

#### 3.2.1 `<include>` 태그란?
다른 Launch 파일을 불러와서 실행하는 태그입니다. **다른 파일의 내용을 가져와서 실행**하는 거예요.

```xml
<include file="경로/파일명.launch">
    <arg name="인자명" value="값"/>
</include>
```

#### 3.2.2 두산 드라이버 실행 (20-24번 줄)

```xml
<include file="$(find dsr_launcher)/launch/single_robot.launch">
    <arg name="model" value="$(arg doosan_model)"/>
    <arg name="mode" value="real"/>
    <arg name="host" value="$(arg doosan_host)"/>
</include>
```

**단어별 설명:**
- `$(find dsr_launcher)`: ROS 패키지 `dsr_launcher`의 경로를 찾아줌
- `single_robot.launch`: 두산 로봇 드라이버를 실행하는 Launch 파일
- `$(arg doosan_model)`: 위에서 정의한 `doosan_model` 변수의 값을 사용
- `mode="real"`: 실제 로봇 모드 (시뮬레이션이 아님)
- `host`: 로봇 컨트롤러의 IP 주소 전달

**이것이 하는 일:**
1. 두산 로봇 드라이버를 실행
2. 로봇과 ROS 시스템 간의 통신 연결 설정
3. `/dsr01a0912/motion/move_joint` 같은 서비스를 제공

#### 3.2.3 두산 명령 수신 노드 (27번 줄)

```xml
<node name="move_robot_node" pkg="doosan_helper" type="move_robot_node" output="screen"/>
```

**`<node>` 태그란?**
ROS 노드(프로그램)를 실행하는 태그입니다.

**속성 설명:**
- `name`: 노드의 이름 (`move_robot_node`)
- `pkg`: 패키지 이름 (`doosan_helper`)
- `type`: 실행할 프로그램 파일명 (`move_robot_node` - C++ 실행 파일)
- `output="screen"`: 로그를 화면에 출력

**이 노드가 하는 일:**
1. `/katech/robot_command` 토픽을 구독 (명령 수신)
2. 두산 드라이버의 `move_joint` 서비스를 호출 (로봇 제어)
3. `/doosan/status` 토픽에 상태 발행 (완료/에러 등)

---

### 3.3 모바일 로봇 시스템 섹션 (30-34번 줄)

#### 3.3.1 환경 변수 설정 (33번 줄)

```xml
<env name="PYTHONPATH" value="$(find mobile_robot_control)/src:$(env PYTHONPATH)" />
```

**`<env>` 태그란?**
환경 변수를 설정하는 태그입니다. Python이 모듈을 찾을 수 있도록 경로를 추가합니다.

**설명:**
- `PYTHONPATH`: Python이 모듈을 찾는 경로
- `$(find mobile_robot_control)/src`: `mobile_robot_control` 패키지의 `src` 폴더 경로
- `:$(env PYTHONPATH)`: 기존 PYTHONPATH 값도 유지 (추가)

**왜 필요한가?**
`move_mobile_robot_node.py`가 같은 폴더의 `mobile_robot_twist_control.py`를 import하기 위해 필요합니다.

#### 3.3.2 모바일 로봇 제어 노드 (34번 줄)

```xml
<node name="mobile_robot_ros_node" pkg="mobile_robot_control" type="move_mobile_robot_node.py" output="screen" />
```

**설명:**
- `name`: 노드 이름 (`mobile_robot_ros_node`)
- `pkg`: 패키지 이름 (`mobile_robot_control`)
- `type`: Python 스크립트 파일명 (`move_mobile_robot_node.py`)
- `output="screen"`: 로그를 화면에 출력

**이 노드가 하는 일:**
1. `/mobile/cmd` 토픽을 구독 (이동 명령 수신)
2. Woosh SDK를 통해 모바일 로봇을 실제로 제어
3. `/mobile/status` 토픽에 상태 발행 (IDLE, MOVING, COMPLETED, ERROR)

---

### 3.4 중앙 관제 노드 섹션 (37-45번 줄)

#### 3.4.1 중앙 관제 노드 실행 (40-45번 줄)

```xml
<node name="central_coordinator" pkg="central_coordinator" type="coordinator_node.py" output="screen">
    <!-- 파라미터 전달 -->
    <param name="mobile_distance" value="$(arg mobile_distance)"/>
    <param name="mobile_speed" value="$(arg mobile_speed)"/>
    <param name="cycle_delay" value="$(arg cycle_delay)"/>
</node>
```

**`<param>` 태그란?**
노드에 전달할 파라미터를 설정하는 태그입니다. 노드 내부에서 `rospy.get_param()`으로 읽을 수 있습니다.

**설명:**
- `name`: 파라미터 이름
- `value`: 파라미터 값 (`$(arg ...)`로 위에서 정의한 변수 사용)

**이 노드가 하는 일:**
1. **초기화**: 두산 로봇이 준비될 때까지 대기
2. **사이클 반복**: 
   - 모바일 로봇 전진 명령
   - 두산 로봇 작업 자세 명령
   - 모바일 로봇 후진 명령
   - 두산 로봇 홈 복귀 명령
3. **상태 모니터링**: 각 로봇의 상태를 확인하며 진행

---

## 4. 실행 순서와 데이터 흐름

### 4.1 노드 실행 순서

```
[1] 두산 드라이버 (include)
    └─> /dsr01a0912/motion/move_joint 서비스 제공
    └─> /dsr01a0912/state 토픽 발행

[2] 두산 명령 노드 (move_robot_node)
    └─> /katech/robot_command 구독
    └─> /doosan/status 발행

[3] 모바일 로봇 노드 (mobile_robot_ros_node)
    └─> /mobile/cmd 구독
    └─> /mobile/status 발행

[4] 중앙 관제 노드 (central_coordinator)
    └─> 모든 토픽을 구독/발행하여 전체 제어
```

### 4.2 데이터 흐름도

```
[중앙 관제 노드]
    │
    ├─> /mobile/cmd 발행 (이동 명령)
    │   └─> [모바일 노드] 수신 → 로봇 제어
    │       └─> /mobile/status 발행 (완료 신호)
    │           └─> [중앙 관제 노드] 수신
    │
    └─> /katech/robot_command 발행 (자세 명령)
        └─> [두산 명령 노드] 수신
            └─> /dsr01a0912/motion/move_joint 서비스 호출
                └─> [두산 드라이버] 처리 → 로봇 제어
                    └─> /doosan/status 발행 (완료 신호)
                        └─> [중앙 관제 노드] 수신
```

---

## 5. 실제 실행 방법

### 5.1 기본 실행

```bash
# 1. ROS 환경 설정
source ~/catkin_ws/devel/setup.bash

# 2. Launch 파일 실행
roslaunch central_coordinator integrated_system.launch
```

### 5.2 파라미터를 변경하여 실행

```bash
# 더 빠르고 멀리 이동하도록 설정
roslaunch central_coordinator integrated_system.launch \
    mobile_distance:=0.5 \
    mobile_speed:=0.3 \
    cycle_delay:=3.0

# 다른 로봇 모델과 IP로 실행
roslaunch central_coordinator integrated_system.launch \
    doosan_model:=m1013 \
    doosan_host:=192.168.1.50
```

### 5.3 실행 확인 방법

**터미널 1: Launch 파일 실행**
```bash
roslaunch central_coordinator integrated_system.launch
```

**터미널 2: 노드 확인**
```bash
rosnode list
# 출력 예시:
# /central_coordinator
# /mobile_robot_ros_node
# /move_robot_node
# /dsr01a0912/...
```

**터미널 3: 토픽 확인**
```bash
rostopic list
# 출력 예시:
# /mobile/cmd
# /mobile/status
# /katech/robot_command
# /doosan/status
# /dsr01a0912/state
```

**터미널 4: 상태 모니터링**
```bash
# 모바일 로봇 상태 확인
rostopic echo /mobile/status

# 두산 로봇 상태 확인
rostopic echo /doosan/status
```

---

## 6. 핵심 개념 정리

### 6.1 주요 태그 정리

| 태그 | 역할 | 예시 |
|------|------|------|
| `<arg>` | 실행 시 변경 가능한 변수 정의 | `<arg name="speed" default="0.2"/>` |
| `<param>` | 노드에 전달할 파라미터 설정 | `<param name="distance" value="0.3"/>` |
| `<node>` | ROS 노드(프로그램) 실행 | `<node name="my_node" pkg="my_pkg" type="my_node.py"/>` |
| `<include>` | 다른 Launch 파일 포함 | `<include file="$(find pkg)/launch/file.launch"/>` |
| `<env>` | 환경 변수 설정 | `<env name="PYTHONPATH" value="/path"/>` |

### 6.2 특수 문법

| 문법 | 의미 | 예시 |
|------|------|------|
| `$(find 패키지명)` | 패키지 경로 찾기 | `$(find mobile_robot_control)` |
| `$(arg 변수명)` | arg 변수 값 사용 | `$(arg mobile_distance)` |
| `$(env 환경변수명)` | 환경 변수 값 사용 | `$(env PYTHONPATH)` |

### 6.3 주석 (Comment)

```xml
<!-- 이것은 주석입니다. 실행되지 않아요 -->
```

---

## 7. 학습 체크리스트

다음 내용을 이해했는지 확인해보세요:

- [ ] Launch 파일이 여러 노드를 한 번에 실행하는 설정 파일이라는 것을 이해했다
- [ ] `<arg>`, `<param>`, `<node>`, `<include>` 태그의 차이를 안다
- [ ] `$(find ...)`, `$(arg ...)` 문법의 의미를 안다
- [ ] 각 노드가 어떤 역할을 하는지 설명할 수 있다
- [ ] 노드 간 데이터 흐름(토픽)을 이해했다
- [ ] 파라미터를 변경하여 실행할 수 있다

---

## 8. 다음 단계

이제 Launch 파일을 이해했으니, 다음 단계로 넘어가세요:

1. **중앙 관제 노드 코드 분석** (`coordinator_node.py`)
   - Launch 파일이 실행하는 노드 중 하나
   - 실제 협업 로직이 구현된 곳

2. **토픽 통신 이해**
   - `/mobile/cmd`, `/doosan/status` 등이 어떻게 사용되는지
   - `rostopic pub`, `rostopic echo` 명령어 연습

3. **실제 장비 테스트**
   - Launch 파일 실행 후 로그 확인
   - 각 단계별 동작 검증

---

**작성자**: AI Assistant  
**최종 수정일**: 2025-01-XX  
**대상 독자**: ROS 초보자

