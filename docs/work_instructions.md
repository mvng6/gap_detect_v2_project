# 작업 지시서: ROS Action 기반 로봇 협동 시스템 구축

## 1. 소개: 단순 제어에서 지능형 협동으로

현재 우리 시스템은 두산 로봇을 `/katech/robot_command` 토픽에 정수 값을 보내는 방식으로 성공적으로 제어하고 있으며, 모바일 로봇은 독립적인 Python 스크립트로 정밀 이동이 가능합니다. 이는 각 로봇의 기능을 검증하는 훌륭한 첫 단계였습니다.

이제 우리는 다음 단계로 나아가, 두 로봇이 서로의 상태를 인지하고 순차적으로 작업을 수행하는 **지능형 협동 시스템**을 구축하고자 합니다.

### 목표

단순한 '트리거' 방식에서 벗어나, ROS의 표준 통신 아키텍처인 **액션(Action)**을 도입하여 다음과 같은 목표를 달성합니다.

1.  **명확한 작업 관리:** "이동 시작"이라는 단순 신호가 아닌, "목표 지점 X까지 이동하고, 완료되면 알려줘"와 같이 목표, 피드백, 결과를 포함하는 정형화된 작업을 요청합니다.
2.  **안정적인 순서 제어:** 중앙 관제 노드가 각 로봇의 작업 완료를 명확히 확인한 후 다음 명령을 내리므로, 타이밍 문제나 작업 누락 없이 안정적인 순차 동작을 보장합니다.
3.  **확장성:** 새로운 로봇이나 작업 단계를 추가하기 쉬운 모듈식 구조를 만듭니다.

이 문서는 기존의 동작하는 코드는 그대로 유지하면서, 새로운 기능들을 단계별로 추가하여 최종 목표를 달성하는 과정을 안내합니다.

---

## 2. 전체 작업 흐름

우리는 다음 5단계에 걸쳐 시스템을 구축할 것입니다.

-   **1단계: 공용 통신 인터페이스 정의 (`robot_interfaces` 패키지)**
    -   모든 노드가 사용할 '언어'인 액션(.action) 파일을 정의하고, 이를 관리하는 전용 패키지를 생성합니다.

-   **2단계: 모바일 로봇 노드 업그레이드**
    -   기존의 `asyncio` 기반 제어 로직을 감싸는 새로운 ROS 노드를 만듭니다.
    -   이 노드는 `MoveMobile.action` 요청을 받아 로봇을 움직이고, 자신의 상태(이동 중/정지)를 토픽으로 발행합니다.

-   **3단계: 두산 로봇 노드 업그레이드**
    -   `move_robot_node`의 서비스 호출 기능을 활용하는 새로운 액션 서버(`MoveArmSequence.action`)를 C++로 구현합니다.
    -   이 서버는 모바일 로봇이 멈췄는지 **확인**한 후에만 움직임을 시작하는 안전장치를 포함합니다.

-   **4단계: 중앙 관제탑 노드 구현 (`robot_coordinator` 패키지)**
    -   전체 작업 순서를 지휘하는 '지휘자' 노드를 만듭니다.
    -   모바일 로봇에게 이동 명령을 보내고, 완료를 기다린 후 두산 로봇에게 팔 동작 명령을 내립니다.

-   **5단계: 통합 및 테스트**
    -   지금까지 만든 모든 노드를 `roslaunch` 파일을 통해 한 번에 실행하고, 전체 시나리오가 정상적으로 동작하는지 검증합니다.

이제 1단계부터 시작하겠습니다.

---

## 3. 1단계: 공용 통신 인터페이스 정의 (`robot_interfaces`)

이 단계에서는 모든 로봇과 제어 노드가 공유할 '언어'를 정의합니다. 즉, 액션(`action`) 메시지를 담을 전용 패키지를 만듭니다. 이렇게 하면 인터페이스 정의를 코드와 분리하여 관리가 쉬워집니다.

### 3.1. `robot_interfaces` 패키지 생성

먼저, Docker 컨테이너에 접속하여 `catkin_ws/src` 디렉터리에서 새 패키지를 생성합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws/src`

```bash
# /root/catkin_ws/src 경로에서 실행
catkin_create_pkg robot_interfaces std_msgs geometry_msgs actionlib_msgs message_generation actionlib
```

-   `message_generation`은 `.action` 파일을 C++ 헤더와 Python 코드로 변환하기 위해 필요합니다.
-   `actionlib`과 `actionlib_msgs`는 액션 라이브러리를 사용하기 위한 표준 의존성입니다.

### 3.2. 액션(`action`) 파일 정의

이제 생성된 `robot_interfaces` 폴더 안에 `action`이라는 새 폴더를 만들고, 그 안에 두 로봇을 위한 액션 정의 파일을 작성합니다.

> **[파일 생성 위치]**
> Host VS Code: `~/robot_ws/src/robot_interfaces/`

1.  `action` 폴더를 생성합니다.
    -   최종 경로: `~/robot_ws/src/robot_interfaces/action/`

2.  `MoveMobile.action` 파일을 생성하고 아래 내용을 붙여넣습니다.
    -   파일 경로: `~/robot_ws/src/robot_interfaces/action/MoveMobile.action`
    ```action
    # 목표 (Goal): 어떤 작업을 해야 하는가?
    # 이 예시에서는 목표 지점 대신, 정해진 거리와 속도로 이동하도록 단순화합니다.
    # 추후 Pose (x, y, theta) 목표로 확장할 수 있습니다.
    float32 target_distance     # 이동할 목표 거리 (m)
    float32 max_speed           # 최대 이동 속도 (m/s)
    ---
    # 결과 (Result): 작업이 성공했는가? 결과는 무엇인가?
    bool success                # 목표 도달 시 True
    float32 final_distance      # 실제 이동한 거리
    ---
    # 피드백 (Feedback): 작업 중 진행 상황은 어떤가?
    float32 distance_remaining  # 목표까지 남은 거리 (m)
    ```

3.  `MoveArmSequence.action` 파일을 생성하고 아래 내용을 붙여넣습니다.
    -   파일 경로: `~/robot_ws/src/robot_interfaces/action/MoveArmSequence.action`
    ```action
    # 목표 (Goal)
    # 특정 시퀀스를 ID로 요청합니다 (예: 1번은 장애물 회피 동작).
    int32 sequence_id
    ---
    # 결과 (Result)
    bool success
    ---
    # 피드백 (Feedback)
    # 현재 어떤 단계에 있는지 문자로 피드백합니다.
    string status  # 예: "Moving to target", "Returning to home"
    ```

### 3.3. `package.xml` 수정

`package.xml` 파일에 액션 메시지 빌드에 필요한 의존성을 추가합니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/robot_interfaces/package.xml`

아래 `<build_depend>`와 `<exec_depend>` 태그들을 기존 `package.xml` 파일의 적절한 위치에 추가하거나, 기존 태그를 아래 내용으로 교체합니다.

```xml
  <!-- 기존 내용은 생략... -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 아래 의존성들을 추가 또는 확인합니다. -->
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>

  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <!-- 기존 내용은 생략... -->
```

### 3.4. `CMakeLists.txt` 수정

마지막으로 `CMakeLists.txt`를 수정하여 `catkin_make`가 `.action` 파일을 빌드하도록 설정합니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/robot_interfaces/CMakeLists.txt`

`CMakeLists.txt` 파일에서 아래 섹션들을 찾아 주석을 해제하고 필요한 패키지 이름을 추가합니다. 대부분 이미 존재하므로 주석(`#`)만 제거하면 됩니다.

```cmake
# 1. find_package 섹션에 actionlib와 message_generation 추가
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
)

# 2. add_action_files 섹션 주석 해제 및 .action 파일 등록
add_action_files(
  FILES
  MoveMobile.action
  MoveArmSequence.action
)

# 3. generate_messages 섹션 주석 해제
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

# 4. catkin_package 섹션에 CATKIN_DEPENDS 추가
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_interfaces
 CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs std_msgs
#  DEPENDS system_lib
)
```

### 3.5. 빌드 및 확인

모든 파일 수정이 완료되면, 컨테이너 내부에서 `catkin_make`를 실행하여 액션 메시지가 올바르게 생성되는지 확인합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws`

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

빌드가 성공적으로 완료되면, `rosmsg list | grep robot_interfaces` 명령어로 새로 정의한 액션 관련 메시지들이 생성되었는지 확인할 수 있습니다.

```bash
# 아래와 유사한 결과가 출력되어야 합니다.
rosmsg list | grep robot_interfaces
> robot_interfaces/MoveArmSequenceAction
> robot_interfaces/MoveArmSequenceActionFeedback
> robot_interfaces/MoveArmSequenceActionGoal
> robot_interfaces/MoveArmSequenceActionResult
> robot_interfaces/MoveArmSequenceFeedback
> robot_interfaces/MoveArmSequenceGoal
> robot_interfaces/MoveArmSequenceResult
> robot_interfaces/MoveMobileAction
> ... (MoveMobile 관련 메시지들)
```

이것으로 1단계가 완료되었습니다. 이제 우리의 로봇들은 새로운 '언어'를 가졌습니다. 다음 단계에서는 이 언어를 사용하여 모바일 로봇을 제어하는 노드를 만듭니다.

---

## 4. 2단계: 모바일 로봇 노드 업그레이드

이 단계에서는 기존의 `mobile_robot_control_node.py` 파일은 **수정하지 않고**, 그 기능(특히 `MobileRobotController` 클래스)을 라이브러리처럼 활용하는 새로운 ROS 액션 서버 노드를 만듭니다.

### 4.1. 새 액션 서버 노드 파일 생성

`mobile_robot_control` 패키지의 `src` 폴더에 새로운 Python 파일을 생성합니다. 이 파일이 ROS 액션 서버의 역할을 담당하게 됩니다.

> **[파일 생성 위치]**
> Host VS Code: `~/robot_ws/src/mobile_robot_control/src/`

-   새 파일 이름: `mobile_robot_action_server.py`

### 4.2. 액션 서버 코드 작성

방금 생성한 `mobile_robot_action_server.py` 파일에 아래의 전체 코드를 붙여넣습니다. 코드의 각 부분에 대한 설명은 주석을 참고하세요.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/mobile_robot_control/src/mobile_robot_action_server.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import asyncio
import threading
from std_msgs.msg import String

# Action 메시지 타입 임포트
from robot_interfaces.msg import MoveMobileAction, MoveMobileGoal, MoveMobileResult, MoveMobileFeedback

# 기존 제어 로직 임포트 (파일을 수정하지 않고 클래스만 가져와 사용)
from mobile_robot_control_node import MobileRobotController, RobotConfig, VelocityProfileConfig

class MobileRobotActionServer:
    def __init__(self):
        rospy.loginfo("🤖 모바일 로봇 액션 서버 초기화 시작...")

        # 로봇 상태를 발행할 퍼블리셔
        self._status_publisher = rospy.Publisher('/mobile_robot/status', String, queue_size=1)

        # 액션 서버 생성
        # 서버 이름: /move_mobile
        # 액션 타입: MoveMobileAction
        # 콜백 함수: self.execute_cb (새로운 목표(goal)가 들어오면 이 함수가 호출됨)
        self._server = actionlib.SimpleActionServer(
            '/move_mobile',
            MoveMobileAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        # 액션 서버 시작
        self._server.start()
        rospy.loginfo("✅ 모바일 로봇 액션 서버가 /move_mobile 토픽에서 목표를 기다리고 있습니다.")

    def execute_cb(self, goal: MoveMobileGoal):
        """
        새로운 Action Goal을 받았을 때 실행되는 메인 콜백 함수.
        asyncio 로직을 별도의 스레드에서 실행하여 ROS 이벤트 루프와 분리합니다.
        """
        rospy.loginfo(f"🎯 새로운 목표 수신: {goal.target_distance:.2f}m 이동 (최대 속도: {goal.max_speed:.2f}m/s)")

        # asyncio 코드를 실행할 별도의 스레드 생성 및 시작
        thread = threading.Thread(target=self.run_async_task, args=(goal,))
        thread.start()

        # 스레드가 완료될 때까지 대기 (이 시간 동안 피드백 수신 가능)
        thread.join()

        rospy.loginfo("- 스레드 작업 완료, 결과 처리 -")

        # 스레드에서 저장한 결과에 따라 Action 서버의 최종 상태 결정
        if hasattr(self, '_thread_result') and self._thread_result.success:
            rospy.loginfo("✅ Action 성공 처리")
            self._server.set_succeeded(self._thread_result)
        else:
            rospy.loginfo("❌ Action 실패 처리")
            # 실패 시에는 빈 결과(기본값)를 전송
            self._server.set_aborted(MoveMobileResult(success=False, final_distance=self._thread_result.final_distance))

    def run_async_task(self, goal: MoveMobileGoal):
        """
        별도의 스레드에서 asyncio 이벤트 루프를 실행하는 함수.
        """
        try:
            # 새 이벤트 루프를 얻거나 생성하여 현재 스레드의 기본 루프로 설정
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # 메인 비동기 로직 실행
            result = loop.run_until_complete(self.handle_movement(goal))
            self._thread_result = result
        except Exception as e:
            rospy.logerr(f"💥 Asyncio 태스크 실행 중 오류 발생: {e}")
            self._thread_result = MoveMobileResult(success=False, final_distance=0.0)
        finally:
            loop.close()

    async def handle_movement(self, goal: MoveMobileGoal) -> MoveMobileResult:
        """
        실제 로봇 연결 및 이동을 처리하는 비동기 함수.
        """
        # 로봇 연결 설정 (IP 등은 실제 환경에 맞게 조정 필요)
        robot_config = RobotConfig(ip='169.254.128.2', port=5480)
        controller = MobileRobotController(robot_config)

        try:
            await controller.connect()

            # 1. 상태 발행: "MOVING"
            self._status_publisher.publish(String(data="MOVING"))

            # 2. 이동 실행 및 피드백 발행
            # 이동이 완료될 때까지 주기적으로 피드백을 발행하는 태스크와
            # 실제 이동을 실행하는 태스크를 동시에 실행
            feedback_task = asyncio.create_task(self.publish_feedback(controller, goal.target_distance))

            motion_result = await controller.move_distance(
                target_distance=goal.target_distance,
                speed=goal.max_speed,
                velocity_config=VelocityProfileConfig(max_speed=abs(goal.max_speed))
            )

            # 피드백 태스크가 완료되도록 잠시 대기 후 취소
            await asyncio.sleep(0.1)
            feedback_task.cancel()

            # 3. 상태 발행: "STOPPED"
            self._status_publisher.publish(String(data="STOPPED"))

            # 4. 최종 결과 생성
            result = MoveMobileResult(
                success=motion_result.success,
                final_distance=motion_result.traveled_distance
            )
            rospy.loginfo(f"📊 이동 완료. 실제 이동 거리: {motion_result.traveled_distance:.3f}m")

        except Exception as e:
            rospy.logerr(f"💥 로봇 이동 처리 중 오류: {e}")
            self._status_publisher.publish(String(data="STOPPED")) # 오류 발생 시에도 정지 상태 발행
            result = MoveMobileResult(success=False, final_distance=0.0)
        finally:
            await controller.disconnect()

        return result

    async def publish_feedback(self, controller: MobileRobotController, target_distance: float):
        """
        주기적으로 로봇의 위치를 확인하고 Action Feedback을 발행하는 비동기 함수.
        """
        start_pose = await controller.get_current_pose()
        if not start_pose:
            rospy.logwarn("피드백 발행을 위한 시작 위치를 얻지 못했습니다.")
            return

        while not self._server.is_preempt_requested():
            await asyncio.sleep(0.2) # 5Hz

            current_pose = controller.current_pose # 콜백으로 업데이트되는 위치 사용
            if current_pose:
                traveled_distance = controller.calculate_distance(start_pose, current_pose)
                remaining = target_distance - traveled_distance

                # 피드백 메시지 생성 및 발행
                feedback = MoveMobileFeedback(distance_remaining=remaining)
                self._server.publish_feedback(feedback)

                # 목표에 거의 도달하면 루프 종료
                if remaining < 0.01:
                    break

        rospy.loginfo("⏹️ 피드백 발행 중단.")


if __name__ == '__main__':
    try:
        rospy.init_node('mobile_robot_action_server')
        server = MobileRobotActionServer()
        rospy.spin() # ROS 이벤트 루프 시작 (콜백 대기)
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 프로그램 종료")

```

### 4.3. 코드에 대한 주요 설명

-   **`threading` 사용 이유**: `rospy.spin()`은 메인 스레드를 차지하여 ROS 이벤트를 처리합니다. 반면, `MobileRobotController`는 `asyncio`라는 별도의 이벤트 루프를 사용합니다. 이 둘을 충돌 없이 함께 사용하기 위해, `asyncio` 로직 전체를 별도의 스레드에서 실행하는 것이 가장 간단하고 안정적인 방법입니다.
-   **액션 서버의 역할**: 이 코드는 ROS 네트워크로부터 "이동"이라는 목표를 받고, 그에 맞춰 `MobileRobotController`의 `move_distance` 함수를 호출하는 '어댑터' 또는 '브릿지' 역할을 합니다.
-   **상태 발행**: 로봇이 움직이기 시작하면 `/mobile_robot/status` 토픽에 `"MOVING"`을, 움직임이 끝나면 `"STOPPED"`를 발행합니다. 이는 나중에 두산 로봇이 자신의 움직임 시작 여부를 결정하는 데 사용됩니다.

### 4.4. 실행 권한 부여

새로 만든 Python 파일이 ROS에서 실행될 수 있도록 실행 권한을 부여해야 합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws/src/mobile_robot_control/src`

```bash
chmod +x mobile_robot_action_server.py
```

이제 모바일 로봇은 ROS 액션 시스템에 통합될 준비를 마쳤습니다. 아직 실행은 하지 않고, 다음 단계로 넘어가 두산 로봇 노드를 업그레이드합니다.

---

## 5. 3단계: 두산 로봇 노드 업그레이드

이 단계에서는 `doosan_helper` 패키지 안에 새로운 C++ 액션 서버 노드를 만듭니다. 이 서버는 `/move_arm_sequence` 액션 목표를 받아 처리하며, 가장 중요한 기능으로 **모바일 로봇의 상태를 확인**하여 안전하게 팔을 움직이는 로직을 포함합니다.

### 5.1. 새 액션 서버 C++ 파일 생성

`doosan_helper/src` 폴더에 새로운 C++ 소스 파일을 생성합니다.

> **[파일 생성 위치]**
> Host VS Code: `~/robot_ws/src/doosan_helper/src/`

-   새 파일 이름: `doosan_action_server.cpp`

### 5.2. 액션 서버 C++ 코드 작성

새로 생성한 `doosan_action_server.cpp` 파일에 아래의 전체 코드를 붙여넣습니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/doosan_helper/src/doosan_action_server.cpp`

```cpp
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_interfaces/MoveArmSequenceAction.h>
#include <std_msgs/String.h>
#include "dsr_msgs/MoveJoint.h"

class DoosanActionServer
{
public:
    DoosanActionServer(std::string name) :
        as_(nh_, name, boost::bind(&DoosanActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        // 모바일 로봇의 상태를 구독
        mobile_status_sub_ = nh_.subscribe("/mobile_robot/status", 1, &DoosanActionServer::mobileStatusCB, this);

        // 두산 로봇 자신의 상태를 발행
        doosan_status_pub_ = nh_.advertise<std_msgs::String>("/doosan_robot/status", 1);

        // 두산 로봇의 move_joint 서비스를 사용하기 위한 클라이언트
        move_client_ = nh_.serviceClient<dsr_msgs::MoveJoint>("/dsr01a0912/motion/move_joint");

        as_.start();
        ROS_INFO("✅ 두산 로봇 액션 서버가 시작되었습니다.");
    }

    // 모바일 로봇 상태 콜백
    void mobileStatusCB(const std_msgs::String::ConstPtr& msg)
    {
        latest_mobile_status_ = msg->data;
    }

    // 메인 액션 콜백
    void executeCB(const robot_interfaces::MoveArmSequenceGoalConstPtr &goal)
    {
        ROS_INFO("🎯 두산 로봇: 새로운 목표(sequence_id: %d)를 받았습니다.", goal->sequence_id);

        // 1. 모바일 로봇이 멈출 때까지 기다리기 (최대 10초)
        ros::Rate r(10); // 10Hz
        ros::Time start_time = ros::Time::now();
        while (latest_mobile_status_ != "STOPPED")
        {
            if (ros::Time::now() - start_time > ros::Duration(10.0))
            {
                ROS_ERROR("❌ 시간 초과: 모바일 로봇이 10초 내에 'STOPPED' 상태가 되지 않았습니다.");
                as_.setAborted();
                return;
            }
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_WARN(" preempt 요청으로 작업을 중단합니다.");
                as_.setPreempted();
                return;
            }
            ROS_INFO_THROTTLE(1, "⏳ 모바일 로봇이 멈추기를 기다리는 중... (현재: %s)", latest_mobile_status_.c_str());
            r.sleep();
        }

        ROS_INFO("✅ 모바일 로봇 정지 확인. 팔 동작을 시작합니다.");

        // 2. 팔 움직임 실행
        publishStatus("MOVING");
        bool success = true;

        // sequence_id에 따라 다른 동작 수행 (현재는 1가지 동작만 정의)
        if (goal->sequence_id == 1) {
            // 피드백 발행
            robot_interfaces::MoveArmSequenceFeedback feedback;
            feedback.status = "Moving to target position";
            as_.publishFeedback(feedback);

            // 첫 번째 자세로 이동
            if (!moveArm({90.0, 0.0, 90.0, 0.0, 90.0, -90.0})) {
                success = false;
            }

            // 작업 중단 요청이 없다면, 잠시 대기 후 홈으로 복귀
            if (success && !as_.isPreemptRequested()) {
                ros::Duration(1.0).sleep(); // 간단한 대기
                feedback.status = "Returning to home position";
                as_.publishFeedback(feedback);
                if (!moveArm({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
                    success = false;
                }
            }
        } else {
            ROS_ERROR("지원하지 않는 sequence_id 입니다: %d", goal->sequence_id);
            success = false;
        }

        // 3. 최종 결과 전송
        robot_interfaces::MoveArmSequenceResult result;
        result.success = success;

        if (success)
        {
            ROS_INFO("✅ 팔 동작 시퀀스 완료.");
            as_.setSucceeded(result);
        }
        else
        {
            ROS_ERROR("❌ 팔 동작 시퀀스 실패.");
            as_.setAborted(result);
        }

        publishStatus("IDLE_HOME");
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot_interfaces::MoveArmSequenceAction> as_;
    std::string action_name_;

    ros::Subscriber mobile_status_sub_;
    ros::Publisher doosan_status_pub_;
    ros::ServiceClient move_client_;

    std::string latest_mobile_status_ = "UNKNOWN";

    // 로봇 팔을 움직이는 헬퍼 함수
    bool moveArm(const std::vector<double>& pos)
    {
        dsr_msgs::MoveJoint srv;
        srv.request.pos = pos;
        srv.request.vel = 30.0;
        srv.request.acc = 60.0;
        srv.request.mode = 0; // MOVE_MODE_ABSOLUTE

        if (move_client_.call(srv) && srv.response.success) {
            ROS_INFO("move_joint 서비스 호출 성공.");
            return true;
        } else {
            ROS_ERROR("move_joint 서비스 호출 실패.");
            return false;
        }
    }

    // 상태 발행 헬퍼 함수
    void publishStatus(const std::string& status)
    {
        std_msgs::String msg;
        msg.data = status;
        doosan_status_pub_.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "doosan_action_server");
    DoosanActionServer server("move_arm_sequence");
    ros::spin();
    return 0;
}
```

### 5.3. `CMakeLists.txt` 수정

새로운 C++ 노드를 빌드 시스템에 추가해야 합니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/doosan_helper/CMakeLists.txt`

`CMakeLists.txt` 파일의 맨 아래에 다음 내용을 추가합니다. 기존의 `move_robot_node` 관련 설정 바로 뒤에 추가하는 것이 좋습니다.

```cmake
# Doosan Action Server 실행 파일 추가
add_executable(doosan_action_server src/doosan_action_server.cpp)

# 필요한 라이브러리 링크
target_link_libraries(doosan_action_server
  ${catkin_LIBRARIES}
)
```

### 5.4. `package.xml` 의존성 확인

`doosan_helper` 패키지가 `actionlib`과 `robot_interfaces`에 의존하도록 `package.xml`에 명시해야 합니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/doosan_helper/package.xml`

파일을 열어 아래 태그들이 있는지 확인하고, 없다면 추가해줍니다.

```xml
  <!-- ... -->
  <build_depend>actionlib</build_depend>
  <build_depend>robot_interfaces</build_depend>

  <exec_depend>actionlib</exec_depend>
  <exec_depend>robot_interfaces</exec_depend>
  <!-- ... -->
```

### 5.5. 재빌드

모든 수정이 완료되었으므로, `catkin_make`를 실행하여 새로운 노드를 컴파일합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws`

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

빌드가 성공하면, 이제 두산 로봇도 액션 시스템에 통합되었습니다. 다음 단계에서는 이 모든 것을 지휘할 중앙 관제탑 노드를 만듭니다.

---

## 6. 4단계: 중앙 관제탑 노드 구현 (`robot_coordinator`)

드디어 프로젝트의 '지휘자'를 만들 차례입니다. 이 노드는 두 로봇 액션 서버의 **클라이언트** 역할을 하며, 전체 작업 흐름을 순서대로 관리합니다.

### 6.1. `robot_coordinator` 패키지 생성

먼저, 중앙 관제탑 노드를 위한 새 패키지를 생성합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws/src`

```bash
# /root/catkin_ws/src 경로에서 실행
catkin_create_pkg robot_coordinator roscpp actionlib robot_interfaces std_msgs
```

### 6.2. 관제탑 노드 C++ 파일 생성

`robot_coordinator/src` 폴더에 C++ 소스 파일을 생성합니다.

> **[파일 생성 위치]**
> Host VS Code: `~/robot_ws/src/robot_coordinator/src/`

-   새 파일 이름: `coordinator_node.cpp`

### 6.3. 관제탑 노드 C++ 코드 작성

생성한 `coordinator_node.cpp` 파일에 아래의 전체 코드를 붙여넣습니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/robot_coordinator/src/coordinator_node.cpp`

```cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_interfaces/MoveMobileAction.h>
#include <robot_interfaces/MoveArmSequenceAction.h>

class RobotCoordinator
{
public:
    RobotCoordinator() :
        // 액션 클라이언트 초기화 ("서버이름", true)
        // true는 스레드를 분리하여 통신함을 의미
        ac_mobile_("move_mobile", true),
        ac_doosan_("move_arm_sequence", true)
    {
        ROS_INFO("🤖 중앙 관제탑 노드 초기화 중...");
    }

    // 전체 시퀀스를 실행하는 메인 함수
    void runSequence()
    {
        // 1. 두 액션 서버가 켜질 때까지 무한정 대기
        ROS_INFO("⏳ 모바일 로봇 액션 서버를 기다리는 중...");
        ac_mobile_.waitForServer();
        ROS_INFO("✅ 모바일 로봇 액션 서버 연결 완료.");

        ROS_INFO("⏳ 두산 로봇 액션 서버를 기다리는 중...");
        ac_doosan_.waitForServer();
        ROS_INFO("✅ 두산 로봇 액션 서버 연결 완료.");

        // 무한 루프: 모바일 이동 -> 두산 팔 동작 -> 반복
        ros::Rate loop_rate(0.1); // 루프 사이 약 10초 대기
        int cycle_count = 1;

        while (ros::ok())
        {
            ROS_INFO("\n================ CYCLE %d START ================", cycle_count);

            // 2. 모바일 로봇 이동 명령
            if (!runMobileSequence()) {
                ROS_ERROR("모바일 로봇 시퀀스 실패. 10초 후 재시도...");
                ros::Duration(10.0).sleep();
                continue; // 다음 사이클로
            }

            // 3. 두산 로봇 팔 동작 명령
            if (!runDoosanSequence()) {
                ROS_ERROR("두산 로봇 시퀀스 실패. 10초 후 재시도...");
                ros::Duration(10.0).sleep();
                continue; // 다음 사이클로
            }

            ROS_INFO("================ CYCLE %d COMPLETE ================\n", cycle_count);
            cycle_count++;
            loop_rate.sleep();
        }
    }

private:
    actionlib::SimpleActionClient<robot_interfaces::MoveMobileAction> ac_mobile_;
    actionlib::SimpleActionClient<robot_interfaces::MoveArmSequenceAction> ac_doosan_;

    // 모바일 로봇 시퀀스 실행
    bool runMobileSequence() {
        ROS_INFO("[1/2] ➡️ 모바일 로봇 이동 시작 (1.0m, 0.2m/s)");
        robot_interfaces::MoveMobileGoal goal;
        goal.target_distance = 1.0;
        goal.max_speed = 0.2;

        ac_mobile_.sendGoal(goal);

        // 결과가 올 때까지 30초 동안 대기
        bool finished_before_timeout = ac_mobile_.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_mobile_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("✅ 모바일 로봇 이동 성공!");
                return true;
            } else {
                ROS_WARN("⚠️ 모바일 로봇 이동 실패: %s", state.toString().c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("❌ 모바일 로봇 이동 시간 초과!");
            ac_mobile_.cancelGoal();
            return false;
        }
    }

    // 두산 로봇 시퀀스 실행
    bool runDoosanSequence() {
        ROS_INFO("[2/2] 🦾 두산 로봇 동작 시작 (sequence 1)");
        robot_interfaces::MoveArmSequenceGoal goal;
        goal.sequence_id = 1;

        ac_doosan_.sendGoal(goal);

        // 결과가 올 때까지 30초 동안 대기
        bool finished_before_timeout = ac_doosan_.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_doosan_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("✅ 두산 로봇 동작 성공!");
                return true;
            } else {
                ROS_WARN("⚠️ 두산 로봇 동작 실패: %s", state.toString().c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("❌ 두산 로봇 동작 시간 초과!");
            ac_doosan_.cancelGoal();
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_coordinator");
    RobotCoordinator coordinator;
    coordinator.runSequence();
    return 0;
}
```

### 6.4. `CMakeLists.txt` 수정

새 C++ 노드를 빌드 시스템에 추가합니다.

> **[파일 수정]**
> Host VS Code: `~/robot_ws/src/robot_coordinator/CMakeLists.txt`

파일의 아래 부분에 다음 내용을 추가합니다.

```cmake
# ... (find_package 등)

add_executable(coordinator_node src/coordinator_node.cpp)

target_link_libraries(coordinator_node
  ${catkin_LIBRARIES}
)
```

### 6.5. 재빌드

마지막으로 `catkin_make`를 실행하여 관제탑 노드를 컴파일합니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws`

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

이제 모든 조각이 준비되었습니다. 마지막 단계에서는 이 모든 노드를 한 번에 실행하고 테스트하는 방법을 알아봅니다.

---

## 7. 5단계: 통합 및 테스트

이제 개별적으로 개발한 모든 노드들을 `roslaunch`를 사용하여 함께 실행하고, 전체 협동 작업 시나리오를 테스트합니다.

### 7.1. 마스터 런치(Launch) 파일 생성

여러 터미널에서 각 노드를 수동으로 실행하는 대신, 모든 노드를 한 번에 실행하는 마스터 런치 파일을 만듭니다. `robot_coordinator` 패키지 안에 `launch` 폴더와 파일을 생성합니다.

> **[파일 생성 위치]**
> Host VS Code: `~/robot_ws/src/robot_coordinator/`

1.  `launch` 폴더를 생성합니다.
    -   최종 경로: `~/robot_ws/src/robot_coordinator/launch/`

2.  `coordinate_robots.launch` 파일을 생성하고 아래 내용을 붙여넣습니다.
    -   파일 경로: `~/robot_ws/src/robot_coordinator/launch/coordinate_robots.launch`

```xml
<launch>
    <!-- ======================= 인자(Argument) 정의 ======================= -->
    <!-- 두산 로봇 연결 정보 -->
    <arg name="model"  default="a0912" doc="Doosan Robot Model Name"/>
    <arg name="mode"   default="real"  doc="real/simulation"/>
    <arg name="host"   default="192.168.137.100" doc="Doosan Controller IP address"/>

    <!-- 모바일 로봇 연결 정보 -->
    <arg name="mobile_ip" default="169.254.128.2" doc="Mobile Robot IP address"/>

    <!-- ======================= 두산 로봇 관련 노드 ======================= -->

    <!-- 1. 두산 로봇 드라이버 및 MoveIt 실행 -->
    <!-- 중요: dsr_launcher 패키지의 dsr_moveit.launch 파일을 포함(include) -->
    <include file="$(find dsr_launcher)/launch/dsr_moveit.launch">
        <arg name="model" value="$(arg model)"/>
        <arg name="mode"  value="$(arg mode)"/>
        <arg name="host"  value="$(arg host)"/>
    </include>

    <!-- 2. 두산 로봇 액션 서버 실행 -->
    <node name="doosan_action_server" pkg="doosan_helper" type="doosan_action_server" output="screen"/>

    <!-- ======================= 모바일 로봇 관련 노드 ======================= -->

    <!-- 3. 모바일 로봇 액션 서버 실행 -->
    <!-- 참고: mobile_robot_action_server.py 파일 내에서 IP 주소를 사용하므로,
         여기서는 launch 파일의 인자를 직접 넘기지는 않음 (필요시 코드 수정 가능) -->
    <node name="mobile_robot_action_server" pkg="mobile_robot_control" type="mobile_robot_action_server.py" output="screen"/>

    <!-- ======================= 중앙 관제탑 노드 ======================= -->

    <!-- 4. 중앙 관제탑 노드 실행 -->
    <!-- respawn="true"는 노드가 예기치 않게 종료되면 자동으로 재시작하는 옵션 -->
    <node name="robot_coordinator" pkg="robot_coordinator" type="coordinator_node" output="screen" respawn="true" launch-prefix="bash -c 'sleep 5; $0 $@'"/>

</launch>
```

-   `launch-prefix="bash -c 'sleep 5; $0 $@'"`: 이 부분은 다른 노드(특히 액션 서버)들이 완전히 실행될 시간을 벌어주기 위해 관제탑 노드를 5초 지연시켜 실행하는 유용한 트릭입니다.

### 7.2. 전체 시스템 실행

이제 단 하나의 명령어로 전체 시스템을 실행할 수 있습니다.

> **[명령어 실행 위치]**
> Host 터미널: `docker exec -it my_noetic_ws bash`
> Container 내부: `/root/catkin_ws`

```bash
# 1. 환경 설정 (매번 새 터미널에서 필요)
cd /root/catkin_ws
source devel/setup.bash

# 2. 마스터 런치 파일 실행
roslaunch robot_coordinator coordinate_robots.launch
```

런치 파일이 실행되면, 터미널에 각 노드의 로그가 출력되면서 다음과 같은 순서로 작업이 진행되는 것을 볼 수 있습니다.

1.  `coordinator_node`가 두 액션 서버(`move_mobile`, `move_arm_sequence`)를 기다립니다.
2.  연결이 완료되면, 모바일 로봇에게 1.0m 이동 명령을 보냅니다.
3.  `mobile_robot_action_server`가 목표를 받아 로봇을 움직이고, 상태를 `MOVING`으로 발행합니다.
4.  이동이 완료되면, 결과를 `coordinator_node`에 보내고 상태를 `STOPPED`로 발행합니다.
5.  `coordinator_node`가 성공 결과를 받고, 두산 로봇에게 팔 동작(sequence 1) 명령을 보냅니다.
6.  `doosan_action_server`가 `/mobile_robot/status` 토픽을 통해 `STOPPED` 상태를 확인하고 팔을 움직입니다.
7.  팔 동작이 완료되면, 결과를 `coordinator_node`에 보냅니다.
8.  `coordinator_node`가 모든 과정이 끝났음을 로그로 출력하고, 약 10초 후 다음 사이클을 시작합니다.

### 7.3. 디버깅 및 검증

시스템이 예상대로 동작하지 않을 경우, 다음과 같은 ROS 도구를 사용하여 문제를 진단할 수 있습니다. (새 `docker exec` 터미널 필요)

-   **노드 및 토픽 연결 상태 확인:** `rqt_graph`
    -   GUI를 통해 노드, 토픽, 서비스, 액션이 어떻게 연결되어 있는지 시각적으로 보여줍니다.
    ```bash
    # 새 터미널에서 실행
    rqt_graph
    ```

-   **토픽 메시지 실시간 확인:** `rostopic echo`
    -   로봇들의 상태가 올바르게 발행되는지 확인합니다.
    ```bash
    # 모바일 로봇 상태 확인
    rostopic echo /mobile_robot/status

    # 두산 로봇 상태 확인
    rostopic echo /doosan_robot/status
    ```

-   **액션 상태 확인:** `rostopic list` 및 `rostopic echo`
    -   액션 서버와 클라이언트 간의 통신(goal, feedback, result 등)을 상세히 볼 수 있습니다.
    ```bash
    # 사용 가능한 모든 액션 관련 토픽 확인
    rostopic list | grep move_mobile

    # 예: 모바일 로봇의 피드백 토픽 확인
    rostopic echo /move_mobile/feedback
    ```

## 8. 결론

이 작업 지시서를 통해 우리는 단순한 개별 제어 스크립트에서 출발하여, 두 로봇이 서로의 상태를 인지하고 협력하는 완전한 ROS 기반의 자율 시스템을 구축했습니다.

이제 이 아키텍처를 기반으로 더 복잡한 시나리오(예: 물체 집어서 옮기기)를 추가하거나, 새로운 로봇을 통합하는 등의 확장이 매우 용이해졌습니다. 수고하셨습니다!
