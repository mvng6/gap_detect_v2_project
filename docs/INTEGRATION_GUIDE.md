# 통합 관제 시스템 구축 작업 지시서

## ⚠️ 주의사항 (반드시 읽어주세요)

### Docker 환경 및 빌드 관련
- **호스트 PC 환경**: Ubuntu 24.04 LTS
- **ROS 환경**: Docker 컨테이너 내부에서 ROS Noetic 사용
- **빌드 담당**: User가 직접 빌드 진행
- **중요**: 이 문서는 시스템 설계 및 구현 방법을 안내하며, **코드 작성 후 반드시 User가 직접 빌드를 수행**해야 합니다.

### Docker 환경에서 작업하는 방법
```bash
# Docker 컨테이너 접속
docker exec -it <컨테이너_이름> bash

# 작업 공간 이동
cd /root/catkin_ws

# 빌드 (User가 필요 시 실행)
catkin_make

# 환경 변수 적용
source devel/setup.bash
```

### 현재 시스템 네트워크 구성
- **호스트 PC IP**: 192.168.137.X (두산 로봇 네트워크)
- **두산 로봇 IP**: 192.168.137.100
- **모바일 로봇 IP**: 169.254.128.2

---

## 📋 목차
1. [현재 시스템 구조 분석](#1-현재-시스템-구조-분석)
2. [통합 관제 시스템 구축 방안](#2-통합-관제-시스템-구축-방안)
3. [방안 A: 간단한 Topic 기반 통합](#3-방안-a-간단한-topic-기반-통합-권장)
4. [방안 B: 고급 Action 기반 통합](#4-방안-b-고급-action-기반-통합)
5. [실행 및 테스트](#5-실행-및-테스트)

---

## 1. 현재 시스템 구조 분석

### 1.1 두산 로봇 제어 시스템

#### 사용 중인 터미널 구성 (총 3개)

**터미널 1: 두산 로봇 드라이버 실행**
```bash
roslaunch dsr_launcher single_robot.launch model:=a0912 mode:=real host:=192.168.137.100
```
- 역할: 두산 로봇 제어기와 ROS 인터페이스 연결
- 제공하는 서비스: `/dsr01a0912/motion/move_joint` 등

**터미널 2: 로봇 명령 수신 노드**
```bash
rosrun doosan_helper move_robot_node
```
- 역할: `/katech/robot_command` 토픽 구독 및 명령 실행
- 명령 프로토콜:
  - `0` → 자세 `[90, 0, 90, 0, 90, -90]`로 이동
  - `1` → 자세 `[-90, 0, 90, 0, 90, -90]`로 이동
  - `99` → 홈 자세 `[0, 0, 0, 0, 0, 0]`로 이동

**터미널 3: 명령 발행 노드 (수동 실행)**
```bash
rosrun doosan_helper trigger_home_node  # 홈으로 이동
```
- 역할: `/katech/robot_command`에 명령 발행 (원샷 실행 후 종료)

#### 통신 구조
```
[trigger_home_node] 
    ↓ (발행: /katech/robot_command, msg=99)
[move_robot_node] 
    ↓ (구독 및 서비스 호출: /dsr01a0912/motion/move_joint)
[두산 로봇 드라이버] 
    ↓
[실제 로봇 하드웨어]
```

### 1.2 모바일 로봇 제어 시스템

#### 사용 중인 터미널 구성 (총 2개)

**터미널 1: ROS 마스터**
```bash
roscore
```

**터미널 2: 모바일 로봇 제어 스크립트**
```bash
cd ~/catkin_ws/src/mobile_robot_control/src
python3 mobile_robot_twist_control.py --distance 0.2 --speed 0.2
```
- 역할: 
  - 모바일 로봇 SDK 연결 (IP: 169.254.128.2)
  - 입력 파라미터(distance, speed)에 따라 자동 이동 실행
  - Odometry 피드백 기반 정밀 거리 제어
  - 가감속 프로파일 적용

#### 특징
- **독립 실행 방식**: ROS 토픽/서비스 없이 Python 스크립트 직접 실행
- **파라미터 기반**: 명령줄 인자로 동작 제어
- **Asyncio 사용**: 비동기 이벤트 루프로 SDK 제어

---

## 2. 통합 관제 시스템 구축 방안

현재 두 로봇은 **독립적으로** 동작하며, 서로의 상태를 알지 못합니다. 
통합 관제 시스템의 목표는 다음과 같습니다:

### 2.1 목표

**순차적 협동 작업 시나리오**
1. 중앙 관제 노드가 모바일 로봇에게 이동 명령
2. 모바일 로봇이 이동 완료 후 상태 보고
3. 중앙 관제 노드가 두산 로봇에게 동작 명령
4. 두산 로봇이 동작 완료 후 상태 보고
5. 사이클 반복

### 2.2 설계 원칙

1. **기존 코드 재사용**: 현재 동작하는 `move_robot_node`와 `mobile_robot_twist_control.py` 활용
2. **단계적 구축**: 간단한 방안부터 시작, 필요 시 고급 방안으로 확장
3. **안전성**: 로봇 간 충돌 방지를 위한 상태 확인 로직 포함

### 2.3 두 가지 구현 방안

| 항목 | 방안 A: Topic 기반 | 방안 B: Action 기반 |
|------|-------------------|-------------------|
| **난이도** | ⭐ 쉬움 | ⭐⭐⭐ 어려움 |
| **구현 시간** | 짧음 (2-3시간) | 김 (1-2일) |
| **기존 코드 활용** | 높음 (거의 그대로 사용) | 낮음 (대폭 수정 필요) |
| **피드백** | 제한적 (상태만) | 상세 (진행률, 에러 등) |
| **확장성** | 보통 | 높음 |
| **ROS 표준** | Topic/Service | Action (권장 패턴) |
| **추천 대상** | 빠른 프로토타입 필요 시 | 장기 프로젝트, 복잡한 시나리오 |

---

## 3. 방안 A: 간단한 Topic 기반 통합 (권장)

이 방안은 **기존 코드를 최소한으로 수정**하고, ROS Topic과 Service만으로 통합합니다.

### 3.1 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    중앙 관제 노드                              │
│              (central_coordinator_node)                      │
│                                                               │
│  ┌─────────────────────────────────────────────┐             │
│  │ 순차 제어 로직                                 │             │
│  │ 1. 모바일 이동 요청                             │             │
│  │ 2. 모바일 완료 대기                             │             │
│  │ 3. 두산 동작 요청                               │             │
│  │ 4. 두산 완료 대기                               │             │
│  │ 5. 반복                                       │             │
│  └─────────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────┘
        │ 발행                           │ 발행
        │ /mobile/cmd                   │ /katech/robot_command
        │                               │
        ▼ 구독                           ▼ 구독
┌──────────────────┐            ┌──────────────────┐
│ 모바일 로봇 노드   │            │ 두산 로봇 노드     │
│ (새로 작성)       │            │ (move_robot_node) │
│                  │            │ (기존 사용)       │
└──────────────────┘            └──────────────────┘
        │ 발행                           │ 발행
        │ /mobile/status                │ /doosan/status
        │                               │
        └───────────┬───────────────────┘
                    │ 구독
                    ▼
        ┌───────────────────────┐
        │   중앙 관제 노드         │
        └───────────────────────┘
```

### 3.2 필요한 토픽 정의

#### 새로 생성할 토픽

1. **모바일 로봇 명령**: `/mobile/cmd`
   - 타입: `std_msgs/Float64MultiArray`
   - 내용: `[distance, speed]`
   - 예: `[1.0, 0.2]` → 1m를 0.2m/s로 이동

2. **모바일 로봇 상태**: `/mobile/status`
   - 타입: `std_msgs/String`
   - 값: `"IDLE"`, `"MOVING"`, `"COMPLETED"`, `"ERROR"`

3. **두산 로봇 상태**: `/doosan/status`
   - 타입: `std_msgs/String`
   - 값: `"IDLE"`, `"MOVING"`, `"COMPLETED"`, `"ERROR"`

### 3.3 구현 단계

#### Step 1: 두산 로봇 상태 발행 노드 추가 (기존 코드 수정)

**파일**: `src/doosan_helper/src/move_robot_node.cpp`

현재 `move_robot_node`는 명령을 실행만 하고 상태를 발행하지 않습니다.
상태 발행 기능을 추가해야 합니다.

**수정 사항 요약**:
1. 헤더 파일 추가: `#include "std_msgs/String.h"`
2. 전역 변수 선언: `ros::Publisher status_pub;`
3. `main()` 함수에서 Publisher 초기화 및 초기 상태 발행
4. `commandCallback()` 함수에서 각 단계마다 상태 발행

**전체 수정된 코드** (`move_robot_node.cpp`):
```cpp
#include "ros/ros.h"
#include "std_msgs/Int32.h"     // 토픽 메시지 (방아쇠)
#include "std_msgs/String.h"    // 상태 발행용 메시지 (추가됨) ⭐
#include "dsr_msgs/MoveJoint.h" // 서비스 메시지 (총알)
#include <boost/bind.hpp>       // 콜백 함수에 인자를 넘기기 위해 필요

// 전역 변수: 상태 발행용 Publisher (추가됨) ⭐
ros::Publisher status_pub;

// "방아쇠"가 당겨지면(메시지가 오면) 실행될 함수
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    // 서비스 메시지(총알) 준비
    dsr_msgs::MoveJoint srv;
    bool should_call_service = false;

    // 메시지 값에 따라 다른 자세 설정
    if (msg->data == 0)
    {
        ROS_INFO("Trigger message '0' received. Preparing pose [90, 0, 90, 0, 90, -90]");
        srv.request.pos = {90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else if (msg->data == 1)
    {
        ROS_INFO("Trigger message '1' received. Preparing pose [-90, 0, 90, 0, 90, -90]");
        srv.request.pos = {-90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else if (msg->data == 99)
    {
        ROS_INFO("Trigger message 'home(99)' received. Preparing pose [0, 0, 0, 0, 0, 0]");
        srv.request.pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        should_call_service = true;
    }
    else
    {
        ROS_WARN("Received unsupported command: %d. Ignoring.", msg->data);
        should_call_service = false;
    }

    // 서비스 호출이 필요한 경우에만 실행
    if (should_call_service)
    {
        // 기타 파라미터 설정 (속도, 가속도 등)
        srv.request.vel = 30.0;
        srv.request.acc = 60.0;
        srv.request.mode = 0; // 0: MOVE_MODE_ABSOLUTE
        
        // 이동 시작 전 상태 발행 (추가됨) ⭐
        std_msgs::String status;
        status.data = "MOVING";
        status_pub.publish(status);

        // 서비스 호출 ("발사")
        ROS_INFO("Calling move_joint service...");
        if (client.call(srv))
        {
            // 성공 응답 확인
            if(srv.response.success) {
                ROS_INFO("Service call successful: Robot moved.");
                // 완료 상태 발행 (추가됨) ⭐
                status.data = "COMPLETED";
                status_pub.publish(status);
            } else {
                ROS_WARN("Service call reported failure.");
                // 에러 상태 발행 (추가됨) ⭐
                status.data = "ERROR";
                status_pub.publish(status);
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /dsr01a0912/motion/move_joint");
            // 에러 상태 발행 (추가됨) ⭐
            status.data = "ERROR";
            status_pub.publish(status);
        }
    }
}

int main(int argc, char **argv)
{
    // 노드 초기화
    ros::init(argc, argv, "move_robot_node");
    ros::NodeHandle nh;

    // 1. 상태 발행용 Publisher 초기화 (추가됨) ⭐
    status_pub = nh.advertise<std_msgs::String>("/doosan/status", 1);
    
    // 초기 상태 발행 (추가됨) ⭐
    std_msgs::String status_msg;
    status_msg.data = "IDLE";
    status_pub.publish(status_msg);
    ROS_INFO("Initial status published: IDLE");

    // 2. "두산 로봇 서비스용 전화기" 만들기 (Service Client)
    ros::ServiceClient move_client = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01a0912/motion/move_joint");

    // 서비스가 켜질 때까지 기다립니다
    ROS_INFO("Waiting for /dsr01a0912/motion/move_joint service...");
    move_client.waitForExistence();
    ROS_INFO("Service server found.");

    // 3. "방아쇠용 귀" 만들기 (Subscriber)
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32>(
        "/katech/robot_command", 10, 
        boost::bind(commandCallback, _1, boost::ref(move_client))
    );

    ROS_INFO("move_robot_node is ready. Waiting for command on /katech/robot_command topic.");

    // 대기 모드 (콜백 함수가 모든 일을 처리함)
    ros::spin();

    return 0;
}
```

**주요 변경 사항 (⭐ 표시)**:
- 헤더에 `std_msgs/String.h` 추가
- 전역 변수로 `status_pub` 선언
- `main()` 함수에서 초기 상태 "IDLE" 발행
- `commandCallback()` 함수에서 "MOVING", "COMPLETED", "ERROR" 상태 발행

#### Step 2: 모바일 로봇 ROS 노드 작성 (새 파일)

**파일**: `src/mobile_robot_control/src/mobile_robot_ros_node.py`

##### 🤔 왜 기존 코드를 그대로 사용할 수 없나요?

**핵심 이유: 두산 로봇은 "서버 형태", 모바일 로봇은 "원샷 실행 형태"**

| 비교 항목 | 두산 로봇 (`move_robot_node`) | 모바일 로봇 (`mobile_robot_twist_control`) |
|----------|------------------------------|------------------------------------------|
| **실행 방식** | 서버 형태 (계속 실행) | 원샷 형태 (한 번 실행 후 종료) |
| **명령 대기** | ✅ `ros::spin()`으로 토픽 구독 | ❌ 명령줄 인자로만 받음 |
| **ROS 통신** | ✅ 토픽/서비스 활용 | ❌ ROS 독립적 (asyncio만 사용) |
| **로봇 연결** | ✅ 드라이버가 연결 유지 | ❌ 매번 연결/해제 |
| **통합 가능** | ✅ 바로 사용 가능 | ❌ 래핑 필요 |

**두산 로봇 구조 (기존 사용 가능):**
```cpp
int main() {
    // 서비스 클라이언트 생성
    ros::ServiceClient move_client = ...;
    
    // 토픽 구독 (명령 대기)
    ros::Subscriber sub = nh.subscribe("/katech/robot_command", ...);
    
    // 무한 대기 - 프로그램이 종료되지 않음 ⭐
    ros::spin();  
}
```

**모바일 로봇 구조 (원샷 실행):**
```python
async def main():
    args = parser.parse_args()  # --distance 0.5
    controller = MobileRobotTwistController()
    await controller.connect()
    await controller.move_distance(args.distance, args.speed)
    await controller.stop()
    # 프로그램 종료 ⭐

asyncio.run(main())  # 한 번 실행하고 끝
```

**결론:** 
모바일 로봇 코드는 매번 실행/종료되므로, 통합 시스템에서 "계속 대기하며 명령을 받는" 서버 형태로 만들기 위해 ROS 노드로 래핑해야 합니다.

##### 해결 방법: ROS 노드 래퍼 작성

기존 `MobileRobotTwistController` 클래스를 **재사용**하되, ROS 토픽으로 제어하는 래퍼 노드를 만듭니다:

**전체 코드**:
```python
#!/usr/bin/env python3
"""
모바일 로봇 ROS 통합 노드
기존 MobileRobotTwistController를 ROS 토픽 기반으로 제어
"""
import rospy
import asyncio
import threading
from std_msgs.msg import String, Float64MultiArray

# 기존 클래스 임포트
from mobile_robot_twist_control import MobileRobotTwistController


class MobileRobotROSNode:
    """ROS 토픽을 통해 모바일 로봇을 제어하는 노드"""
    
    def __init__(self):
        rospy.init_node('mobile_robot_ros_node', anonymous=False)
        
        # 상태 발행자
        self.status_pub = rospy.Publisher('/mobile/status', String, queue_size=1)
        
        # 명령 구독자
        self.cmd_sub = rospy.Subscriber('/mobile/cmd', Float64MultiArray, self.command_callback)
        
        # 제어 클래스 (비동기 작업용)
        self.controller = None
        self.current_status = "IDLE"
        
        # 비동기 이벤트 루프 (별도 스레드에서 실행)
        self.loop = None
        self.loop_thread = None
        
        rospy.loginfo("🤖 모바일 로봇 ROS 노드 초기화 완료")
        self.publish_status("IDLE")
        
    def publish_status(self, status):
        """상태 발행"""
        self.current_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo(f"📊 상태 발행: {status}")
    
    def command_callback(self, msg):
        """
        명령 수신 콜백
        msg.data = [distance, speed]
        """
        if len(msg.data) < 2:
            rospy.logerr("❌ 잘못된 명령 형식. [distance, speed] 필요")
            return
        
        distance = msg.data[0]
        speed = msg.data[1]
        
        rospy.loginfo(f"🎯 명령 수신: {distance}m 이동, 속도 {speed}m/s")
        
        # 이미 실행 중이면 무시
        if self.current_status == "MOVING":
            rospy.logwarn("⚠️ 이미 이동 중입니다. 명령 무시.")
            return
        
        # 비동기 작업을 별도 스레드에서 실행
        thread = threading.Thread(target=self.execute_movement, args=(distance, speed))
        thread.daemon = True
        thread.start()
    
    def execute_movement(self, distance, speed):
        """
        별도 스레드에서 비동기 이동 실행
        """
        try:
            # 새 이벤트 루프 생성
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 이동 실행
            loop.run_until_complete(self._async_move(distance, speed))
        except Exception as e:
            rospy.logerr(f"💥 이동 중 오류: {e}")
            self.publish_status("ERROR")
        finally:
            loop.close()
    
    async def _async_move(self, distance, speed):
        """실제 비동기 이동 로직"""
        self.publish_status("MOVING")
        
        try:
            # 컨트롤러 생성 및 연결
            controller = MobileRobotTwistController(verbose=False)
            await controller.connect()
            
            # 이동 실행
            success, final_distance = await controller.move_distance(
                target_distance=distance,
                speed=speed,
                accel_distance=0.15,
                decel_distance=0.2
            )
            
            # 연결 종료
            await controller.stop()
            
            # 결과에 따라 상태 변경
            if success:
                rospy.loginfo("✅ 이동 완료!")
                self.publish_status("COMPLETED")
            else:
                rospy.logerr("❌ 이동 실패")
                self.publish_status("ERROR")
                
        except Exception as e:
            rospy.logerr(f"💥 이동 실행 오류: {e}")
            self.publish_status("ERROR")
    
    def run(self):
        """노드 실행 (ROS spin)"""
        rospy.loginfo("🚀 모바일 로봇 ROS 노드 실행 중...")
        rospy.loginfo("   명령 대기: /mobile/cmd [distance, speed]")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MobileRobotROSNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 노드 종료")
```

**실행 권한 부여** (Docker 내부):
```bash
chmod +x /root/catkin_ws/src/mobile_robot_control/src/mobile_robot_ros_node.py
```

#### Step 3: 중앙 관제 노드 작성 (새 패키지)

**패키지 생성** (Docker 내부):
```bash
cd /root/catkin_ws/src
catkin_create_pkg central_coordinator roscpp rospy std_msgs
```

**파일**: `src/central_coordinator/src/coordinator_node.py`

```python
#!/usr/bin/env python3
"""
중앙 관제 노드 (Topic 기반)
모바일 로봇과 두산 로봇을 순차적으로 제어
"""
import rospy
from std_msgs.msg import String, Int32, Float64MultiArray
import time


class CentralCoordinator:
    """중앙 관제 노드 - 두 로봇의 순차 제어"""
    
    def __init__(self):
        rospy.init_node('central_coordinator', anonymous=False)
        
        # Publishers (명령 발행)
        self.mobile_cmd_pub = rospy.Publisher('/mobile/cmd', Float64MultiArray, queue_size=1)
        self.doosan_cmd_pub = rospy.Publisher('/katech/robot_command', Int32, queue_size=1)
        
        # Subscribers (상태 수신)
        self.mobile_status_sub = rospy.Subscriber('/mobile/status', String, self.mobile_status_callback)
        self.doosan_status_sub = rospy.Subscriber('/doosan/status', String, self.doosan_status_callback)
        
        # 상태 변수
        self.mobile_status = "UNKNOWN"
        self.doosan_status = "UNKNOWN"
        
        # 시나리오 파라미터 (User가 수정 가능)
        self.mobile_distance = rospy.get_param('~mobile_distance', 1.0)  # 이동 거리 (m)
        self.mobile_speed = rospy.get_param('~mobile_speed', 0.2)        # 이동 속도 (m/s)
        self.doosan_command = rospy.get_param('~doosan_command', 99)    # 두산 명령 (99=홈)
        self.cycle_delay = rospy.get_param('~cycle_delay', 5.0)         # 사이클 간 대기 (초)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎮 중앙 관제 노드 시작")
        rospy.loginfo(f"   모바일: {self.mobile_distance}m, {self.mobile_speed}m/s")
        rospy.loginfo(f"   두산: 명령 {self.doosan_command}")
        rospy.loginfo(f"   사이클 대기: {self.cycle_delay}초")
        rospy.loginfo("=" * 60)
        
        # 초기화 대기
        rospy.sleep(2.0)
    
    def mobile_status_callback(self, msg):
        """모바일 로봇 상태 업데이트"""
        self.mobile_status = msg.data
        rospy.loginfo(f"📱 모바일 상태: {msg.data}")
    
    def doosan_status_callback(self, msg):
        """두산 로봇 상태 업데이트"""
        self.doosan_status = msg.data
        rospy.loginfo(f"🦾 두산 상태: {msg.data}")
    
    def wait_for_status(self, robot_name, target_status, timeout=60.0):
        """
        특정 로봇이 목표 상태가 될 때까지 대기
        
        Args:
            robot_name: "mobile" 또는 "doosan"
            target_status: 기다릴 상태 (예: "COMPLETED", "IDLE")
            timeout: 최대 대기 시간 (초)
        
        Returns:
            bool: 성공 여부
        """
        rospy.loginfo(f"⏳ {robot_name} 로봇이 '{target_status}' 상태가 될 때까지 대기...")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 현재 상태 확인
            if robot_name == "mobile":
                current_status = self.mobile_status
            elif robot_name == "doosan":
                current_status = self.doosan_status
            else:
                rospy.logerr(f"❌ 알 수 없는 로봇 이름: {robot_name}")
                return False
            
            # 목표 상태 도달
            if current_status == target_status:
                rospy.loginfo(f"✅ {robot_name} 로봇 상태: {target_status}")
                return True
            
            # 에러 상태 체크
            if current_status == "ERROR":
                rospy.logerr(f"❌ {robot_name} 로봇 에러 발생!")
                return False
            
            # 타임아웃 체크
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logerr(f"❌ 타임아웃 ({timeout}초 초과)")
                return False
            
            rate.sleep()
        
        return False
    
    def send_mobile_command(self, distance, speed):
        """모바일 로봇에게 이동 명령 전송"""
        rospy.loginfo(f"➡️  모바일 로봇 명령: {distance}m, {speed}m/s")
        
        cmd = Float64MultiArray()
        cmd.data = [distance, speed]
        
        # 명령 발행 (여러 번 전송하여 확실히 전달)
        for _ in range(3):
            self.mobile_cmd_pub.publish(cmd)
            rospy.sleep(0.1)
    
    def send_doosan_command(self, command_id):
        """두산 로봇에게 동작 명령 전송"""
        rospy.loginfo(f"➡️  두산 로봇 명령: {command_id}")
        
        cmd = Int32()
        cmd.data = command_id
        
        # 명령 발행 (여러 번 전송하여 확실히 전달)
        for _ in range(3):
            self.doosan_cmd_pub.publish(cmd)
            rospy.sleep(0.1)
    
    def run_sequence(self):
        """메인 시퀀스 실행"""
        cycle_count = 1
        
        while not rospy.is_shutdown():
            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo(f"🔄 사이클 {cycle_count} 시작")
            rospy.loginfo("=" * 60)
            
            # 1단계: 모바일 로봇 이동
            rospy.loginfo("\n[1/3] 📱 모바일 로봇 이동 시작")
            self.send_mobile_command(self.mobile_distance, self.mobile_speed)
            
            # 완료 대기
            if not self.wait_for_status("mobile", "COMPLETED", timeout=60.0):
                rospy.logerr("모바일 로봇 이동 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 모바일 로봇 이동 완료")
            rospy.sleep(1.0)
            
            # 2단계: 두산 로봇 동작
            rospy.loginfo("\n[2/3] 🦾 두산 로봇 동작 시작")
            self.send_doosan_command(self.doosan_command)
            
            # 완료 대기
            if not self.wait_for_status("doosan", "COMPLETED", timeout=60.0):
                rospy.logerr("두산 로봇 동작 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 두산 로봇 동작 완료")
            
            # 3단계: 사이클 완료
            rospy.loginfo(f"\n[3/3] ✅ 사이클 {cycle_count} 완료!")
            rospy.loginfo(f"⏸️  {self.cycle_delay}초 대기 후 다음 사이클 시작...\n")
            
            cycle_count += 1
            rospy.sleep(self.cycle_delay)


if __name__ == '__main__':
    try:
        coordinator = CentralCoordinator()
        coordinator.run_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 관제 노드 종료")
```

**실행 권한 부여** (Docker 내부):
```bash
chmod +x /root/catkin_ws/src/central_coordinator/src/coordinator_node.py
```

#### Step 4: Launch 파일 작성

**파일**: `src/central_coordinator/launch/integrated_system.launch`

```xml
<?xml version="1.0"?>
<launch>
    <!-- ==================== 파라미터 설정 ==================== -->
    
    <!-- 두산 로봇 설정 -->
    <arg name="doosan_model" default="a0912" doc="두산 로봇 모델명"/>
    <arg name="doosan_host" default="192.168.137.100" doc="두산 컨트롤러 IP"/>
    
    <!-- 모바일 로봇 설정 -->
    <arg name="mobile_distance" default="1.0" doc="모바일 이동 거리 (m)"/>
    <arg name="mobile_speed" default="0.2" doc="모바일 이동 속도 (m/s)"/>
    
    <!-- 두산 로봇 명령 -->
    <arg name="doosan_command" default="99" doc="두산 로봇 명령 (0, 1, 99)"/>
    
    <!-- 사이클 설정 -->
    <arg name="cycle_delay" default="5.0" doc="사이클 간 대기 시간 (초)"/>
    
    
    <!-- ==================== 두산 로봇 시스템 ==================== -->
    
    <!-- 1. 두산 로봇 드라이버 실행 -->
    <include file="$(find dsr_launcher)/launch/single_robot.launch">
        <arg name="model" value="$(arg doosan_model)"/>
        <arg name="mode" value="real"/>
        <arg name="host" value="$(arg doosan_host)"/>
    </include>
    
    <!-- 2. 두산 로봇 명령 수신 노드 (기존 코드 사용) -->
    <node name="move_robot_node" pkg="doosan_helper" type="move_robot_node" output="screen"/>
    
    
    <!-- ==================== 모바일 로봇 시스템 ==================== -->
    
    <!-- 3. 모바일 로봇 ROS 노드 (새로 작성한 노드) -->
    <node name="mobile_robot_ros_node" pkg="mobile_robot_control" type="mobile_robot_ros_node.py" output="screen"/>
    
    
    <!-- ==================== 중앙 관제 노드 ==================== -->
    
    <!-- 4. 중앙 관제 노드 (지연 실행: 다른 노드들이 준비될 때까지 5초 대기) -->
    <node name="central_coordinator" pkg="central_coordinator" type="coordinator_node.py" output="screen"
          launch-prefix="bash -c 'sleep 5; $0 $@'">
        <!-- 파라미터 전달 -->
        <param name="mobile_distance" value="$(arg mobile_distance)"/>
        <param name="mobile_speed" value="$(arg mobile_speed)"/>
        <param name="doosan_command" value="$(arg doosan_command)"/>
        <param name="cycle_delay" value="$(arg cycle_delay)"/>
    </node>
    
</launch>
```

### 3.4 CMakeLists.txt 수정 (필요 시)

**파일**: `src/central_coordinator/CMakeLists.txt`

Python 노드만 사용하므로 특별한 수정은 불필요하지만, Python 스크립트를 설치하려면:

```cmake
catkin_install_python(PROGRAMS
  src/coordinator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 3.5 빌드 및 실행

#### 빌드 (Docker 내부, User가 실행)
```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

#### 전체 시스템 실행
```bash
# 단일 명령으로 전체 시스템 실행
roslaunch central_coordinator integrated_system.launch
```

#### 파라미터 커스터마이징 실행
```bash
# 모바일 2m 이동, 속도 0.3m/s, 두산 명령 0번, 사이클 간 10초 대기
roslaunch central_coordinator integrated_system.launch \
    mobile_distance:=2.0 \
    mobile_speed:=0.3 \
    doosan_command:=0 \
    cycle_delay:=10.0
```

### 3.6 디버깅 및 모니터링

#### 터미널 1: Launch 파일 실행
```bash
roslaunch central_coordinator integrated_system.launch
```

#### 터미널 2: 상태 모니터링
```bash
# 모바일 로봇 상태 확인
rostopic echo /mobile/status

# 두산 로봇 상태 확인
rostopic echo /doosan/status

# 모든 토픽 목록
rostopic list

# 노드 그래프 시각화
rqt_graph
```

#### 수동 명령 테스트
```bash
# 모바일 로봇에게 직접 명령
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.5, 0.1]"

# 두산 로봇에게 직접 명령
rostopic pub /katech/robot_command std_msgs/Int32 "data: 99"
```

---

## 4. 방안 B: 고급 Action 기반 통합

이 방안은 ROS의 **Action Server/Client** 패턴을 사용하여 더 정교한 제어를 구현합니다.
자세한 내용은 기존 문서 `work_instructions.md`를 참고하세요.

### 4.1 주요 차이점

| 특징 | Topic 기반 (방안 A) | Action 기반 (방안 B) |
|------|-------------------|---------------------|
| 피드백 | 상태만 (IDLE, MOVING 등) | 진행률, 남은 거리, 현재 단계 등 상세 정보 |
| 취소 기능 | 없음 (구현 필요) | 기본 제공 (preempt) |
| 결과 확인 | 상태 변화로 간접 확인 | 명시적 결과 메시지 |
| 타임아웃 | 수동 구현 | 클라이언트에 내장 |
| 재사용성 | 낮음 | 높음 (표준 패턴) |

### 4.2 필요한 작업 요약

1. **Action 메시지 정의** (`robot_interfaces` 패키지)
   - `MoveMobile.action`: 모바일 로봇 이동
   - `MoveArmSequence.action`: 두산 로봇 동작

2. **Action Server 구현**
   - 모바일: `mobile_robot_action_server.py`
   - 두산: `doosan_action_server.cpp`

3. **Action Client 구현** (중앙 관제 노드)
   - `coordinator_node.cpp` (C++) 또는 `coordinator_node.py` (Python)

상세 구현은 `work_instructions.md`를 참고하세요.

---

## 5. 실행 및 테스트

### 5.1 시스템 시작 전 체크리스트

- [ ] Docker 컨테이너 실행 중
- [ ] 두산 로봇 전원 켜짐 (IP: 192.168.137.100)
- [ ] 모바일 로봇 전원 켜짐 (IP: 169.254.128.2)
- [ ] 네트워크 연결 확인 (`ping` 테스트)
- [ ] 로봇 주변 안전 공간 확보
- [ ] **빌드 완료** (`catkin_make`)

### 5.2 단계별 테스트

#### 1단계: 개별 노드 테스트

**터미널 1: 두산 로봇 드라이버만 실행**
```bash
roslaunch dsr_launcher single_robot.launch model:=a0912 mode:=real host:=192.168.137.100
```

**터미널 2: move_robot_node 실행**
```bash
rosrun doosan_helper move_robot_node
```

**터미널 3: 상태 확인**
```bash
rostopic echo /doosan/status
```

**터미널 4: 수동 명령 테스트**
```bash
rostopic pub /katech/robot_command std_msgs/Int32 "data: 99"
```

→ 두산 로봇이 홈으로 이동하고, `/doosan/status`에 "MOVING" → "COMPLETED" 발행 확인

#### 2단계: 모바일 노드 테스트

**터미널 1: 모바일 ROS 노드 실행**
```bash
rosrun mobile_robot_control mobile_robot_ros_node.py
```

**터미널 2: 상태 확인**
```bash
rostopic echo /mobile/status
```

**터미널 3: 수동 명령 테스트**
```bash
# 0.5m를 0.1m/s로 이동
rostopic pub /mobile/cmd std_msgs/Float64MultiArray "data: [0.5, 0.1]"
```

→ 모바일 로봇이 이동하고, `/mobile/status`에 "MOVING" → "COMPLETED" 발행 확인

#### 3단계: 통합 시스템 테스트

**전체 시스템 실행**
```bash
roslaunch central_coordinator integrated_system.launch
```

→ 모바일 이동 → 두산 동작 → 반복 확인

### 5.3 문제 해결 (Troubleshooting)

#### 문제 1: 모바일 로봇 연결 실패
```
❌ 로봇 연결 실패: Connection refused
```

**해결 방법**:
1. 모바일 로봇 전원 확인
2. IP 주소 확인 (`ping 169.254.128.2`)
3. `mobile_robot_ros_node.py` 내부의 IP 설정 확인

#### 문제 2: 두산 로봇 서비스 호출 실패
```
Failed to call service /dsr01a0912/motion/move_joint
```

**해결 방법**:
1. 두산 드라이버 실행 확인 (`rosnode list | grep dsr`)
2. 서비스 목록 확인 (`rosservice list`)
3. 로봇 제어기 전원 및 네트워크 확인

#### 문제 3: 상태가 업데이트되지 않음
```
⏳ mobile 로봇이 'COMPLETED' 상태가 될 때까지 대기...
(무한 대기)
```

**해결 방법**:
1. 해당 노드가 실행 중인지 확인 (`rosnode list`)
2. 토픽이 발행되는지 확인 (`rostopic hz /mobile/status`)
3. 로그에서 에러 메시지 확인

#### 문제 4: Docker 컨테이너에서 빌드 실패
```
Could not find a package configuration file provided by "..."
```

**해결 방법**:
1. `package.xml`에 의존성 추가 확인
2. `rosdep install --from-paths src --ignore-src -r -y` 실행
3. `catkin_make clean` 후 재빌드

### 5.4 성능 최적화 팁

1. **네트워크 지연 최소화**
   - ROS_MASTER_URI를 localhost로 설정 (Docker 내부)
   - QoS 설정 (ROS2 전환 시)

2. **동작 매끄럽게 하기**
   - 모바일 가감속 파라미터 조정 (`--accel`, `--decel`)
   - 두산 로봇 속도/가속도 조정 (`srv.request.vel`, `srv.request.acc`)

3. **안전성 향상**
   - 타임아웃 값 적절히 설정
   - 에러 발생 시 로봇 정지 로직 추가
   - E-stop 통합 (향후 과제)

---

## 6. 향후 확장 계획

### 6.1 단기 확장 (1-2주)

- [ ] 더 복잡한 시나리오 추가 (예: 모바일 왕복, 두산 다양한 자세)
- [ ] 로봇 상태 GUI 모니터링 (rqt_plot, RViz)
- [ ] 에러 복구 로직 강화

### 6.2 중기 확장 (1-2개월)

- [ ] ROS2로 마이그레이션 (Humble)
- [ ] Action 기반 시스템으로 전환 (방안 B)
- [ ] 센서 통합 (카메라, 라이다)
- [ ] 자율 내비게이션 (SLAM)

### 6.3 장기 확장 (3-6개월)

- [ ] 멀티 로봇 협업 (모바일 2대 + 두산 2대)
- [ ] AI 기반 작업 계획 (Task Planning)
- [ ] 디지털 트윈 (Gazebo 시뮬레이션)
- [ ] 클라우드 연동 (Fleet Management)

---

## 7. 참고 자료

### 7.1 관련 문서

- `Project.md`: 프로젝트 전체 개요
- `work_instructions.md`: Action 기반 구현 상세 가이드 (방안 B)
- `README.md`: 프로젝트 빠른 시작 가이드

### 7.2 유용한 ROS 명령어 모음

```bash
# 노드 관련
rosnode list                    # 실행 중인 노드 목록
rosnode info /노드이름           # 노드 정보 확인
rosnode kill /노드이름           # 노드 강제 종료

# 토픽 관련
rostopic list                   # 토픽 목록
rostopic echo /토픽이름          # 토픽 메시지 실시간 확인
rostopic hz /토픽이름            # 토픽 발행 주기 확인
rostopic pub /토픽이름 타입 데이터  # 수동 메시지 발행

# 서비스 관련
rosservice list                 # 서비스 목록
rosservice call /서비스이름 인자  # 서비스 호출

# 시스템 관련
roscore                         # ROS 마스터 시작
rqt_graph                       # 노드 그래프 시각화
rqt_console                     # 로그 뷰어
```

### 7.3 디버깅 도구

```bash
# 로그 레벨 변경
rosservice call /node/set_logger_level ros.package DEBUG

# 네트워크 확인
rostopic bw /topic              # 토픽 대역폭 확인
rostopic delay /topic           # 토픽 지연 확인

# 녹화 및 재생
rosbag record -a                # 모든 토픽 녹화
rosbag play file.bag            # 재생
```

---

## 8. 결론

이 문서는 **방안 A (Topic 기반)** 를 중심으로 통합 관제 시스템 구축 방법을 상세히 설명했습니다.

### 권장 작업 순서

1. ✅ **Step 1 완료**: `move_robot_node.cpp` 수정 (상태 발행 기능 추가)
2. ✅ **Step 2 완료**: `mobile_robot_ros_node.py` 작성
3. ✅ **Step 3 완료**: `coordinator_node.py` 작성
4. ✅ **Step 4 완료**: `integrated_system.launch` 작성
5. 🔨 **User 작업**: `catkin_make`로 빌드
6. 🧪 **단계별 테스트**: 개별 노드 → 통합 시스템
7. 🚀 **실전 운영**: 파라미터 조정 및 최적화

### 추가 질문이나 문제 발생 시

- ROS 로그 확인: `/root/.ros/log/latest/` 디렉터리
- 이 문서의 "문제 해결" 섹션 참고
- `rostopic`, `rosnode` 명령어로 실시간 디버깅

**성공적인 통합 시스템 구축을 기원합니다! 🎉**

