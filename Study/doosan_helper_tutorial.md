# 두산 로봇 명령 노드 학습 가이드: `move_robot_node.cpp`

## 📚 학습 목표
- `/dsr_robot/robot_cmd` 토픽이 실제 두산 로봇의 `MoveJoint` 서비스 호출로 이어지는 과정을 이해한다.
- ROS C++ 노드 구조와 서비스 클라이언트 사용 방법을 익힌다.
- 명령 ID별 관절 각도 매핑과 안전 파라미터(속도, 가속도 등)를 파악한다.
- 실제 장비 연결 전, 토픽/서비스를 활용한 모의 테스트 방법을 습득한다.

## 1. 파일 개요
- 위치: `src/doosan_helper/src/move_robot_node.cpp`
- 역할: 중앙 관제 노드가 발행하는 명령 토픽을 구독해 두산 로봇의 `move_joint` 서비스를 호출하고, 실행 결과를 상태 토픽으로 공유
- 주요 인터페이스
  - 구독: `/dsr_robot/robot_cmd` (`std_msgs::Int32`)
  - 퍼블리시: `/doosan/status` (`std_msgs::String`)
  - 서비스 클라이언트: `/dsr01a0912/motion/move_joint` (`dsr_msgs::MoveJoint`)

## 2. 전체 코드 구조

```1:121:src/doosan_helper/src/move_robot_node.cpp
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "dsr_msgs/MoveJoint.h"
#include <boost/bind.hpp>

ros::Publisher status_pub;

void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    ...
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot_node");
    ros::NodeHandle nh;
    ...
    ros::ServiceClient move_client = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01a0912/motion/move_joint");
    move_client.waitForExistence();
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32>(
        "/dsr_robot/robot_cmd", 10,
        boost::bind(commandCallback, _1, boost::ref(move_client))
    );
    ros::spin();
}
```

## 3. 헤더 및 전역 리소스 이해
- 표준 메시지(`Int32`, `String`)와 두산 로봇 서비스 메시지(`MoveJoint`)를 포함
- `status_pub` 전역 선언: 콜백과 `main`에서 공유하는 퍼블리셔
- `boost::bind`: 콜백 함수에 서비스 클라이언트 참조를 전달하기 위해 사용

## 4. 콜백 함수 상세 분석

```13:78:src/doosan_helper/src/move_robot_node.cpp
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    dsr_msgs::MoveJoint srv;
    bool should_call_service = false;

    if (msg->data == 0)
    {
        srv.request.pos = {90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else if (msg->data == 1)
    {
        srv.request.pos = {-90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else if (msg->data == 99)
    {
        srv.request.pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        should_call_service = true;
    }
    else
    {
        ROS_WARN("Received unsupported command: %d. Ignoring.", msg->data);
    }

    if (should_call_service)
    {
        srv.request.vel = 30.0;
        srv.request.acc = 60.0;
        srv.request.mode = 0;

        std_msgs::String status;
        status.data = "MOVING";
        status_pub.publish(status);

        if (client.call(srv))
        {
            if(srv.response.success) {
                status.data = "COMPLETED";
            } else {
                status.data = "ERROR";
            }
        }
        else
        {
            status.data = "ERROR";
        }
        status_pub.publish(status);
    }
}
```

### 핵심 포인트
- **명령 ID 매핑**
  - 0 → 자세0 (좌우 대칭 자세)
  - 1 → 자세1 (작업 자세)
  - 99 → 홈 자세
- **안전 파라미터**
  - `vel = 30.0` (deg/s)
  - `acc = 60.0` (deg/s²)
  - `mode = 0` (절대 위치)
- **상태 업데이트**
  - 서비스 호출 전: `MOVING`
  - 성공 응답: `COMPLETED`
  - 실패/예외: `ERROR`

## 5. 메인 함수 흐름

```82:118:src/doosan_helper/src/move_robot_node.cpp
ros::init(argc, argv, "move_robot_node");
ros::NodeHandle nh;
status_pub = nh.advertise<std_msgs::String>("/doosan/status", 1);
std_msgs::String status_msg;
status_msg.data = "IDLE";
status_pub.publish(status_msg);

ros::ServiceClient move_client = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01a0912/motion/move_joint");
move_client.waitForExistence();

ros::Subscriber sub = nh.subscribe<std_msgs::Int32>(
    "/dsr_robot/robot_cmd", 10,
    boost::bind(commandCallback, _1, boost::ref(move_client))
);

ros::spin();
```

### 주요 단계
1. **노드 초기화**: 이름 `move_robot_node`
2. **상태 퍼블리셔 생성 및 초기 상태 발행**: 관제 노드가 즉시 IDLE 상태를 확인할 수 있도록 함
3. **서비스 클라이언트 준비**: 두산 드라이버 서비스가 준비될 때까지 대기 (`waitForExistence`)
4. **토픽 구독**: 큐 크기 10, 콜백에 서비스 클라이언트 참조 전달
5. **이벤트 루프 진입**: `ros::spin()`으로 콜백 기반 처리 시작

## 6. 안전 및 예외 처리 고려사항
- 지원되지 않는 명령 ID 수신 시 경고 로그 출력 후 무시
- 서비스 호출 실패(`client.call`이 false) 시 즉시 `ERROR` 상태 발행
- 응답에서 `success` 플래그 확인하여 실패 처리 분리
- 초기 `IDLE`, 실행 중 `MOVING`, 최종 `COMPLETED/ERROR` 상태를 사용자가 모니터링하도록 보장

## 7. 실습 과제

### 과제 1: 명령 ID 확장 실습
1. `commandCallback`에 `else if (msg->data == 2)` 블록을 추가해 새로운 자세를 정의해보세요.
2. 안전을 위해 관절 각도를 현실적인 범위 내로 제한하고, 로그에 자세명을 명시하세요.
3. 관제 노드에서 해당 ID를 발행하도록 수정한 뒤 동작을 검증하세요.

### 과제 2: 서비스 호출 모의 테스트
1. 실제 로봇 없이 테스트하려면 `rosservice list`로 `move_joint` 서비스가 제공되는지 확인하세요.
2. 드라이버를 실행하기 어렵다면, 임시로 서비스 서버를 모킹(mock)하여 응답을 반환하도록 구성해보세요.
3. 모킹 시나리오에서 `srv.response.success = false`를 반환하여 `ERROR` 상태 발행 로직을 검증하세요.

### 과제 3: 속도/가속도 파라미터 튜닝
1. `vel`, `acc` 값을 런치 파일의 파라미터로 노출하도록 개선안을 설계해보세요.
2. 파라미터화된 버전을 구현하면, 관제 노드와 동일하게 런치 인자를 통해 안전하게 조정할 수 있습니다.

## 8. 이해도 체크리스트
- [ ] 명령 ID별 관절 각도 매핑을 설명할 수 있다.
- [ ] 서비스 호출 전후로 상태 토픽이 어떻게 변하는지 알고 있다.
- [ ] `boost::bind`를 사용하는 이유를 이해했다.
- [ ] 서비스 실패 시 `ERROR` 상태가 발행되는 경로를 추적할 수 있다.
- [ ] 속도/가속도 파라미터가 동작 안전성에 미치는 영향을 설명할 수 있다.

## 9. 다음 학습 단계 예고
- 두산 드라이버 런치 파일(`dsr_launcher/single_robot.launch`)과 `dsr_msgs/MoveJoint.srv` 정의 분석
- 목표: 서비스 서버 측 인터페이스를 이해하고, 명령이 하위 레벨로 전달되는 과정을 파악

---

**작성자**: AI Assistant  
**최종 수정일**: 2025-01-XX  
**대상 독자**: ROS 초보자


