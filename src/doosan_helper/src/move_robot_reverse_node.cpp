#include "ros/ros.h"
#include "std_msgs/Int32.h"     // 토픽 메시지 (방아쇠)
#include "dsr_msgs/MoveJoint.h" // 서비스 메시지 (총알)
#include <boost/bind.hpp>      // 콜백 함수에 인자를 넘기기 위해 필요

// "방아쇠"가 당겨지면(메시지가 오면) 실행될 함수
// msg: 수신된 토픽 메시지 (e.g., 1)
// client: 두산 로봇 서비스에 연결된 "전화기"
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    // 메시지 값이 1일 때만 작동
    if (msg->data == 0)
    {
        ROS_INFO("Trigger message '1' received. Calling move_joint service...");

        // 1. 서비스 메시지(총알) 준비
        dsr_msgs::MoveJoint srv;

        // 2. 총알 장전 (사용자님이 원하는 관절 각도)
        // C++ std::vector를 사용하여 6개의 각도(Degree)를 설정합니다.
        srv.request.pos = {-90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        
        // 기타 파라미터 설정 (속도, 가속도 등)
        srv.request.vel = 30.0;
        srv.request.acc = 60.0;
        srv.request.mode = 0; // 0: MOVE_MODE_ABSOLUTE

        // 3. 서비스 호출 ("발사")
        if (client.call(srv))
        {
            // 성공 응답 확인
            if(srv.response.success) {
                ROS_INFO("Service call successful: Robot reverse moved.");
            } else {
                ROS_WARN("Service call reported failure.");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /dsr01a0912/motion/move_joint");
        }
    }
}

int main(int argc, char **argv)
{
    // 노드 초기화. 이름: "move_robot_reverse_node"
    ros::init(argc, argv, "move_robot_reverse_node");
    ros::NodeHandle nh;

    // 1. "두산 로봇 서비스용 전화기" 만들기 (Service Client)
    //    이 노드는 "/dsr01a0912/motion/move_joint" 서비스에 전화를 겁니다.
    ros::ServiceClient move_client = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01a0912/motion/move_joint");

    // (선택사항) 서비스가 켜질 때까지 기다립니다.
    ROS_INFO("Waiting for /dsr01a0912/motion/move_joint service...");
    move_client.waitForExistence();
    ROS_INFO("Service server found.");

    // 2. "방아쇠용 귀" 만들기 (Subscriber)
    //    "/katech/robot_command" 토픽을 듣습니다.
    //    메시지가 오면, commandCallback 함수를 실행합니다.
    //    (중요!) boost::bind를 사용해, 콜백 함수에 "전화기"(move_client)를 함께 넘겨줍니다.
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32>(
        "/katech/robot_command", 10, 
        boost::bind(commandCallback, _1, boost::ref(move_client))
    );

    ROS_INFO("move_robot_reverse_node is ready. Waiting for command on /katech/robot_command topic.");

    // 대기 모드 (콜백 함수가 모든 일을 처리함)
    ros::spin();

    return 0;
}