#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    // 노드 초기화. 이름: "trigger_one_node"
    ros::init(argc, argv, "trigger_one_node");
    ros::NodeHandle nh;

    // 1. "명령용 확성기" 만들기 (Publisher)
    //    "/katech/robot_command" 토픽에 Int32 메시지를 보냅니다.
    ros::Publisher command_pub = nh.advertise<std_msgs::Int32>("/katech/robot_command", 1, true); // latch=true

    // 잠깐 기다려서 Publisher가 준비될 시간을 줍니다.
    ros::Duration(0.5).sleep(); 

    // 2. 메시지 준비 및 '1' 설정
    std_msgs::Int32 msg;
    msg.data = 1;

    // 3. 메시지 발행 ("01이라고 외치기")
    ROS_INFO("Publishing command '1' to /katech/robot_command");
    command_pub.publish(msg);

    // 발행 후 바로 종료되지 않도록 잠시 대기 (선택사항)
    ros::Duration(0.5).sleep(); 

    ROS_INFO("Command '1' published. Exiting.");

    return 0;
}