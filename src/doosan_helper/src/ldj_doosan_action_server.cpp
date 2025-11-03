#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_interfaces/MoveArmSequenceAction.h>
#include <std_msgs/String.h>
#include "dsr_msgs/MoveJoint.h"

class LDJDoosanActionServer
{
public:
    LDJDoosanActionServer(std::string name) :
        as_(nh_, name, boost::bind(&LDJDoosanActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        // 모바일 로봇의 상태를 구독
        mobile_status_sub_ = nh_.subscribe("/mobile_robot/status", 1, &LDJDoosanActionServer::mobileStatusCB, this);

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
        if (pos.size() !=6)
        {
            ROS_ERROR("moveArm: pos 벡터는 6개의 요소를 가져야 합니다. 현재 크기: %zu", pos.size());
            return false;
        }
        dsr_msgs::MoveJoint srv;
        std::copy(pos.begin(), pos.end(), srv.request.pos.begin());
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
    LDJDoosanActionServer server("move_arm_sequence");
    ros::spin();
    return 0;
}