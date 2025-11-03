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
        ROS_INFO("[1/2] ➡️ 모바일 로봇 이동 시작 (0.3m, 0.2m/s)");
        robot_interfaces::MoveMobileGoal goal;
        goal.target_distance = 0.3;
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