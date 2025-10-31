#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>

/**
 * @brief 협동로봇-모바일로봇 협업 코디네이터 노드
 * 
 * 이 노드는 모바일 로봇과 협동로봇 간의 순차적 협업을 조율합니다.
 * 
 * 동작 흐름:
 * 1. 모바일 로봇으로부터 이동 완료 메시지 수신 (/katech/mobile_ready)
 * 2. 모바일 로봇의 완전 정지 확인 (Twist 속도 모니터링)
 * 3. 협동로봇에 명령 전송 (/katech/robot_command)
 * 4. 협동로봇 동작 완료 확인 (JointState 모니터링)
 * 5. 모바일 로봇에 완료 메시지 전송 (/katech/cobot_done)
 * 
 * Author: KATECH Robotics Team
 */

class CobotCoordinator
{
public:
    CobotCoordinator() : nh_("~"), 
                         mobile_ready_(false),
                         cobot_busy_(false),
                         mobile_stopped_(false)
    {
        // 파라미터 로드
        nh_.param<double>("velocity_threshold", velocity_threshold_, 0.01); // m/s
        nh_.param<double>("position_tolerance", position_tolerance_, 0.02);  // rad (약 1도)
        nh_.param<double>("stability_duration", stability_duration_, 1.0);   // 초
        nh_.param<int>("robot_command", robot_command_, 0);                 // 전송할 명령 (기본값: 0)
        
        // 목표 관절 자세 (data=0에 대응하는 자세)
        target_joint_positions_ = {
            90.0 * M_PI / 180.0,   // Joint 0: 90도
            0.0,                    // Joint 1: 0도
            90.0 * M_PI / 180.0,   // Joint 2: 90도
            0.0,                    // Joint 3: 0도
            90.0 * M_PI / 180.0,   // Joint 4: 90도
            -90.0 * M_PI / 180.0   // Joint 5: -90도
        };
        
        // Publisher 설정
        robot_command_pub_ = nh_.advertise<std_msgs::Int32>("/katech/robot_command", 1, true);
        cobot_done_pub_ = nh_.advertise<std_msgs::Bool>("/katech/cobot_done", 1, true);
        
        // Subscriber 설정
        mobile_ready_sub_ = nh_.subscribe("/katech/mobile_ready", 1, 
                                          &CobotCoordinator::mobileReadyCallback, this);
        twist_sub_ = nh_.subscribe("/woosh/twist", 10, 
                                   &CobotCoordinator::twistCallback, this);
        joint_state_sub_ = nh_.subscribe("/dsr01a0912/joint_states", 10, 
                                         &CobotCoordinator::jointStateCallback, this);
        
        ROS_INFO("==================================================");
        ROS_INFO("🤝 Cobot Coordinator Node 시작");
        ROS_INFO("==================================================");
        ROS_INFO("파라미터:");
        ROS_INFO("  - 속도 임계값: %.3f m/s", velocity_threshold_);
        ROS_INFO("  - 위치 허용 오차: %.3f rad (%.1f도)", 
                 position_tolerance_, position_tolerance_ * 180.0 / M_PI);
        ROS_INFO("  - 안정화 시간: %.1f초", stability_duration_);
        ROS_INFO("  - 로봇 명령: %d", robot_command_);
        ROS_INFO("==================================================");
        ROS_INFO("토픽:");
        ROS_INFO("  - 구독: /katech/mobile_ready");
        ROS_INFO("  - 구독: /woosh/twist");
        ROS_INFO("  - 구독: /dsr01a0912/joint_states");
        ROS_INFO("  - 발행: /katech/robot_command");
        ROS_INFO("  - 발행: /katech/cobot_done");
        ROS_INFO("==================================================");
        ROS_INFO("⏳ 모바일 로봇 이동 완료 대기 중...");
    }
    
    /**
     * @brief 모바일 로봇 준비 완료 콜백
     */
    void mobileReadyCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data && !mobile_ready_ && !cobot_busy_)
        {
            ROS_INFO("==================================================");
            ROS_INFO("✅ 모바일 로봇 이동 완료 메시지 수신!");
            ROS_INFO("==================================================");
            mobile_ready_ = true;
            mobile_stopped_ = false;
            last_velocity_ = 999.0;  // 초기값 설정
            
            // 정지 확인 시작
            checkMobileStoppedAndProceed();
        }
    }
    
    /**
     * @brief 모바일 로봇 Twist 속도 콜백
     */
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 선속도 크기 계산
        double linear_speed = std::sqrt(
            msg->linear.x * msg->linear.x + 
            msg->linear.y * msg->linear.y + 
            msg->linear.z * msg->linear.z
        );
        
        // 각속도 크기 계산
        double angular_speed = std::fabs(msg->angular.z);
        
        last_velocity_ = std::max(linear_speed, angular_speed);
    }
    
    /**
     * @brief 협동로봇 관절 상태 콜백
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        last_joint_state_ = *msg;
    }
    
    /**
     * @brief 모바일 로봇 정지 확인 및 협동로봇 구동 진행
     */
    void checkMobileStoppedAndProceed()
    {
        if (!mobile_ready_ || cobot_busy_) return;
        
        ROS_INFO("🔍 모바일 로봇 정지 상태 확인 중...");
        ROS_INFO("   (%.1f초 동안 속도 < %.3f m/s 유지 필요)", 
                 stability_duration_, velocity_threshold_);
        
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10);  // 10Hz
        
        while (ros::ok())
        {
            ros::spinOnce();
            
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            // 속도 체크
            if (last_velocity_ < velocity_threshold_)
            {
                if (elapsed >= stability_duration_)
                {
                    ROS_INFO("✅ 모바일 로봇 완전 정지 확인! (%.1f초 경과)", elapsed);
                    mobile_stopped_ = true;
                    break;
                }
            }
            else
            {
                // 아직 움직이고 있으면 타이머 리셋
                start_time = ros::Time::now();
            }
            
            // 주기적 상태 출력
            if (static_cast<int>(elapsed * 10) % 10 == 0)
            {
                ROS_INFO("   현재 속도: %.3f m/s | 경과: %.1f초", last_velocity_, elapsed);
            }
            
            rate.sleep();
        }
        
        if (mobile_stopped_)
        {
            // 협동로봇 구동 시작
            triggerCobotMotion();
        }
    }
    
    /**
     * @brief 협동로봇 동작 트리거
     */
    void triggerCobotMotion()
    {
        cobot_busy_ = true;
        
        ROS_INFO("==================================================");
        ROS_INFO("🤖 협동로봇 동작 시작");
        ROS_INFO("==================================================");
        ROS_INFO("명령 전송: /katech/robot_command = %d", robot_command_);
        
        // 명령 메시지 발행
        std_msgs::Int32 cmd_msg;
        cmd_msg.data = robot_command_;
        robot_command_pub_.publish(cmd_msg);
        
        ROS_INFO("✅ 명령 발행 완료");
        ROS_INFO("⏳ 협동로봇 동작 완료 대기 중...");
        
        // 동작 완료 대기
        ros::Duration(2.0).sleep();  // 동작 시작 대기
        waitForCobotCompletion();
    }
    
    /**
     * @brief 협동로봇 동작 완료 대기
     */
    void waitForCobotCompletion()
    {
        ros::Rate rate(10);  // 10Hz
        ros::Time start_time = ros::Time::now();
        double timeout = 60.0;  // 60초 타임아웃
        
        while (ros::ok())
        {
            ros::spinOnce();
            
            double elapsed = (ros::Time::now() - start_time).toSec();
            
            // 타임아웃 체크
            if (elapsed > timeout)
            {
                ROS_ERROR("❌ 협동로봇 동작 타임아웃 (%.0f초)", timeout);
                cobot_busy_ = false;
                mobile_ready_ = false;
                return;
            }
            
            // 관절 상태 체크
            if (last_joint_state_.position.size() >= target_joint_positions_.size())
            {
                if (isAtTargetPosition())
                {
                    ROS_INFO("✅ 협동로봇 목표 자세 도달! (%.1f초 경과)", elapsed);
                    notifyMobileRobot();
                    break;
                }
            }
            
            // 주기적 상태 출력 (5초마다)
            if (static_cast<int>(elapsed) % 5 == 0 && static_cast<int>(elapsed * 10) % 10 == 0)
            {
                ROS_INFO("   동작 진행 중... (%.1f초 경과)", elapsed);
            }
            
            rate.sleep();
        }
        
        cobot_busy_ = false;
        mobile_ready_ = false;
        
        ROS_INFO("==================================================");
        ROS_INFO("⏳ 다음 협업 시퀀스 대기 중...");
        ROS_INFO("==================================================");
    }
    
    /**
     * @brief 목표 관절 자세 도달 확인
     */
    bool isAtTargetPosition()
    {
        if (last_joint_state_.position.size() < target_joint_positions_.size())
        {
            return false;
        }
        
        for (size_t i = 0; i < target_joint_positions_.size(); ++i)
        {
            double error = std::fabs(last_joint_state_.position[i] - target_joint_positions_[i]);
            if (error > position_tolerance_)
            {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * @brief 모바일 로봇에 완료 알림
     */
    void notifyMobileRobot()
    {
        ROS_INFO("==================================================");
        ROS_INFO("📢 모바일 로봇에 작업 완료 알림 전송");
        ROS_INFO("==================================================");
        
        std_msgs::Bool done_msg;
        done_msg.data = true;
        cobot_done_pub_.publish(done_msg);
        
        ROS_INFO("✅ 완료 메시지 발행: /katech/cobot_done");
    }

private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher robot_command_pub_;
    ros::Publisher cobot_done_pub_;
    
    // Subscribers
    ros::Subscriber mobile_ready_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber joint_state_sub_;
    
    // 상태 플래그
    bool mobile_ready_;
    bool cobot_busy_;
    bool mobile_stopped_;
    
    // 센서 데이터
    double last_velocity_;
    sensor_msgs::JointState last_joint_state_;
    std::vector<double> target_joint_positions_;
    
    // 파라미터
    double velocity_threshold_;
    double position_tolerance_;
    double stability_duration_;
    int robot_command_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_coordinator_node");
    
    CobotCoordinator coordinator;
    
    ros::spin();
    
    return 0;
}

