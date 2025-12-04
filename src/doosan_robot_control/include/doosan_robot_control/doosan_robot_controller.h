/**
 * @file doosan_robot_controller.h
 * @brief Doosan Robot Controller - 두산로봇 제어 클래스 헤더
 * 
 * 확장 가능한 두산로봇 제어 프레임워크.
 * 기본 연결, 상태 모니터링, ROS 통합을 제공합니다.
 * 
 * @author KATECH Robotics Team
 * @date 2025-10-29
 * @license MIT
 */

#ifndef DOOSAN_ROBOT_CONTROLLER_H
#define DOOSAN_ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include <vector>
#include <array>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

// Doosan Robot Messages (dsr_msgs에서 제공하는 메시지들)
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/RobotError.h>
#include <dsr_msgs/ModbusState.h>

// Doosan Robot Services (필요한 서비스들)
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/SetRobotMode.h>
#include <dsr_msgs/GetRobotState.h>
#include <dsr_msgs/MoveStop.h>
#include <dsr_msgs/RobotStop.h>

namespace doosan_robot_control {

/**
 * @brief 로봇 연결 설정 구조체
 */
struct RobotConfig {
    std::string robot_id;          ///< 로봇 네임스페이스/ID (예: "dsr01")
    std::string robot_model;       ///< 로봇 모델 (예: "a0912", "m1013")
    std::string host;              ///< 로봇 IP 주소
    int port;                      ///< 로봇 포트 (기본: 12345)
    std::string mode;              ///< 동작 모드 ("real" or "virtual")
    double control_rate;           ///< 제어 루프 주기 (Hz)
    
    // 기본값 설정
    RobotConfig() 
        : robot_id("dsr01")
        , robot_model("a0912")
        , host("192.168.137.100")
        , port(12345)
        , mode("real")
        , control_rate(10.0)
    {}
};

/**
 * @brief 로봇 상태 정보 구조체
 */
struct RobotStatus {
    bool is_connected;             ///< 연결 상태
    bool is_moving;                ///< 이동 중 여부
    bool has_error;                ///< 에러 발생 여부
    int robot_state;               ///< 로봇 상태 코드
    std::string error_message;     ///< 에러 메시지
    std::array<double, 6> joint_positions;  ///< 현재 관절 위치 (rad)
    std::array<double, 6> joint_velocities; ///< 현재 관절 속도 (rad/s)
    
    RobotStatus() 
        : is_connected(false)
        , is_moving(false)
        , has_error(false)
        , robot_state(0)
        , error_message("")
    {
        joint_positions.fill(0.0);
        joint_velocities.fill(0.0);
    }
};

/**
 * @class DoosanRobotController
 * @brief 두산로봇 제어 메인 클래스
 * 
 * 두산로봇 SDK를 래핑하여 사용하기 쉬운 인터페이스를 제공합니다.
 * 연결 관리, 상태 모니터링, 기본 동작 제어 기능을 포함합니다.
 */
class DoosanRobotController {
public:
    /**
     * @brief 생성자
     * @param nh ROS 노드 핸들
     * @param config 로봇 설정
     */
    explicit DoosanRobotController(ros::NodeHandle& nh, const RobotConfig& config);
    
    /**
     * @brief 소멸자
     */
    ~DoosanRobotController();
    
    // ==================== 초기화 및 연결 ====================
    
    /**
     * @brief 로봇 초기화 및 연결 설정
     * @return 성공 여부
     */
    bool initialize();
    
    /**
     * @brief 로봇 연결 대기
     * @param timeout_sec 타임아웃 (초)
     * @return 연결 성공 여부
     */
    bool waitForConnection(double timeout_sec = 10.0);
    
    /**
     * @brief 로봇 상태 확인
     * @return 현재 로봇 상태
     */
    RobotStatus getRobotStatus() const;
    
    /**
     * @brief 연결 상태 확인
     * @return 연결 여부
     */
    bool isConnected() const { return status_.is_connected; }
    
    // ==================== 정보 조회 ====================
    
    /**
     * @brief 현재 관절 위치 조회
     * @return 6축 관절 위치 (rad)
     */
    std::array<double, 6> getCurrentJointPositions() const;
    
    /**
     * @brief 현재 TCP 위치 조회 (Task Space)
     * @param pose 출력 포즈
     * @return 성공 여부
     */
    bool getCurrentPose(geometry_msgs::Pose& pose);
    
    /**
     * @brief 로봇 상태 정보 출력 (디버깅용)
     */
    void printRobotStatus() const;
    
    // ==================== 기본 제어 (확장 가능) ====================
    
    /**
     * @brief 관절 공간 이동 (Joint Space Move)
     * @param joint_positions 목표 관절 위치 (6축, rad)
     * @param velocity 속도 비율 (0.0 ~ 1.0)
     * @param acceleration 가속도 비율 (0.0 ~ 1.0)
     * @return 성공 여부
     */
    bool moveJoint(const std::array<double, 6>& joint_positions, 
                   double velocity = 0.5, 
                   double acceleration = 0.5);
    
    /**
     * @brief 직선 이동 (Linear Move in Task Space)
     * @param target_pose 목표 TCP 위치
     * @param velocity 속도 (mm/s)
     * @param acceleration 가속도 (mm/s^2)
     * @return 성공 여부
     */
    bool moveLine(const geometry_msgs::Pose& target_pose,
                  double velocity = 100.0,
                  double acceleration = 100.0);
    
    /**
     * @brief 로봇 정지
     * @param stop_mode 정지 모드 (0: 감속 정지, 1: 급정지)
     * @return 성공 여부
     */
    bool stopMotion(int stop_mode = 0);
    
    // ==================== ROS 토픽 인터페이스 ====================
    
    /**
     * @brief 제어 루프 실행 (주기적 상태 퍼블리시)
     * 
     * ros::spin() 대신 사용하여 주기적으로 로봇 상태를 확인하고
     * ROS 토픽으로 퍼블리시합니다.
     */
    void spin();
    
    /**
     * @brief 제어 루프 한 번 실행
     * @return 계속 실행 여부
     */
    bool spinOnce();

protected:
    // ==================== ROS 콜백 함수들 ====================
    
    /**
     * @brief 로봇 상태 콜백 (dsr_control에서 퍼블리시)
     * @param msg 로봇 상태 메시지
     */
    void robotStateCallback(const dsr_msgs::RobotState::ConstPtr& msg);
    
    /**
     * @brief 관절 상태 콜백
     * @param msg 관절 상태 메시지
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    /**
     * @brief 로봇 에러 콜백
     * @param msg 에러 메시지
     */
    void robotErrorCallback(const dsr_msgs::RobotError::ConstPtr& msg);
    
    // ==================== 내부 헬퍼 함수들 ====================
    
    /**
     * @brief ROS 서비스 클라이언트 초기화
     */
    void initializeServiceClients();
    
    /**
     * @brief ROS 토픽 초기화 (Subscriber/Publisher)
     */
    void initializeTopics();
    
    /**
     * @brief 파라미터 로드
     */
    void loadParameters();
    
    /**
     * @brief 네임스페이스 조합 (예: "/dsr01a0912")
     * @return 전체 네임스페이스
     */
    std::string getFullNamespace() const;

private:
    // ROS 노드 핸들
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 로봇 설정 및 상태
    RobotConfig config_;
    RobotStatus status_;
    
    // ROS Subscribers
    ros::Subscriber robot_state_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber robot_error_sub_;
    
    // ROS Publishers (사용자 정의 상태 토픽)
    ros::Publisher status_pub_;
    ros::Publisher connection_status_pub_;
    
    // ROS Service Clients (두산 로봇 서비스들)
    ros::ServiceClient move_joint_client_;
    ros::ServiceClient move_line_client_;
    ros::ServiceClient get_current_pose_client_;
    ros::ServiceClient stop_client_;
    ros::ServiceClient set_robot_mode_client_;
    ros::ServiceClient get_robot_state_client_;
    
    // 제어 타이머
    ros::Timer control_timer_;
    
    // 최신 수신 데이터
    sensor_msgs::JointState latest_joint_state_;
    dsr_msgs::RobotState latest_robot_state_;
    
    // 동기화
    mutable std::mutex status_mutex_;
    
    // 로깅
    void logInfo(const std::string& message) const;
    void logWarn(const std::string& message) const;
    void logError(const std::string& message) const;
};

} // namespace doosan_robot_control

#endif // DOOSAN_ROBOT_CONTROLLER_H

