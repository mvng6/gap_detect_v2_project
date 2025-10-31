/**
 * @file doosan_robot_controller.cpp
 * @brief Doosan Robot Controller - 구현 파일
 * 
 * @author KATECH Robotics Team
 * @date 2025-10-29
 */

#include "doosan_robot_control/doosan_robot_controller.h"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace doosan_robot_control {

// ==================== 생성자 / 소멸자 ====================

DoosanRobotController::DoosanRobotController(ros::NodeHandle& nh, const RobotConfig& config)
    : nh_(nh)
    , private_nh_("~")
    , config_(config)
    , status_()
{
    logInfo("DoosanRobotController 생성");
    logInfo("로봇 ID: " + config_.robot_id);
    logInfo("로봇 모델: " + config_.robot_model);
    logInfo("연결 대상: " + config_.host + ":" + std::to_string(config_.port));
    logInfo("모드: " + config_.mode);
}

DoosanRobotController::~DoosanRobotController()
{
    logInfo("DoosanRobotController 종료");
}

// ==================== 초기화 ====================

bool DoosanRobotController::initialize()
{
    logInfo("========================================");
    logInfo("로봇 초기화 시작");
    logInfo("========================================");
    
    try {
        // 파라미터 로드
        loadParameters();
        
        // ROS 토픽 초기화
        initializeTopics();
        
        // ROS 서비스 클라이언트 초기화
        initializeServiceClients();
        
        logInfo("초기화 완료!");
        return true;
        
    } catch (const std::exception& e) {
        logError("초기화 실패: " + std::string(e.what()));
        return false;
    }
}

void DoosanRobotController::loadParameters()
{
    logInfo("파라미터 로드 중...");
    
    // Private 노드 핸들에서 파라미터 읽기
    private_nh_.param<std::string>("robot_id", config_.robot_id, config_.robot_id);
    private_nh_.param<std::string>("robot_model", config_.robot_model, config_.robot_model);
    private_nh_.param<std::string>("host", config_.host, config_.host);
    private_nh_.param<int>("port", config_.port, config_.port);
    private_nh_.param<std::string>("mode", config_.mode, config_.mode);
    private_nh_.param<double>("control_rate", config_.control_rate, config_.control_rate);
    
    logInfo("  - robot_id: " + config_.robot_id);
    logInfo("  - robot_model: " + config_.robot_model);
    logInfo("  - host: " + config_.host);
    logInfo("  - port: " + std::to_string(config_.port));
    logInfo("  - mode: " + config_.mode);
    logInfo("  - control_rate: " + std::to_string(config_.control_rate) + " Hz");
}

void DoosanRobotController::initializeTopics()
{
    logInfo("ROS 토픽 초기화 중...");
    
    std::string ns = getFullNamespace();
    
    // Subscribers (두산 로봇에서 퍼블리시하는 토픽들)
    robot_state_sub_ = nh_.subscribe(
        ns + "/state", 10, 
        &DoosanRobotController::robotStateCallback, this);
    
    joint_state_sub_ = nh_.subscribe(
        ns + "/joint_states", 10,
        &DoosanRobotController::jointStateCallback, this);
    
    robot_error_sub_ = nh_.subscribe(
        ns + "/error", 10,
        &DoosanRobotController::robotErrorCallback, this);
    
    // Publishers (사용자 정의 토픽)
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/katech/doosan_status", 10, true);  // latched
    
    connection_status_pub_ = nh_.advertise<std_msgs::Bool>(
        "/katech/doosan_connected", 10, true);  // latched
    
    logInfo("토픽 초기화 완료");
    logInfo("  - 구독: " + ns + "/state");
    logInfo("  - 구독: " + ns + "/joint_states");
    logInfo("  - 구독: " + ns + "/error");
    logInfo("  - 발행: /katech/doosan_status");
    logInfo("  - 발행: /katech/doosan_connected");
}

void DoosanRobotController::initializeServiceClients()
{
    logInfo("ROS 서비스 클라이언트 초기화 중...");
    
    std::string ns = getFullNamespace();
    
    // 서비스 클라이언트 생성 (두산 로봇 SDK에서 제공)
    move_joint_client_ = nh_.serviceClient<dsr_msgs::MoveJoint>(
        ns + "/motion/move_joint");
    
    move_line_client_ = nh_.serviceClient<dsr_msgs::MoveLine>(
        ns + "/motion/move_line");
    
    get_current_pose_client_ = nh_.serviceClient<dsr_msgs::GetCurrentPose>(
        ns + "/system/get_current_pose");
    
    stop_client_ = nh_.serviceClient<dsr_msgs::MoveStop>(
        ns + "/motion/stop");
    
    set_robot_mode_client_ = nh_.serviceClient<dsr_msgs::SetRobotMode>(
        ns + "/system/set_robot_mode");
    
    get_robot_state_client_ = nh_.serviceClient<dsr_msgs::GetRobotState>(
        ns + "/system/get_robot_state");
    
    logInfo("서비스 클라이언트 초기화 완료");
}

std::string DoosanRobotController::getFullNamespace() const
{
    return "/" + config_.robot_id + config_.robot_model;
}

// ==================== 연결 관리 ====================

bool DoosanRobotController::waitForConnection(double timeout_sec)
{
    logInfo("========================================");
    logInfo("로봇 연결 대기 중... (최대 " + std::to_string(timeout_sec) + "초)");
    logInfo("========================================");
    
    ros::Time start_time = ros::Time::now();
    ros::Rate check_rate(10);  // 10Hz로 체크
    
    while (ros::ok()) {
        // 콜백 처리
        ros::spinOnce();
        
        // 연결 상태 확인 (joint_states 토픽이 수신되면 연결된 것으로 간주)
        if (latest_joint_state_.name.size() > 0) {
            status_.is_connected = true;
            logInfo("✅ 로봇 연결 성공!");
            
            // 연결 상태 퍼블리시
            std_msgs::Bool conn_msg;
            conn_msg.data = true;
            connection_status_pub_.publish(conn_msg);
            
            return true;
        }
        
        // 타임아웃 체크
        double elapsed = (ros::Time::now() - start_time).toSec();
        if (elapsed > timeout_sec) {
            logError("❌ 연결 타임아웃!");
            status_.is_connected = false;
            return false;
        }
        
        // 진행 상황 출력
        if (static_cast<int>(elapsed) % 2 == 0) {
            logInfo("대기 중... (" + std::to_string(static_cast<int>(elapsed)) + "초 경과)");
        }
        
        check_rate.sleep();
    }
    
    return false;
}

RobotStatus DoosanRobotController::getRobotStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

// ==================== 정보 조회 ====================

std::array<double, 6> DoosanRobotController::getCurrentJointPositions() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.joint_positions;
}

bool DoosanRobotController::getCurrentPose(geometry_msgs::Pose& pose)
{
    if (!isConnected()) {
        logWarn("로봇이 연결되지 않았습니다");
        return false;
    }
    
    dsr_msgs::GetCurrentPose srv;
    
    if (get_current_pose_client_.call(srv)) {
        // 서비스 응답을 geometry_msgs::Pose로 변환
        pose.position.x = srv.response.pos[0] / 1000.0;  // mm -> m
        pose.position.y = srv.response.pos[1] / 1000.0;
        pose.position.z = srv.response.pos[2] / 1000.0;
        
        // 오리엔테이션 (Roll-Pitch-Yaw를 Quaternion으로 변환 필요)
        // 여기서는 단순화를 위해 생략 (필요시 tf2 라이브러리 사용)
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        
        return true;
    } else {
        logError("현재 포즈 조회 실패");
        return false;
    }
}

void DoosanRobotController::printRobotStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    std::stringstream ss;
    ss << "\n========================================\n";
    ss << "🤖 로봇 상태 정보\n";
    ss << "========================================\n";
    ss << "연결 상태: " << (status_.is_connected ? "✅ 연결됨" : "❌ 연결 안됨") << "\n";
    ss << "이동 중: " << (status_.is_moving ? "예" : "아니오") << "\n";
    ss << "에러 발생: " << (status_.has_error ? "⚠️ 예" : "아니오") << "\n";
    
    if (status_.has_error) {
        ss << "에러 메시지: " << status_.error_message << "\n";
    }
    
    ss << "\n관절 위치 (도):\n";
    for (size_t i = 0; i < 6; ++i) {
        ss << "  Joint " << (i+1) << ": " 
           << std::fixed << std::setprecision(2) 
           << (status_.joint_positions[i] * 180.0 / M_PI) << "°\n";
    }
    
    ss << "\n관절 속도 (rad/s):\n";
    for (size_t i = 0; i < 6; ++i) {
        ss << "  Joint " << (i+1) << ": " 
           << std::fixed << std::setprecision(3) 
           << status_.joint_velocities[i] << " rad/s\n";
    }
    
    ss << "========================================\n";
    
    ROS_INFO_STREAM(ss.str());
}

// ==================== 기본 제어 ====================

bool DoosanRobotController::moveJoint(
    const std::array<double, 6>& joint_positions,
    double velocity,
    double acceleration)
{
    if (!isConnected()) {
        logError("로봇이 연결되지 않았습니다");
        return false;
    }
    
    logInfo("관절 공간 이동 시작...");
    
    dsr_msgs::MoveJoint srv;
    
    // 관절 위치 설정 (rad -> degree)
    for (size_t i = 0; i < 6; ++i) {
        srv.request.pos[i] = joint_positions[i] * 180.0 / M_PI;
    }
    
    srv.request.vel = velocity * 100.0;  // 0.0~1.0 -> 0~100
    srv.request.acc = acceleration * 100.0;
    srv.request.time = 0.0;  // 0: 자동 계산
    srv.request.mode = 0;    // 0: absolute
    srv.request.blendType = 0;  // 0: no blend
    srv.request.syncType = 0;   // 0: no sync
    
    if (move_joint_client_.call(srv)) {
        if (srv.response.success) {
            logInfo("✅ 관절 이동 명령 전송 성공");
            return true;
        } else {
            logError("관절 이동 실패");
            return false;
        }
    } else {
        logError("관절 이동 서비스 호출 실패");
        return false;
    }
}

bool DoosanRobotController::moveLine(
    const geometry_msgs::Pose& target_pose,
    double velocity,
    double acceleration)
{
    if (!isConnected()) {
        logError("로봇이 연결되지 않았습니다");
        return false;
    }
    
    logInfo("직선 이동 시작...");
    
    dsr_msgs::MoveLine srv;
    
    // 위치 설정 (m -> mm)
    srv.request.pos[0] = target_pose.position.x * 1000.0;
    srv.request.pos[1] = target_pose.position.y * 1000.0;
    srv.request.pos[2] = target_pose.position.z * 1000.0;
    
    // 오리엔테이션 (Quaternion -> RPY, 단순화를 위해 생략)
    srv.request.pos[3] = 0.0;  // Roll
    srv.request.pos[4] = 0.0;  // Pitch
    srv.request.pos[5] = 0.0;  // Yaw
    
    // vel[2]: [mm/sec, deg/sec] - boost::array는 고정 크기, 인덱스 접근
    srv.request.vel[0] = velocity;
    srv.request.vel[1] = velocity;
    // acc[2]: [mm/sec2, deg/sec2]
    srv.request.acc[0] = acceleration;
    srv.request.acc[1] = acceleration;
    srv.request.time = 0.0;
    srv.request.mode = 0;
    srv.request.blendType = 0;
    srv.request.ref = 0;  // base coordinate
    
    if (move_line_client_.call(srv)) {
        if (srv.response.success) {
            logInfo("✅ 직선 이동 명령 전송 성공");
            return true;
        } else {
            logError("직선 이동 실패");
            return false;
        }
    } else {
        logError("직선 이동 서비스 호출 실패");
        return false;
    }
}

bool DoosanRobotController::stopMotion(int stop_mode)
{
    logInfo("로봇 정지 명령 전송...");
    
    dsr_msgs::MoveStop srv;
    srv.request.stop_mode = stop_mode;
    
    if (stop_client_.call(srv)) {
        if (srv.response.success) {
            logInfo("✅ 정지 명령 전송 성공");
            return true;
        } else {
            logError("정지 실패");
            return false;
        }
    } else {
        logError("정지 서비스 호출 실패");
        return false;
    }
    
    return false;
}

// ==================== ROS 콜백 ====================

void DoosanRobotController::robotStateCallback(const dsr_msgs::RobotState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_robot_state_ = *msg;
    
    // 상태 업데이트
    status_.robot_state = msg->robot_state;
    status_.is_moving = (msg->robot_state == 3);  // 3: STATE_BUSY
    
    // 연결 상태 업데이트
    status_.is_connected = true;
}

void DoosanRobotController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_joint_state_ = *msg;
    
    // 관절 위치 및 속도 업데이트
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            status_.joint_positions[i] = msg->position[i];
        }
    }
    
    if (msg->velocity.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            status_.joint_velocities[i] = msg->velocity[i];
        }
    }
}

void DoosanRobotController::robotErrorCallback(const dsr_msgs::RobotError::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    if (msg->level > 0) {  // 에러 또는 경고
        status_.has_error = true;
        // RobotError는 msg1, msg2, msg3 필드를 사용
        status_.error_message = msg->msg1;
        if (!msg->msg2.empty()) {
            status_.error_message += " | " + msg->msg2;
        }
        if (!msg->msg3.empty()) {
            status_.error_message += " | " + msg->msg3;
        }
        
        if (msg->level == 1) {
            logWarn("⚠️ 로봇 경고: " + status_.error_message);
        } else {
            logError("❌ 로봇 에러: " + status_.error_message);
        }
    } else {
        status_.has_error = false;
        status_.error_message = "";
    }
}

// ==================== 제어 루프 ====================

void DoosanRobotController::spin()
{
    ros::Rate rate(config_.control_rate);
    
    logInfo("========================================");
    logInfo("제어 루프 시작 (" + std::to_string(config_.control_rate) + " Hz)");
    logInfo("========================================");
    
    while (ros::ok()) {
        if (!spinOnce()) {
            break;
        }
        rate.sleep();
    }
    
    logInfo("제어 루프 종료");
}

bool DoosanRobotController::spinOnce()
{
    // ROS 콜백 처리
    ros::spinOnce();
    
    // 상태 정보 퍼블리시
    if (isConnected()) {
        std_msgs::String status_msg;
        std::stringstream ss;
        ss << "Connected | State: " << status_.robot_state 
           << " | Moving: " << (status_.is_moving ? "Yes" : "No")
           << " | Error: " << (status_.has_error ? "Yes" : "No");
        status_msg.data = ss.str();
        status_pub_.publish(status_msg);
    }
    
    return ros::ok();
}

// ==================== 로깅 ====================

void DoosanRobotController::logInfo(const std::string& message) const
{
    ROS_INFO_STREAM("[DoosanRobot] " << message);
}

void DoosanRobotController::logWarn(const std::string& message) const
{
    ROS_WARN_STREAM("[DoosanRobot] " << message);
}

void DoosanRobotController::logError(const std::string& message) const
{
    ROS_ERROR_STREAM("[DoosanRobot] " << message);
}

} // namespace doosan_robot_control

