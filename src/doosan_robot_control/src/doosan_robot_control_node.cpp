/**
 * @file doosan_robot_control_node.cpp
 * @brief Doosan Robot Control Node - 메인 실행 파일
 * 
 * 두산로봇 제어 노드의 진입점입니다.
 * ROS 파라미터를 읽어서 DoosanRobotController를 생성하고 실행합니다.
 * 
 * @author KATECH Robotics Team
 * @date 2025-10-29
 * @license MIT
 */

#include <ros/ros.h>
#include <signal.h>
#include "doosan_robot_control/doosan_robot_controller.h"

// 전역 컨트롤러 포인터 (시그널 핸들러에서 사용)
std::shared_ptr<doosan_robot_control::DoosanRobotController> g_controller;

/**
 * @brief 시그널 핸들러 (Ctrl+C 등)
 * @param sig 시그널 번호
 */
void signalHandler(int sig)
{
    ROS_INFO("\n========================================");
    ROS_INFO("종료 시그널 수신 (Ctrl+C)");
    ROS_INFO("========================================");
    
    // ROS 종료
    ros::shutdown();
}

/**
 * @brief 메인 함수
 * @param argc 인자 개수
 * @param argv 인자 배열
 * @return 종료 코드
 */
int main(int argc, char** argv)
{
    try {
        // ==================== ROS 초기화 ====================
        ros::init(argc, argv, "doosan_robot_control_node", ros::init_options::NoSigintHandler);
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 시그널 핸들러 등록
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        ROS_INFO("========================================");
        ROS_INFO("Doosan Robot Control Node 시작");
        ROS_INFO("========================================");
        
        // ==================== 파라미터 읽기 ====================
        
        doosan_robot_control::RobotConfig config;
        
        // 명령줄 또는 launch 파일에서 파라미터 로드
        private_nh.param<std::string>("robot_id", config.robot_id, "dsr01");
        private_nh.param<std::string>("robot_model", config.robot_model, "a0912");
        private_nh.param<std::string>("host", config.host, "192.168.137.100");
        private_nh.param<int>("port", config.port, 12345);
        private_nh.param<std::string>("mode", config.mode, "real");
        private_nh.param<double>("control_rate", config.control_rate, 10.0);
        
        ROS_INFO("설정 로드 완료:");
        ROS_INFO("  - Robot ID: %s", config.robot_id.c_str());
        ROS_INFO("  - Robot Model: %s", config.robot_model.c_str());
        ROS_INFO("  - Host: %s:%d", config.host.c_str(), config.port);
        ROS_INFO("  - Mode: %s", config.mode.c_str());
        ROS_INFO("  - Control Rate: %.1f Hz", config.control_rate);
        
        // ==================== 컨트롤러 생성 및 초기화 ====================
        
        g_controller = std::make_shared<doosan_robot_control::DoosanRobotController>(nh, config);
        
        if (!g_controller->initialize()) {
            ROS_ERROR("컨트롤러 초기화 실패!");
            return 1;
        }
        
        // ==================== 로봇 연결 대기 ====================
        
        double connection_timeout = 30.0;  // 30초 타임아웃
        private_nh.param<double>("connection_timeout", connection_timeout, 30.0);
        
        ROS_INFO("\n========================================");
        ROS_INFO("로봇 연결 시도 중...");
        ROS_INFO("========================================");
        ROS_INFO("⚠️  주의: dsr_control 노드가 실행 중인지 확인하세요!");
        ROS_INFO("   예: roslaunch dsr_launcher dsr_moveit.launch model:=%s mode:=%s host:=%s", 
                 config.robot_model.c_str(), config.mode.c_str(), config.host.c_str());
        ROS_INFO("========================================\n");
        
        if (!g_controller->waitForConnection(connection_timeout)) {
            ROS_ERROR("\n========================================");
            ROS_ERROR("로봇 연결 실패!");
            ROS_ERROR("========================================");
            ROS_ERROR("문제 해결 방법:");
            ROS_ERROR("1. dsr_control 노드가 실행 중인지 확인");
            ROS_ERROR("2. 로봇 IP 주소가 올바른지 확인 (%s)", config.host.c_str());
            ROS_ERROR("3. 로봇이 켜져 있고 네트워크에 연결되어 있는지 확인");
            ROS_ERROR("4. 방화벽 설정 확인");
            ROS_ERROR("========================================");
            return 1;
        }
        
        // ==================== 초기 상태 확인 ====================
        
        ROS_INFO("\n========================================");
        ROS_INFO("초기 로봇 상태 확인");
        ROS_INFO("========================================");
        
        // 상태 수신을 위한 짧은 대기
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        
        // 로봇 상태 출력
        g_controller->printRobotStatus();
        
        // ==================== 데모 동작 (선택사항) ====================
        
        bool run_demo = false;
        private_nh.param<bool>("run_demo", run_demo, false);
        
        if (run_demo) {
            ROS_INFO("\n========================================");
            ROS_INFO("🎬 데모 모드 실행");
            ROS_INFO("========================================");
            
            // 예제: 현재 관절 위치 읽기
            auto joint_positions = g_controller->getCurrentJointPositions();
            ROS_INFO("현재 관절 위치 확인 완료");
            
            // 예제: 홈 포지션으로 이동 (모든 관절 0도)
            ROS_INFO("홈 포지션으로 이동 시작...");
            std::array<double, 6> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            
            if (g_controller->moveJoint(home_position, 0.3, 0.3)) {
                ROS_INFO("✅ 홈 포지션 이동 명령 전송 완료");
                ROS_INFO("   (실제 이동은 로봇 상태를 확인하세요)");
            } else {
                ROS_WARN("⚠️  홈 포지션 이동 명령 전송 실패");
            }
            
            ROS_INFO("========================================");
            ROS_INFO("데모 모드 종료");
            ROS_INFO("========================================\n");
        }
        
        // ==================== 메인 루프 ====================
        
        ROS_INFO("\n========================================");
        ROS_INFO("✅ 준비 완료!");
        ROS_INFO("========================================");
        ROS_INFO("제어 루프 실행 중...");
        ROS_INFO("종료하려면 Ctrl+C를 누르세요.");
        ROS_INFO("========================================\n");
        
        // 제어 루프 실행
        g_controller->spin();
        
        // ==================== 종료 ====================
        
        ROS_INFO("\n========================================");
        ROS_INFO("노드 종료 중...");
        ROS_INFO("========================================");
        
        g_controller.reset();
        
        ROS_INFO("✅ 정상 종료 완료");
        return 0;
        
    } catch (const std::exception& e) {
        ROS_ERROR("\n========================================");
        ROS_ERROR("❌ 예외 발생: %s", e.what());
        ROS_ERROR("========================================");
        return 1;
    } catch (...) {
        ROS_ERROR("\n========================================");
        ROS_ERROR("❌ 알 수 없는 예외 발생");
        ROS_ERROR("========================================");
        return 1;
    }
}

