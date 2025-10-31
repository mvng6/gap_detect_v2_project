#!/bin/bash

# 모바일-협동로봇 협업 시스템 빌드 스크립트

echo "=================================================="
echo "🔨 모바일-협동로봇 협업 시스템 빌드"
echo "=================================================="

# ROS 환경 설정
source /opt/ros/noetic/setup.bash

# 워크스페이스로 이동
cd /home/katech/robot_ws

# 빌드 실행
echo ""
echo "📦 doosan_helper 패키지 빌드 중..."
catkin_make --only-pkg-with-deps doosan_helper

# 빌드 결과 확인
if [ $? -eq 0 ]; then
    echo ""
    echo "=================================================="
    echo "✅ 빌드 성공!"
    echo "=================================================="
    echo ""
    echo "다음 명령으로 환경 설정:"
    echo "  source /home/katech/robot_ws/devel/setup.bash"
    echo ""
    echo "협업 시스템 실행:"
    echo "  roslaunch doosan_helper mobile_cobot_collaboration.launch"
    echo ""
else
    echo ""
    echo "=================================================="
    echo "❌ 빌드 실패!"
    echo "=================================================="
    exit 1
fi

