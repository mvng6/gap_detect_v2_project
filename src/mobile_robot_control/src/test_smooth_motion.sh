#!/bin/bash
# 부드러운 가감속 테스트 스크립트

echo "=========================================="
echo "🧪 부드러운 가감속 테스트"
echo "=========================================="

# 테스트 1: 기본 설정 (0.15m 가속, 0.2m 감속)
echo ""
echo "📍 테스트 1: 기본 가감속 (0.3m 이동)"
echo "   가속: 0.15m | 감속: 0.2m"
python3 mobile_robot_twist_control.py --distance 0.3 --speed 0.2

read -p "계속하려면 Enter를 누르세요..."

# 테스트 2: 빠른 가속, 긴 감속 (안정적)
echo ""
echo "📍 테스트 2: 빠른 가속 + 긴 감속 (0.5m 이동)"
echo "   가속: 0.1m | 감속: 0.3m"
python3 mobile_robot_twist_control.py --distance 0.5 --speed 0.3 --accel 0.1 --decel 0.3

read -p "계속하려면 Enter를 누르세요..."

# 테스트 3: 부드러운 가속, 짧은 감속 (스포티)
echo ""
echo "📍 테스트 3: 부드러운 가속 + 짧은 감속 (0.6m 이동)"
echo "   가속: 0.25m | 감속: 0.15m"
python3 mobile_robot_twist_control.py --distance 0.6 --speed 0.25 --accel 0.25 --decel 0.15

read -p "계속하려면 Enter를 누르세요..."

# 테스트 4: 짧은 거리 (자동 조정 테스트)
echo ""
echo "📍 테스트 4: 짧은 거리 (0.2m 이동, 자동 조정)"
echo "   요청: 가속 0.15m, 감속 0.2m → 자동 조정됨"
python3 mobile_robot_twist_control.py --distance 0.2 --speed 0.15

echo ""
echo "✅ 모든 테스트 완료!"

