#!/usr/bin/env python3
"""
모바일 로봇 배터리 잔량 확인 스크립트

Copyright © 2025 KATECH (Korea Automotive Technology Institute)
Author: LDJ (djlee2@katech.re.kr)
"""
import asyncio
from woosh_robot import WooshRobot, CommuSettings
from woosh.proto.robot.robot_pb2 import RobotInfo
# from woosh.proto.dispatch.system.dispatch_pb2 import GotoCharge
from woosh.proto.dispatch.system_pb2 import GotoCharge
from woosh.proto.robot.robot_pack_pb2 import ExecTask


# ANSI 색상 코드
class Colors:
    """터미널 색상 코드"""
    HEADER = '\033[95m'      # 보라색
    OKBLUE = '\033[94m'      # 파란색
    OKCYAN = '\033[96m'      # 청록색
    OKGREEN = '\033[92m'     # 초록색
    WARNING = '\033[93m'     # 노란색
    FAIL = '\033[91m'        # 빨간색
    ENDC = '\033[0m'         # 색상 종료
    BOLD = '\033[1m'         # 굵게
    UNDERLINE = '\033[4m'    # 밑줄


def print_battery_status(battery_level):
    """
    배터리 잔량을 색상으로 표시
    
    Args:
        battery_level: 배터리 잔량 (%)
    """
    print("\n" + "=" * 60)
    
    # 배터리 레벨에 따라 색상 변경
    if battery_level >= 80:
        # 80% 이상: 초록색
        color = Colors.OKGREEN
        icon = "🔋"
        status = "충분"
    elif battery_level >= 50:
        # 50~79%: 청록색
        color = Colors.OKCYAN
        icon = "🔋"
        status = "보통"
    elif battery_level >= 20:
        # 20~49%: 노란색 (경고)
        color = Colors.WARNING
        icon = "🪫"
        status = "주의"
    else:
        # 20% 미만: 빨간색 (위험)
        color = Colors.FAIL
        icon = "🪫"
        status = "위험"
    
    # 배터리 바 생성
    bar_length = 40
    filled_length = int(bar_length * battery_level / 100)
    bar = "█" * filled_length + "░" * (bar_length - filled_length)
    
    # 출력
    print(f"{Colors.BOLD}{Colors.HEADER}📊 모바일 로봇 배터리 상태{Colors.ENDC}")
    print("=" * 60)
    print(f"\n{color}{Colors.BOLD}{icon}  배터리 잔량: {battery_level}% ({status}){Colors.ENDC}")
    print(f"\n[{color}{bar}{Colors.ENDC}] {battery_level}%\n")
    print("=" * 60 + "\n")


async def main():
    """메인 함수"""
    print(f"{Colors.BOLD}🤖 모바일 로봇 연결 중...{Colors.ENDC}")
    
    # 1) 연결 설정
    settings = CommuSettings(
        addr="169.254.128.2",  # 로봇 IP
        port=5480,             # 로봇 포트
        identity="charger"     # 클라이언트 식별자
    )
    robot = WooshRobot(settings)
    await robot.run()

    # 2) 로봇 정보 요청 → 배터리 잔량 출력
    info, ok, msg = await robot.robot_info_req(RobotInfo())
    if not ok:
        print(f"{Colors.FAIL}{Colors.BOLD}❌ 로봇 정보 조회 실패: {msg}{Colors.ENDC}")
    else:
        # 배터리 정보를 색상으로 출력
        battery_level = info.battery.power
        print_battery_status(battery_level)

    # 3) 연결 종료
    await robot.stop()
    print(f"{Colors.OKGREEN}✅ 연결 종료{Colors.ENDC}")


if __name__ == "__main__":
    asyncio.run(main())