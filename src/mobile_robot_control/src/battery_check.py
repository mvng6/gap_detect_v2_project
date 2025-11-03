import asyncio
from woosh_robot import WooshRobot, CommuSettings
from woosh.proto.robot.robot_pb2 import RobotInfo
# from woosh.proto.dispatch.system.dispatch_pb2 import GotoCharge
from woosh.proto.dispatch.system_pb2 import GotoCharge
from woosh.proto.robot.robot_pack_pb2 import ExecTask

async def main():
    # 1) 연결 설정
    settings = CommuSettings(
        addr="169.254.128.2",  # 로봇 IP
        port=5480,             # 로봇 포트
        identity="charger"     # 클라이언트 식별자
    )
    robot = WooshRobot(settings)
    await robot.run()

    # 2) 로봇 정보 요청 → 배터리 잔량 및 맵 버전 출력
    info, ok, msg = await robot.robot_info_req(RobotInfo())
    if not ok:
        print(f"로봇 정보 조회 실패: {msg}")
    else:
        # info.battery.power 에 배터리 잔량(%)이 담겨 있습니다 :contentReference[oaicite:0]{index=0}
        print(f"배터리 잔량: {info.battery.power}%")

    
    # result, ok, msg = await robot.exec_task_req(charge)

    # 3) 연결 종료
    await robot.stop()

if __name__ == "__main__":
    asyncio.run(main())