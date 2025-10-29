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
        # info.scene.version 에 현재 맵 버전이 담겨 있습니다
        print(f"맵 버전: {info.scene.version}")

    # # 3) 충전 명령 전송
    # charge = GotoCharge()
    # charge.robot = info.genral.serial_number  # 또는 직접 지정한 로봇 ID
    # # woosh.dispatch.system.GotoCharge 인터페이스로 충전 요청 :contentReference[oaicite:1]{index=1}

    # result, ok, msg = await robot.auto_charge(charge)
    # # ok, msg = await robot.auto_charge(charge)

    # if ok:
    #     print("✅ 충전 명령 전송 성공")
    # else:
    #     print(f"❌ 충전 명령 전송 실패: {msg}")

    # charge = GotoCharge()
    # charge.robot = 10001

    charge = ExecTask()
    charge.type = 3
    

    # result , ok , msg = await robot.action_order_req(charge)
    # result , ok , msg = await robot.exec_pre_task_req(charge)
    result, ok, msg = await robot.exec_task_req(charge)



    # 4) 연결 종료
    await robot.stop()

if __name__ == "__main__":
    asyncio.run(main())
