
import asyncio
import argparse 
import sys

# 현재 디렉토리의 상위 폴더를 시스템 경로에 추가하여
# woosh_robot 관련 모듈을 임포트할 수 있도록 합니다.
sys.path.append("..")  

# Woosh Robot SDK 및 관련 모듈 임포트
from woosh_robot import WooshRobot, CommuSettings, NO_PRINT
from woosh.logger import create_logger

# Protobuf 메시지 정의 임포트
from woosh.proto.robot.robot_pb2 import (
    RobotInfo, 
    RobotState, 
    PoseSpeed, 
    Mode,
    Battery,
    Scene,
    TaskProc,
    DeviceState,   
    HardwareState,
    OperationState, 
    Model
)
from woosh.proto.util.common_pb2 import (
    Pose2D, 
    Twist
) 


def parse_args():
    """명령줄 인자를 파싱하는 함수."""
    parser = argparse.ArgumentParser(description='Woosh 로봇 모니터')
    parser.add_argument(
        '--ip', 
        type=str, 
        default='172.20.128.2', 
        help='Woosh 로봇의 IP 주소'
    )
    parser.add_argument(
        '--port', 
        type=int, 
        default=5480, 
        help='Woosh 로봇의 포트 번호'
    )
    return parser.parse_args()


async def main():
    """메인 비동기 함수."""
    args = parse_args()
    print(f"모니터링 시작. IP: {args.ip}, 포트: {args.port}")
    
    robot = None
    try:
        # --- 1. 로거 및 SDK 초기화 ---
        # 로그를 기록할 로거(logger)를 생성합니다.
        logger = create_logger(name="monitor")

        # SDK 연결 설정을 생성합니다.
        settings = CommuSettings(
            addr=args.ip,
            port=args.port,
            identity="monitor",
            logger=logger
        )
        
        # 설정 객체를 사용하여 로봇 인스턴스를 생성하고 연결합니다.
        robot = WooshRobot(settings)
        await robot.run()

        # --- 2. 초기 로봇 정보 요청 --- 
        # 연결 직후, 로봇의 전체 정보를 한 번 요청하여 현재 상태를 파악합니다.
        info, ok, err = await robot.robot_info_req(RobotInfo())
        if not ok:
            logger.error(f"로봇 정보 요청 실패: {err}")
            raise ConnectionError(f"로봇 정보를 가져올 수 없습니다: {err}")
        
        logger.info("--- 초기 로봇 정보 --- ")
        logger.info(
            f"  이름: {info.genral.display_model}, "
            f"ID: {info.genral.serial_number}"
        )
        logger.info(f"  상태: {info.state}")
        logger.info(
            f"  제어 모드: {info.mode.ctrl}, "
            f"작업 모드: {info.mode.work}"
        )
        
        pose: Pose2D = info.pose_speed.pose
        twist: Twist = info.pose_speed.twist
        logger.info(
            f"  위치/자세: [x:{pose.x:.2f}, y:{pose.y:.2f}, theta:{pose.theta:.2f}], "
            f"속도: [선속도:{twist.linear:.2f}, 각속도:{twist.angular:.2f}], "
            f"누적 주행 거리: {info.pose_speed.mileage:.2f}"
        ) 
        
        logger.info(
            f"  충전 상태: {info.battery.charge_state}, "
            f"배터리 전력: {info.battery.power}%"
        )
        logger.info(
            f"  현재 맵: {info.scene.map_name}, "
            f"맵 버전: {info.scene.version}"
        )
        logger.info(
            f"  작업 목적지: {info.task_proc.dest}, "
            f"동작: {info.task_proc.action.type}, "
            f"동작 상태: {info.task_proc.action.state}, "
            f"작업 상태: {info.task_proc.state}"
        )
        logger.info(
            f"  장치 상태: [하드웨어: {info.device_state.hardware}, 소프트웨어: {info.device_state.software}]"
        )
        logger.info(
            f"  운행 상태: [내비게이션: {info.operation_state.nav}, 로봇: {info.operation_state.robot}]"
        )
        logger.info("---------------------")

        # --- 3. 실시간 정보 구독 --- 
        # 로봇의 상태가 변경될 때마다 해당 정보를 수신하여 출력하기 위해 각 토픽을 구독합니다.

        # 3.1 로봇 상태 구독 (예: IDLE, TASK, FAULT)
        def print_robot_state(state: RobotState):
            logger.info(f"[상태 업데이트] {state}")
        await robot.robot_state_sub(print_robot_state, NO_PRINT)

        # 3.2 로봇 모드 구독 (예: AUTO, MANUAL)
        def print_mode(mode: Mode):
            logger.info(
                f"[모드 업데이트] 제어 모드: {mode.ctrl}, 작업 모드: {mode.work}"
            )
        await robot.robot_mode_sub(print_mode, NO_PRINT)

        # 3.3 로봇 위치/속도 구독 (주석 처리됨 - 너무 많은 로그를 방지하기 위함)
        def print_pose_speed(pose_speed: PoseSpeed):
            # 아래 주석을 해제하면 실시간으로 위치/속도 정보를 볼 수 있습니다.
            # pose: Pose2D = pose_speed.pose
            # twist: Twist = pose_speed.twist
            # logger.info(
            #     f"[위치/속도 업데이트] [x:{pose.x:.2f}, y:{pose.y:.2f}, theta:{pose.theta:.2f}], "
            #     f"[선속도:{twist.linear:.2f}, 각속도:{twist.angular:.2f}], "
            #     f"누적 주행 거리: {pose_speed.mileage:.2f}"
            # )
            pass
        await robot.robot_pose_speed_sub(print_pose_speed, NO_PRINT)
        
        # 3.4 배터리 정보 구독
        def print_battery(battery: Battery):
            logger.info(
                f"[배터리 업데이트] 충전 상태: {battery.charge_state}, 전력: {battery.power}%"
            )
        await robot.robot_battery_sub(print_battery, NO_PRINT)

        # 3.5 씬(Scene) 정보 구독
        def print_scene(scene: Scene):
            logger.info(
                f"[씬 업데이트] 맵 이름: {scene.map_name}, 맵 버전: {scene.version}"
            )
        await robot.robot_scene_sub(print_scene, NO_PRINT)

        # 3.6 작업 진행 정보 구독
        def print_task_proc(task_proc: TaskProc):
            logger.info(
                f"[작업 업데이트] 목적지: {task_proc.dest}, "
                f"동작: {task_proc.action.type}, "
                f"동작 상태: {task_proc.action.state}, "
                f"작업 상태: {task_proc.state}"
            )
        await robot.robot_task_process_sub(print_task_proc, NO_PRINT) 

        # 3.7 장치 상태 구독 (예: 비상정지 버튼, 위치 정확도)
        def print_device_state(device_state: DeviceState):
            logger.info(
                f"[장치 상태 업데이트] 하드웨어: {device_state.hardware}, 소프트웨어: {device_state.software}"
            ) 
            if device_state.hardware & DeviceState.kEmgBtn:
                logger.warning("-> 비상 정지 버튼 눌림!")
            if device_state.software & DeviceState.kLocation:
                logger.info("-> 위치 정확도 양호.")
        await robot.robot_device_state_sub(print_device_state, NO_PRINT)

        # 3.8 하드웨어 상태 구독
        def print_hardware_state(hardware_state: HardwareState):
            logger.info(f"[하드웨어 상태 업데이트] {hardware_state}")
        await robot.robot_hardware_state_sub(print_hardware_state, NO_PRINT)

        # 3.9 운행 상태 구독 (예: 장애물, 작업 가능 여부)
        def print_operation_state(operation_state: OperationState):
            logger.info(
                f"[운행 상태 업데이트] 내비게이션: {operation_state.nav}, 로봇: {operation_state.robot}"
            )
            if operation_state.nav & OperationState.kImpede:
                logger.warning("-> 장애물 감지됨!")
            if operation_state.robot & OperationState.kTaskable:
                logger.info("-> 작업 할당 가능.")
        await robot.robot_operation_state_sub(print_operation_state, NO_PRINT)
        
        # 3.10 로봇 모델(Footprint) 구독
        def print_model(model: Model):
            logger.info(f"[로봇 모델 업데이트] {model}")
        await robot.robot_model_sub(print_model, NO_PRINT)

        # --- 4. 프로그램 실행 유지 ---
        # Ctrl+C를 누를 때까지 1초마다 대기하며 프로그램이 종료되지 않도록 합니다.
        while True:
            await asyncio.sleep(1)
         
    except KeyboardInterrupt:
        logger.info("사용자에 의해 모니터가 중지되었습니다.")
    except ConnectionError as e:
        logger.error(f"연결 오류: {e}")
    except TimeoutError as e:
        logger.error(f"연결 시간 초과: {e}")
    except asyncio.CancelledError:
        logger.info("작업이 취소되었습니다.")
    except Exception as e:
        logger.error(f"알 수 없는 오류: {e}")
    finally:
        logger.info("모니터를 종료합니다.")
        # 프로그램 종료 시 로봇 연결을 안전하게 닫습니다.
        if robot is not None:
            try:
                await robot.stop()
                logger.info("로봇 연결이 성공적으로 종료되었습니다.")
            except Exception as e:
                logger.error(f"로봇 연결 종료 중 오류 발생: {e}")


if __name__ == "__main__":
    asyncio.run(main())
