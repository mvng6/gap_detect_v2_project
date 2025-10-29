
import sys
import asyncio
import csv
from datetime import datetime

# Protobuf 메시지 정의 임포트
# 로봇의 다양한 상태와 데이터를 나타내는 클래스들입니다.
from woosh.proto.robot.robot_pb2 import (
    PoseSpeed,
    TaskProc,
    OperationState,
    ScannerData,
)
from woosh.proto.robot.robot_pack_pb2 import (
    ExecTask,
    ActionOrder,
    Twist,
)
from woosh.proto.util.action_pb2 import kCancel as kActionCancel # 작업 취소 상수
from woosh.proto.util.task_pb2 import State as TaskState, Type as TaskType, Direction as TaskDirection # 작업 상태, 유형, 방향 상수
from woosh.proto.ros.ros_pack_pb2 import (
    CallAction,
    Feedbacks,
)
from woosh.proto.ros.action_pb2 import (
    StepControl,
    ControlAction,
)

# Woosh Robot SDK 핵심 클래스 임포트
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot


# --- CSV 로거 클래스 정의 --- #
# 네비게이션 작업 관련 터미널 출력 메시지들을 CSV 파일로 저장하는 클래스입니다.
class NavigationCsvLogger:
    """네비게이션 작업 관련 메시지들을 타임스탬프와 함께 CSV 파일로 로깅합니다."""

    def __init__(self):
        """네비게이션 로그 파일을 초기화하고 헤더를 작성합니다."""
        # 파일 이름에 타임스탬프를 추가하여 실행할 때마다 고유한 로그 파일 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        print(f"📝 네비게이션 작업 CSV 로그 파일을 생성합니다. (타임스탬프: {timestamp})")
        
        try:
            # 네비게이션 작업 전용 로그 파일
            self.nav_file = open(f'navigation_log_{timestamp}.csv', 'w', newline='', encoding='utf-8')
            self.nav_writer = csv.writer(self.nav_file)
            self.nav_writer.writerow([
                'timestamp', 'event_type', 'task_id', 'status', 'x', 'y', 'theta', 'message'
            ])
            
            print("   -> 네비게이션 로그 파일이 성공적으로 생성되었습니다.")

        except IOError as e:
            print(f"❌ 네비게이션 로그 파일 생성 중 오류 발생: {e}")
            self.nav_writer = None

    def log_navigation_start(self, x: float, y: float, theta: float):
        """네비게이션 작업 시작을 기록합니다."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        self.nav_writer.writerow([
            ts, 'NAVIGATION_START', '', 'STARTED', f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}',
            f'네비게이션 작업 시작 - 목표: X={x}, Y={y}, Theta={theta}'
        ])
        self.nav_file.flush()  # 즉시 파일에 쓰기

    def log_navigation_task_sent(self, task_id: str = ''):
        """네비게이션 작업 전송 성공을 기록합니다."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        self.nav_writer.writerow([
            ts, 'TASK_SENT', task_id, 'SUCCESS', '', '', '', 
            f'내비게이션 작업 전송 성공 (작업 ID: {task_id})'
        ])
        self.nav_file.flush()

    def log_navigation_task_failed(self, error_msg: str):
        """네비게이션 작업 전송 실패를 기록합니다."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        self.nav_writer.writerow([
            ts, 'TASK_FAILED', '', 'FAILED', '', '', '', 
            f'내비게이션 작업 전송 실패: {error_msg}'
        ])
        self.nav_file.flush()

    def log_navigation_update(self, task_id: int, state: str, message: str = ''):
        """네비게이션 작업 상태 업데이트를 기록합니다."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        
        # 상태별 한글 메시지 생성
        status_messages = {
            'kExecuting': '내비게이션 실행 중...',
            'kCompleted': '내비게이션 작업 완료!',
            'kFailed': '내비게이션 작업 실패',
            'kCanceled': '내비게이션 작업 취소됨'
        }
        
        display_message = status_messages.get(state, f'상태 업데이트: {state}')
        if message:
            display_message += f' - {message}'
            
        self.nav_writer.writerow([
            ts, 'TASK_UPDATE', str(task_id), state, '', '', '', display_message
        ])
        self.nav_file.flush()

    def log_navigation_timeout(self):
        """네비게이션 작업 타임아웃을 기록합니다."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        self.nav_writer.writerow([
            ts, 'TIMEOUT', '', 'TIMEOUT', '', '', '', 
            '네비게이션 작업 대기 시간 초과'
        ])
        self.nav_file.flush()

    def log_navigation_position_update(self, x: float, y: float, theta: float):
        """네비게이션 중 위치 업데이트를 기록합니다 (선택적)."""
        if not self.nav_writer: return
        ts = datetime.now().isoformat()
        self.nav_writer.writerow([
            ts, 'POSITION_UPDATE', '', 'MOVING', f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}',
            f'위치 업데이트: X={x:.2f}, Y={y:.2f}, Theta={theta:.2f}'
        ])
        self.nav_file.flush()

    def close(self):
        """네비게이션 로그 파일을 안전하게 닫습니다."""
        print("📝 네비게이션 로그 파일을 닫습니다...")
        if hasattr(self, 'nav_file') and self.nav_file:
            self.nav_file.close()
        print("   -> 네비게이션 로그 파일이 성공적으로 저장 및 종료되었습니다.")


# --- 콜백 함수 정의 --- #
# SDK는 비동기 방식으로 로봇의 상태를 수신합니다.
# 특정 정보가 업데이트될 때마다 아래의 콜백 함수들이 호출됩니다.

def print_pose_speed(info: PoseSpeed):
    """로봇의 위치와 속도 정보를 수신했을 때 호출되는 콜백 함수."""
    print(f"위치/속도 업데이트:\n{info}")
    print(f"  현재 맵 ID: {info.map_id}")
    print(f"  선속도: {info.twist.linear:.2f} m/s, 각속도: {info.twist.angular:.2f} rad/s")
    print(f"  누적 주행 거리: {info.mileage:.2f} m")
    print(f"  좌표: x={info.pose.x:.2f}, y={info.pose.y:.2f}, theta={info.pose.theta:.2f}")

def print_task_proc(info: TaskProc):
    """로봇의 작업 진행 상태를 수신했을 때 호출되는 콜백 함수."""
    print(f"작업 진행 업데이트:\n{info}")
    if info.state == TaskState.kCompleted:
        print(f"-> 작업 ID: {info.robot_task_id}, 작업 완료")

def print_operation_state(state: OperationState):
    """로봇의 전반적인 운행 상태를 수신했을 때 호출되는 콜백 함수."""
    print(f"운행 상태 업데이트:\n{state}")
    if state.nav & OperationState.NavBit.kImpede:
        print("-> 로봇이 장애물을 만났습니다.")
    if state.robot & OperationState.RobotBit.kTaskable:
        print("-> 로봇이 작업을 받을 수 있는 상태입니다.")
    else:
        print("-> 로봇이 작업을 받을 수 없는 상태입니다.")

def print_feedbacks(fbs: Feedbacks):
    """ROS Action의 피드백을 수신했을 때 호출되는 콜백 함수."""
    print(f"ROS 피드백:\n{fbs}")
    for fb in fbs.fbs:
        if fb.state == TaskState.kRosSuccess:
            print(f"-> ROS 액션 [{fb.action}] 완료")

def print_scanner_data(data: ScannerData):
    """레이저 스캐너(Lidar) 데이터를 수신했을 때 호출되는 콜백 함수."""
    print(f"레이더 데이터 업데이트:\n{data}")


async def main():
    """메인 비동기 실행 함수."""
    # --- 1. 파라미터 처리 및 연결 설정 --- 
    # 기본 IP 및 포트 설정. 명령줄 인수가 있으면 해당 값으로 대체됩니다.
    addr = "169.254.128.2"
    port = 5480

    if len(sys.argv) >= 3:
        addr = sys.argv[1]
        port = int(sys.argv[2])

    print(f"연결 주소: {addr}:{port}")

    # SDK 연결을 위한 설정 객체 생성
    settings = CommuSettings(
        addr=addr,
        port=port,
        identity="woosdk-demo-ko", # SDK 클라이언트 식별자
    )

    # --- 1.5 네비게이션 CSV 로거 초기화 ---
    nav_logger = NavigationCsvLogger()
    
    # 로봇 인스턴스 생성
    robot = WooshRobot(settings)

    try:
        # --- 2. 로봇 연결 --- 
        # 로봇 연결 시도 및 성공 여부 확인
        if not await robot.run():
            print("❌ 로봇 연결에 실패했습니다. 프로그램을 종료합니다.")
            print("- 로봇 IP 주소와 포트를 확인해주세요.")
            print("- 로봇이 켜져 있고 네트워크에 연결되어 있는지 확인해주세요.")
            return
        
        print("✅ 로봇에 성공적으로 연결되었습니다.")

        # --- 3. 정보 구독 설정 --- 
        # 로봇의 주요 상태 정보가 변경될 때마다 print_... 콜백 함수가 호출되도록 설정합니다.
        # 네비게이션 관련 위치 업데이트만 선별적으로 로깅합니다.
        def pose_speed_callback_with_nav_logging(info: PoseSpeed):
            """위치/속도 정보를 출력하고, 네비게이션 중일 때만 CSV에 로깅하는 콜백."""
            print_pose_speed(info)
            # 네비게이션 중 위치 업데이트 로깅 (선택적 - 필요시 활성화)
            # nav_logger.log_navigation_position_update(info.pose.x, info.pose.y, info.pose.theta)

        print("로봇의 주요 상태 정보 구독을 시작합니다...")
        await robot.robot_pose_speed_sub(pose_speed_callback_with_nav_logging, NO_PRINT)
        await robot.robot_task_process_sub(print_task_proc, NO_PRINT)
        await robot.robot_operation_state_sub(print_operation_state, NO_PRINT)
        await robot.feedbacks_sub(print_feedbacks, NO_PRINT)
        await robot.scanner_data_sub(print_scanner_data, NO_PRINT)
        print("정보 구독 설정 완료.")

        # --- 4. 일회성 정보 요청 및 로봇 상태 확인 --- 
        # 현재 로봇의 위치/속도 정보를 한 번 요청하여 가져옵니다.
        print("\n현재 로봇의 상태 정보를 요청합니다...")
        pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            print(f"✅ 위치/속도 정보: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
            print(f"   맵 ID: {pose_speed.map_id}, 주행거리: {pose_speed.mileage:.2f}m")
        else:
            print(f"❌ 위치/속도 요청 실패: {msg}")

        # 현재 로봇의 운행 상태 정보를 한 번 요청하여 가져옵니다.
        state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if ok:
            print(f"✅ 운행 상태 확인 완료")
            # 로봇이 작업을 받을 수 있는 상태인지 확인
            if state.robot & OperationState.RobotBit.kTaskable:
                print("   -> 🟢 로봇이 작업을 받을 수 있는 상태입니다.")
            else:
                print("   -> 🔴 로봇이 작업을 받을 수 없는 상태입니다.")
                print("   -> 로봇의 현재 상태를 확인하고 작업 가능 상태로 만들어주세요.")
            
            # 내비게이션 상태 확인
            if state.nav & OperationState.NavBit.kImpede:
                print("   -> ⚠️  장애물이 감지되었습니다.")
            else:
                print("   -> 🟢 내비게이션 경로가 깨끗합니다.")
        else:
            print(f"❌ 운행 상태 요청 실패: {msg}")
            print("   -> 로봇 상태를 확인할 수 없어 작업 실행이 어려울 수 있습니다.")

        # --- 5. 사용자 입력 대기 및 작업 실행 --- 
        # 각 단계는 사용자가 엔터를 입력할 때마다 실행됩니다.

        # 5.1 내비게이션 작업
        input("\n엔터를 입력하여 내비게이션을 실행하세요...\n")
        
        print("🚀 내비게이션 작업을 시작합니다...")
        print(f"   목표 위치: X=1.5, Y=0.5, Theta=1.57 (약 90도)")
        
        # 네비게이션 시작 로깅
        nav_logger.log_navigation_start(1.5, 0.5, 1.57)
        
        # 작업 완료 감지를 위한 이벤트 객체 생성
        navigation_completed = asyncio.Event()
        current_task_id = None
        
        def navigation_task_callback(info: TaskProc):
            """내비게이션 작업 완료를 감지하고 CSV에 로깅하는 콜백 함수."""
            nonlocal current_task_id
            
            # 현재 실행 중인 작업인지 확인 (다른 작업의 콜백을 무시)
            if current_task_id is not None and info.robot_task_id != current_task_id:
                return
                
            print(f"📍 내비게이션 업데이트: ID={info.robot_task_id}, 상태={TaskState.Name(info.state)}")
            
            # 네비게이션 상태 업데이트 로깅
            nav_logger.log_navigation_update(info.robot_task_id, TaskState.Name(info.state), info.msg)
            
            if info.msg:
                print(f"   메시지: {info.msg}")
                
            if info.state == TaskState.kCompleted:
                print("   -> ✅ 내비게이션 작업 완료!")
                navigation_completed.set()
            elif info.state == TaskState.kFailed:
                print("   -> ❌ 내비게이션 작업 실패")
                navigation_completed.set()
            elif info.state == TaskState.kCanceled:
                print("   -> ⏹️ 내비게이션 작업 취소됨")
                navigation_completed.set()
            elif info.state == TaskState.kExecuting:
                print("   -> 🔄 내비게이션 실행 중...")
        
        # 내비게이션 전용 콜백 등록 (중복 등록 방지를 위해 기존 콜백 대신 사용)
        await robot.robot_task_process_sub(navigation_task_callback, NO_PRINT)
        
        # 내비게이션 작업 생성 (kParking 사용 - 단순 위치 이동용)
        # CLI 코드와 원본 demo_lite.py를 참고하여 필수 필드들을 모두 설정
        nav_task = ExecTask(
            task_id=77777,  # 고유한 작업 ID 설정 (원본 demo_lite.py 참고)
            type=TaskType.kParking,
            direction=TaskDirection.kDirectionUndefined,  # 방향 미정의 (단순 이동)
        )
        nav_task.pose.x = 1.5
        nav_task.pose.y = 0.5
        nav_task.pose.theta = 1.57
        
        print(f"   작업 설정: type={TaskType.Name(nav_task.type)}, direction={TaskDirection.Name(nav_task.direction)}")

        # 작업 실행 요청 (디버깅을 위해 상세 로그 출력)
        from woosh_interface import FULL_PRINT
        print("🚀 네비게이션 작업을 로봇에 전송합니다...")
        task_result, ok, msg = await robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)
        if ok:
            # 응답에서 작업 ID 추출 (있는 경우)
            if task_result and hasattr(task_result, 'task_id'):
                current_task_id = task_result.task_id
                print(f"✅ 내비게이션 작업 전송 성공 (작업 ID: {current_task_id})")
                nav_logger.log_navigation_task_sent(str(current_task_id))
            else:
                print("✅ 내비게이션 작업 전송 성공")
                nav_logger.log_navigation_task_sent()
            
            print("⏳ 작업 완료를 기다리는 중... (최대 60초)")
            try:
                # 최대 60초간 작업 완료 대기 (내비게이션은 시간이 걸릴 수 있음)
                await asyncio.wait_for(navigation_completed.wait(), timeout=60.0)
                print("🎯 내비게이션 작업 처리가 완료되었습니다.")
            except asyncio.TimeoutError:
                print("⏰ 내비게이션 작업 대기 시간이 초과되었습니다.")
                print("   작업이 계속 진행 중일 수 있습니다.")
                nav_logger.log_navigation_timeout()
        else:
            print(f"❌ 내비게이션 작업 전송 실패: {msg}")
            print("   로봇 상태를 확인하고 다시 시도해주세요.")
            nav_logger.log_navigation_task_failed(msg)

        await asyncio.sleep(2) # 잠시 대기

        # 5.2 작업 취소
        input("\n엔터를 입력하여 현재 작업을 취소하세요...\n")
        cancel_order = ActionOrder(order=kActionCancel)  # 취소 명령 생성
        _, ok, msg = await robot.action_order_req(cancel_order, NO_PRINT, NO_PRINT)
        if ok:
            print("작업 취소 요청 성공")
        else:
            print(f"작업 취소 요청 실패, msg: {msg}")

        await asyncio.sleep(1)

        # 5.3 스텝 제어 (정밀 이동)
        input("\n엔터를 입력하여 스텝 제어(직진 0.5m)를 실행하세요...\n")
        step_control = StepControl()
        step = step_control.steps.add() # 이동할 스텝 추가
        step.mode = StepControl.Step.Mode.kStraight # 모드: 직진
        step.value = 0.5  # 값: 0.5 미터
        step.speed = 0.25 # 속도: 0.25 m/s
        step_control.action = ControlAction.kExecute # 동작: 실행

        call_action = CallAction(step_control=step_control)
        _, ok, msg = await robot.call_action_req(call_action, NO_PRINT, NO_PRINT)
        if ok:
            print("스텝 제어 요청 성공")
        else:
            print(f"스텝 제어 요청 실패, msg: {msg}")

        await asyncio.sleep(5)

        # 5.4 원격 제어 (속도 제어)
        input("\n엔터를 입력하여 원격 제어(회전)를 시작하세요...\n")
        hertz = 20         # 제어 주기 (Hz)
        delay = 1.0 / hertz # 제어 지연 (s)
        linear_speed = 0.0   # 선속도
        angular_speed = 0.785 # 각속도 (초당 45도 회전)

        twist_cmd = Twist(linear=linear_speed, angular=angular_speed)

        print(f"{delay:.2f}초 간격으로 20회 속도 제어 명령을 전송합니다...")
        for _ in range(20):
            _, ok, msg = await robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            if not ok:
                print(f"속도 제어 요청 실패, msg: {msg}")
            await asyncio.sleep(delay)
        print("원격 제어(회전) 완료.")

        # 5.5 부드러운 감속 및 정지
        print("\n부드럽게 감속하여 정지합니다...")
        zero_time = 1.5  # 감속하여 정지하는 데 걸리는 시간 (s)
        num_steps = int(zero_time * hertz) # 감속을 위한 스텝 수
        linear_reduce = linear_speed / num_steps if num_steps > 0 else 0
        angular_reduce = angular_speed / num_steps if num_steps > 0 else 0

        twist_reduce_cmd = Twist(linear=linear_speed, angular=angular_speed)
        for n in range(num_steps):
            # 점진적으로 속도 감소
            twist_reduce_cmd.linear = max(0, twist_reduce_cmd.linear - linear_reduce)
            twist_reduce_cmd.angular = max(0, twist_reduce_cmd.angular - angular_reduce)
            
            print(f"감속 중... 선속도: {twist_reduce_cmd.linear:.2f}, 각속도: {twist_reduce_cmd.angular:.2f}")
            await robot.twist_req(twist_reduce_cmd, NO_PRINT, NO_PRINT)
            await asyncio.sleep(delay)

        # 안전을 위해 0 속도를 한 번 더 전송
        await robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        print("로봇 정지 완료.")
    
        input("\n엔터를 입력하여 프로그램을 종료하세요...\n")

    finally:
        # --- 6. 종료 처리 ---
        # 프로그램이 종료되기 전에 로봇 연결을 해제하고 네비게이션 로그 파일을 닫습니다.
        print("🔌 로봇 연결 및 네비게이션 로거를 종료합니다...")
        if robot.comm.is_connected():
            try:
                await robot.stop()
                print("✅ 로봇 연결이 안전하게 종료되었습니다.")
            except Exception as e:
                print(f"⚠️ 연결 종료 중 오류 발생: {e}")
        
        nav_logger.close()


if __name__ == "__main__":
    try:
        # 메인 비동기 함수 실행
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️ 사용자에 의해 프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류가 발생했습니다: {e}")
        print("로봇 연결 상태와 네트워크를 확인해주세요.")
