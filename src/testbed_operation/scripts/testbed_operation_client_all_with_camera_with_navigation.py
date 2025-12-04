#!/usr/bin/env python

from socket import timeout
import sys
import os

# ROS 임포트 (런타임에 사용 가능)
try:
    import rospy
    from std_msgs.msg import Float64
except ImportError:
    print("Warning: ROS 패키지를 찾을 수 없습니다. ROS 환경에서 실행해야 합니다.")
    rospy = None

import asyncio
from threading import Thread

# Woosh Robot SDK 임포트
try:
    # 상대 경로로 woosh_robot_py 패키지 추가
    woosh_robot_py_path = os.path.join(os.path.dirname(__file__), '../../../../woosh_robot_py')
    woosh_robot_py_path = os.path.abspath(woosh_robot_py_path)
    if os.path.exists(woosh_robot_py_path) and woosh_robot_py_path not in sys.path:
        sys.path.insert(0, woosh_robot_py_path)
    
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT
    from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
    from woosh.proto.robot.robot_pack_pb2 import SwitchMap, SetRobotPose, InitRobot
    from woosh.proto.map.map_pack_pb2 import SceneList
    WOOSH_SDK_AVAILABLE = True
except ImportError as e:
    if rospy:
        rospy.logwarn(f"Woosh Robot SDK를 임포트할 수 없습니다: {e}")
    else:
        print(f"Warning: Woosh Robot SDK를 임포트할 수 없습니다: {e}")
    WOOSH_SDK_AVAILABLE = False
    WooshRobot = None
    CommuSettings = None
    NO_PRINT = None

# 기존 ROS 서비스 임포트
from testbed_operation.srv import MoveJoint, MoveJointRequest
from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionRequest
from testbed_operation.srv import MobilePositionTwist, MobilePositionTwistRequest

# 전역 변수: Woosh Robot 인스턴스 및 이벤트 루프
woosh_robot = None
robot_loop = None


def init_woosh_robot():
    """
    Woosh Robot 초기화 및 연결
    
    Returns:
        bool: 초기화 성공 여부
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        rospy.logerr("Woosh Robot SDK를 사용할 수 없습니다.")
        return False
    
    if woosh_robot is not None:
        rospy.loginfo("Woosh Robot이 이미 초기화되어 있습니다.")
        return True
    
    # ROS 파라미터에서 로봇 IP와 포트 가져오기
    robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
    robot_port = rospy.get_param('~robot_port', 5480)
    
    rospy.loginfo(f"Woosh Robot 연결 시도: {robot_ip}:{robot_port}")
    
    settings = CommuSettings(
        addr=robot_ip,
        port=robot_port,
        identity="testbed_operation_nav"
    )
    
    woosh_robot = WooshRobot(settings)
    
    def run_asyncio():
        """비동기 이벤트 루프 실행"""
        global robot_loop
        robot_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(robot_loop)
        
        async def main():
            try:
                if await woosh_robot.run():
                    rospy.loginfo("Woosh Robot 연결 성공")
                else:
                    rospy.logerr("Woosh Robot 연결 실패")
            except Exception as e:
                rospy.logerr(f"Woosh Robot 연결 중 오류: {e}")
        
        try:
            robot_loop.run_until_complete(main())
            # 연결 후 계속 실행
            robot_loop.run_forever()
        except KeyboardInterrupt:
            rospy.loginfo("Woosh Robot 이벤트 루프 종료")
        finally:
            robot_loop.close()
    
    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()
    
    # 연결 대기 (최대 5초)
    for i in range(50):
        if woosh_robot and woosh_robot.comm.is_connected():
            rospy.loginfo("Woosh Robot 연결 확인 완료")
            return True
        rospy.sleep(0.1)
    
    rospy.logwarn("Woosh Robot 연결 대기 시간 초과")
    return False


def check_current_status():
    """
    1단계: 현재 상태 확인 (맵 ID, 위치 등)
    
    Returns:
        tuple: (성공 여부, PoseSpeed 객체, 메시지)
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        return False, None, "Woosh Robot SDK를 사용할 수 없습니다."
    
    if woosh_robot is None or not woosh_robot.comm.is_connected():
        return False, None, "Woosh Robot이 연결되지 않았습니다."
    
    async def _check_status():
        pose_speed, ok, msg = await woosh_robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, None, f"위치 정보 요청 실패: {msg}"
        
        if pose_speed.map_id == 0:
            return False, pose_speed, "맵이 로드되지 않았습니다. (map_id = 0)"
        else:
            return True, pose_speed, f"맵이 로드되어 있습니다 (ID: {pose_speed.map_id})"
    
    try:
        future = asyncio.run_coroutine_threadsafe(_check_status(), robot_loop)
        success, pose_speed, message = future.result(timeout=5.0)
        
        if success:
            rospy.loginfo(f"✅ 현재 상태 확인 성공: {message}")
            rospy.loginfo(f"   위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
            rospy.loginfo(f"   맵 ID: {pose_speed.map_id}")
        else:
            rospy.logwarn(f"⚠️ 현재 상태 확인: {message}")
        
        return success, pose_speed, message
    except Exception as e:
        rospy.logerr(f"현재 상태 확인 중 오류: {e}")
        return False, None, str(e)


def get_available_maps():
    """
    2단계: 사용 가능한 맵 목록 확인
    
    Returns:
        tuple: (성공 여부, 장면 이름 리스트, 메시지)
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        return False, [], "Woosh Robot SDK를 사용할 수 없습니다."
    
    if woosh_robot is None or not woosh_robot.comm.is_connected():
        return False, [], "Woosh Robot이 연결되지 않았습니다."
    
    async def _get_maps():
        scene_list_req = SceneList()
        scene_list, ok, msg = await woosh_robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        if not ok:
            return False, [], f"맵 목록 요청 실패: {msg}"
        
        if not scene_list or not scene_list.scenes:
            return False, [], "사용 가능한 맵이 없습니다."
        
        available_scenes = []
        for scene in scene_list.scenes:
            available_scenes.append(scene.name)
        
        return True, available_scenes, f"{len(available_scenes)}개의 장면을 찾았습니다."
    
    try:
        future = asyncio.run_coroutine_threadsafe(_get_maps(), robot_loop)
        success, scenes, message = future.result(timeout=5.0)
        
        if success:
            rospy.loginfo(f"✅ 맵 목록 확인 성공: {message}")
            for i, scene_name in enumerate(scenes, 1):
                rospy.loginfo(f"   {i}. {scene_name}")
        else:
            rospy.logwarn(f"⚠️ 맵 목록 확인: {message}")
        
        return success, scenes, message
    except Exception as e:
        rospy.logerr(f"맵 목록 확인 중 오류: {e}")
        return False, [], str(e)


def load_map(scene_name=None):
    """
    3단계: 맵 로드
    
    Args:
        scene_name (str, optional): 로드할 장면 이름. None이면 첫 번째 맵 사용
    
    Returns:
        tuple: (성공 여부, 메시지)
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        return False, "Woosh Robot SDK를 사용할 수 없습니다."
    
    if woosh_robot is None or not woosh_robot.comm.is_connected():
        return False, "Woosh Robot이 연결되지 않았습니다."
    
    async def _load_map():
        # 맵 목록 확인
        scene_list_req = SceneList()
        scene_list, ok, msg = await woosh_robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        if not ok or not scene_list or not scene_list.scenes:
            return False, "사용 가능한 맵이 없습니다."
        
        # 장면 선택
        target_scene = scene_name if scene_name else scene_list.scenes[0].name
        
        # 맵 로드
        switch_map = SwitchMap()
        switch_map.scene_name = target_scene
        result, ok, msg = await woosh_robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
        
        if not ok:
            return False, f"맵 로드 실패: {msg}"
        
        # 맵 로드 완료 대기
        await asyncio.sleep(3)
        
        # 맵 로드 확인
        pose_speed, ok, _ = await woosh_robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok and pose_speed.map_id != 0:
            return True, f"맵 '{target_scene}' 로드 성공 (맵 ID: {pose_speed.map_id})"
        else:
            return False, f"맵 로드 후에도 맵 ID가 0입니다."
    
    try:
        future = asyncio.run_coroutine_threadsafe(_load_map(), robot_loop)
        success, message = future.result(timeout=10.0)
        
        if success:
            rospy.loginfo(f"✅ {message}")
        else:
            rospy.logerr(f"❌ {message}")
        
        return success, message
    except Exception as e:
        rospy.logerr(f"맵 로드 중 오류: {e}")
        return False, str(e)


def set_robot_pose(x=None, y=None, theta=None):
    """
    4단계: 로봇 위치 설정 (로컬라이제이션)
    
    Args:
        x (float, optional): 맵 상의 X 좌표. None이면 현재 위치 사용
        y (float, optional): 맵 상의 Y 좌표. None이면 현재 위치 사용
        theta (float, optional): 맵 상의 방향 (라디안). None이면 현재 방향 사용
    
    Returns:
        tuple: (성공 여부, 메시지)
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        return False, "Woosh Robot SDK를 사용할 수 없습니다."
    
    if woosh_robot is None or not woosh_robot.comm.is_connected():
        return False, "Woosh Robot이 연결되지 않았습니다."
    
    async def _set_pose():
        # 현재 위치 가져오기
        pose_speed, ok, _ = await woosh_robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, "현재 위치를 가져올 수 없습니다."
        
        # 위치 설정
        set_pose = SetRobotPose()
        set_pose.pose.x = x if x is not None else pose_speed.pose.x
        set_pose.pose.y = y if y is not None else pose_speed.pose.y
        set_pose.pose.theta = theta if theta is not None else pose_speed.pose.theta
        
        result, ok, msg = await woosh_robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"위치 설정 실패: {msg}"
        
        # 위치 설정 완료 대기
        await asyncio.sleep(2)
        
        return True, f"로봇 위치 설정 성공: ({set_pose.pose.x:.2f}, {set_pose.pose.y:.2f}, {set_pose.pose.theta:.2f})"
    
    try:
        future = asyncio.run_coroutine_threadsafe(_set_pose(), robot_loop)
        success, message = future.result(timeout=5.0)
        
        if success:
            rospy.loginfo(f"✅ {message}")
        else:
            rospy.logerr(f"❌ {message}")
        
        return success, message
    except Exception as e:
        rospy.logerr(f"로봇 위치 설정 중 오류: {e}")
        return False, str(e)


def initialize_robot():
    """
    5단계: 로봇 초기화
    
    Returns:
        tuple: (성공 여부, 메시지)
    """
    global woosh_robot, robot_loop
    
    if not WOOSH_SDK_AVAILABLE:
        return False, "Woosh Robot SDK를 사용할 수 없습니다."
    
    if woosh_robot is None or not woosh_robot.comm.is_connected():
        return False, "Woosh Robot이 연결되지 않았습니다."
    
    async def _init_robot():
        # 현재 위치 가져오기
        pose_speed, ok, _ = await woosh_robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, "현재 위치를 가져올 수 없습니다."
        
        # 로봇 초기화
        init_robot = InitRobot()
        init_robot.is_record = False
        init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
        init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
        init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0
        
        result, ok, msg = await woosh_robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"로봇 초기화 실패: {msg}"
        
        # 초기화 완료 대기
        await asyncio.sleep(2)
        
        return True, f"로봇 초기화 성공: ({init_robot.pose.x:.2f}, {init_robot.pose.y:.2f}, {init_robot.pose.theta:.2f})"
    
    try:
        future = asyncio.run_coroutine_threadsafe(_init_robot(), robot_loop)
        success, message = future.result(timeout=5.0)
        
        if success:
            rospy.loginfo(f"✅ {message}")
        else:
            rospy.logerr(f"❌ {message}")
        
        return success, message
    except Exception as e:
        rospy.logerr(f"로봇 초기화 중 오류: {e}")
        return False, str(e)


# 기존 함수들 (변경 없음)
def mobile_move(distance=0.1):
    """
    모바일 로봇을 지정된 거리만큼 이동시키는 함수
    기본값: 0.1m
    """
    service_name = '/mobile_positiontwist'
    rospy.loginfo("Waiting for mobile service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_mobile = rospy.ServiceProxy(service_name, MobilePositionTwist)
        req = MobilePositionTwistRequest()
        req.distance = distance

        rospy.loginfo("Sending mobile move command: %.2f m", distance)
        response = move_mobile(req)
        rospy.loginfo("Mobile move command sent successfully.")
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Mobile service call failed: %s", e)
        return False

def dsr_move_joint(pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   vel=30.0, acc=30.0, time=0.0,
                   radius=0.0, mode=0, blendType=0, syncType=0):
    """
    DSR 로봇을 사용자 정의 자세로 이동시키는 범용 함수
    원하는 관절 각도를 직접 지정하여 로봇을 제어
    """
    service_name = '/dsr01a0912/motion/move_joint'
    rospy.loginfo("Waiting for DSR service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_joint = rospy.ServiceProxy(service_name, MoveJoint)
        req = MoveJointRequest()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = radius
        req.mode = mode
        req.blendType = blendType
        req.syncType = syncType

        rospy.loginfo("Sending DSR move_joint command: %s", pos)
        resp = move_joint(req)
        if resp.success:
            rospy.loginfo("DSR move_joint succeeded.")
        else:
            rospy.logwarn("DSR move_joint failed: %s", resp.message if hasattr(resp, 'message') else "Unknown")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("DSR service call failed: %s", e)
        return False

def dsr_move_home(pos=[90.0, 0.0, 90.0, 0.0, 90.0, -90.0],
                  vel=30.0, acc=30.0, time=0.0,
                  radius=0.0, mode=0, blendType=0, syncType=0):
    """
    DSR 로봇을 홈 포지션으로 이동시키는 함수
    기본 자세: [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]
    인자 없이 호출 시 기본 자세로 이동
    """
    service_name = '/dsr01a0912/motion/move_joint'  # move_joint 서비스 사용
    rospy.loginfo("Waiting for DSR service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_joint = rospy.ServiceProxy(service_name, MoveJoint)
        req = MoveJointRequest()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = radius
        req.mode = mode
        req.blendType = blendType
        req.syncType = syncType

        rospy.loginfo("Sending DSR move_home command: %s", pos)
        resp = move_joint(req)
        if resp.success:
            rospy.loginfo("DSR move_home succeeded.")
        else:
            rospy.logwarn("DSR move_home failed: %s", resp.message if hasattr(resp, 'message') else "Unknown")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("DSR service call failed: %s", e)
        return False

def request_qr_pose_correction(current_joint_pos, measurement_point_id="default"):
    """
    QR 코드 기반 위치/자세 보정 요청 함수

    Args:
        current_joint_pos (list): 현재 로봇의 관절 각도 [6개, deg]
        measurement_point_id (str): 측정 포인트 식별자
    
    Returns:
        dict: 보정값 딕셔너리 또는 None (실패시)
    """
    service_name = '/qr_pose_correction'
    rospy.loginfo("QR 위치 보정 서비스 대기중: %s", service_name)

    try:
        rospy.wait_for_service(service_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("QR 위치 보정 서비스 사용 불가 (timeout)")
        return None

    try:
        qr_correction = rospy.ServiceProxy(service_name, QRPoseCorrection)
        req = QRPoseCorrectionRequest()
        req.robot_ready = True
        req.current_joint_pos = current_joint_pos
        req.measurement_point_id = measurement_point_id

        rospy.loginfo("QR 위치 보정 요청 전송: %s", measurement_point_id)
        resp = qr_correction(req)

        if resp.success:
            rospy.loginfo("QR 위치 보정 성공 (신뢰도: %.2f)", resp.qr_confidence)
            return {
                'delta_x': resp.delta_x,
                'delta_y': resp.delta_y,
                'delta_z': resp.delta_z,
                'delta_rx': resp.delta_rx,
                'delta_ry': resp.delta_ry,
                'delta_rz': resp.delta_rz,
            }
        else:
            rospy.logwarn("QR 위치 보정 실패: %s", resp.message)
            return None

    except rospy.ServiceException as e:
        rospy.logerr("QR 위치 보정 서비스 호출 실패: %s", e)
        return None


def main():
    rospy.init_node('integrated_robot_client_with_navigation', anonymous=True)

    # Woosh Robot 초기화 (서버가 이미 맵을 로드했으므로 간소화)
    rospy.loginfo("=== Woosh Robot 초기화 ===")
    rospy.loginfo("참고: 맵 로드 및 기본 설정은 mobile_robot_server에서 수행됩니다.")
    rospy.loginfo("      서버가 실행 중이어야 합니다: mobile_posiotion_server_twist_with_navigation.py")
    
    if not init_woosh_robot():
        rospy.logerr("Woosh Robot 초기화 실패. 네비게이션 기능을 사용할 수 없습니다.")
        rospy.logwarn("기존 기능만 사용합니다.")
    else:
        rospy.loginfo("Woosh Robot 초기화 완료")
        
        # 서버가 이미 맵을 로드했으므로 현재 상태만 확인
        rospy.loginfo("\n=== 현재 상태 확인 (서버가 이미 맵을 로드했으므로 확인만 수행) ===")
        status_ok, pose_speed, status_msg = check_current_status()
        
        if status_ok:
            rospy.loginfo("✅ 서버가 이미 맵을 로드하고 있습니다.")
            rospy.loginfo(f"   맵 ID: {pose_speed.map_id}")
            rospy.loginfo(f"   현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
        else:
            rospy.logwarn("⚠️ 맵이 로드되지 않았습니다. 서버가 정상적으로 실행되었는지 확인하세요.")
            rospy.logwarn("   서버 실행 명령: rosrun mobile_robot_server mobile_posiotion_server_twist_with_navigation.py")
            rospy.logwarn("   서버가 먼저 실행되어야 합니다.")
        
        # 필요시에만 위치 재설정 (일반적으로는 서버가 이미 설정함)
        # 주석 처리: 서버가 이미 위치를 설정했으므로 불필요
        # rospy.loginfo("\n=== 로봇 위치 확인 (서버가 이미 설정했으므로 생략) ===")
        
        rospy.loginfo("\n=== Woosh Robot 준비 완료 ===\n")

    # DSR 로봇 속도, 가속도, 이동 시간 설정
    dsr_vel = 30.0
    dsr_acc = 30.0
    dsr_time = 3.0

    # 1. 홈 포지션 이동
    rospy.loginfo("=== Step 1: Moving to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to home position.")
        return

    rospy.sleep(1.0)    # 안정화 대기

    # 2. QR 코드 측정 위치로 이동
    rospy.loginfo("=== Step 2: Moving to QR measurement position ===")
    qr_measurement_pos = [49.54, 31.27, 87.67, 0.0, 61.06, -130.46]  # A-point1
    if not dsr_move_joint(pos=qr_measurement_pos, vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to QR measurement position.")
        return

    rospy.sleep(0.5)  # 안정화 대기

    # 3. QR 코드 기반 위치/자세 보정
    rospy.loginfo("=== Step 3: Requesting QR pose correction ===")    
    correction_data = request_qr_pose_correction(
            current_joint_pos=qr_measurement_pos,
            measurement_point_id="A-point"
    )
    if correction_data is None:
        rospy.logerr("QR pose correction failed. Aborting.")
        return
        
    # 4. 보정값 기반 로봇 자세 보정
    rospy.loginfo("=== Step 4: Applying pose correction ===")
    if not dsr_move_joint(pos=correction_data, vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to pose correction position.")
        return

    # 5. 홈 포지션으로 복귀
    rospy.loginfo("=== Step 5: Returning to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to return to home position.")
        return
    
    rospy.loginfo("All operations completed successfully.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user.")
    except Exception as e:
        rospy.logerr(f"프로그램 실행 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()

