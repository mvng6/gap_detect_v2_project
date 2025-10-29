import rospy
import asyncio
import sys

from woosh_robot import WooshRobot, CommuSettings
# 1. robot_pb2: 로봇의 기본 상태/정보 (OperationState, PoseSpeed 등)
from woosh.proto.robot.robot_pb2 import RobotInfo, OperationState, PoseSpeed

# 2. map_pack_pb2: 맵 관련 (SceneList)
from woosh.proto.map.map_pack_pb2 import SceneList

# 3. robot_pack_pb2: 로봇에 보내는 명령 (SwitchMap, SetRobotPose, Twist 등)
from woosh.proto.robot.robot_pack_pb2 import ExecTask, Twist, SwitchControlMode, SetRobotPose, SwitchMap

# 4. util.task_pb2: 작업(Task) 관련 (kParking, kDirectionUndefined 등)
from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection

# 5. ros.action_pb2: ROS 액션 관련 (StepControl)
from woosh.proto.ros.action_pb2 import StepControl, ControlAction

# 6. ros.ros_pack_pb2: ROS 액션 "호출" (CallAction)
from woosh.proto.ros.ros_pack_pb2 import CallAction

# 7. util.robot_pb2: 로봇 유틸리티 (kAuto 모드 등)
from woosh.proto.util.robot_pb2 import ControlMode

# =====================================================

# --- ROS 메시지 임포트 (향후 사용 대비) ---
# from std_msgs.msg import Bool, String
# from geometry_msgs.msg import Pose2D, Twist

class RobotConnectionNode:
    """
    모바일 로봇 SDK와의 비동기(asyncio) 연결을 관리하고
    ROS 인터페이스를 제공하는 메인 클래스.
    """
    def __init__(self):
        """노드 초기화, 파라미터 로드, 로봇 객체 생성"""

        #. 1 ROS 노드 초기화
        rospy.init_node('test_connect_node', anonymous=True, disable_signals=True)

        # 2. 파라미터 로드
        self.load_parameters()

        self.settings = None
        self.robot = None

        rospy.loginfo("🤖 Robot Connection Node 초기화 완료")
        rospy.loginfo(f"   -> 연결 대상: {self.robot_ip}:{self.robot_port} (ID: {self.robot_identity})")

    def load_parameters(self):
        """ROS 파라미터 서버에서 연결 정보를 로드합니다."""

        self.robot_ip = rospy.get_param('~robot_ip','169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port',5480)
        self.robot_identity = rospy.get_param('~robot_identity','tester')
        
        # (향후 사용)
        # self.connection_timeout = rospy.get_param('~connection_timeout', 10.0)
        # self.reconnect_interval = rospy.get_param('~reconnect_interval', 2.0)

        rospy.loginfo("📋 파라미터 로드 완료.")

    async def run(self):
        """
        메인 비동기 실행 루프.
        로봇 연결, 배터리 정보 요청, Ctrl+C 대기 및 안전한 종료를 처리
        """
        try:
            self.settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity
        )
            
            # 1. 로봇 객체 생성
            self.robot = WooshRobot(self.settings)
            
            # 2. 로봇 연결 및 실행 (SDK 실행)
            await self.robot.run()

            # 3. 'robot_info_req'로 실제 연결을 "검증"
            info, ok, msg = await self.robot.robot_info_req(RobotInfo())
            if not ok:
                rospy.logerr(f"❌ SDK 연결 검증 실패: {msg}")
                rospy.logerr("   -> IP 주소, 네트워크 또는 로봇 전원을 확인하세요.")
                raise ConnectionError(f"Failed to verify connection: {msg}")
            
            # 4. 검증이 성공했을 때만 "연결 성공" 로그를 출력
            rospy.loginfo(f"✅ 로봇 연결 성공! (Ctrl+C로 종료)")
            rospy.loginfo(f"🔋 현재 배터리 잔량: {info.battery.power}%")
            
# --- 🔽 [개선] 올바른 초기화 및 상대 거리 이동 구현 ---
            rospy.loginfo("--- [로봇 초기화 및 상대 거리 이동 테스트] ---")
            
            try:
                # === 1단계: 현재 상태 확인 ===
                rospy.loginfo("➡️ 1. [Status] 현재 로봇 상태를 확인합니다...")
                pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed())
                if not ok:
                    rospy.logerr(f"❌ [Status] 위치 조회 실패: {msg}")
                    raise Exception("Failed to get pose_speed")
                
                current_map_id = pose_speed.map_id
                rospy.loginfo(f"   현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
                rospy.loginfo(f"   맵 ID: {current_map_id}")

                # === 2단계: 맵 로드 (필요 시) ===
                available_scene_name = None
                if current_map_id == 0:
                    rospy.logwarn("⚠️ [MapLoad] 맵이 로드되지 않았습니다. 사용 가능한 맵을 찾습니다...")
                    scene_list, scene_ok, scene_msg = await self.robot.scene_list_req(SceneList())
                    
                    if not (scene_ok and scene_list.scenes):
                        rospy.logerr(f"❌ [MapLoad] 사용 가능한 맵/장면이 없습니다! ({scene_msg})")
                        rospy.logerr("   💡 TR-200 앱에서 먼저 맵을 생성해주세요.")
                        raise Exception("No available maps")
                    
                    available_scene_name = scene_list.scenes[0].name
                    rospy.loginfo(f"✅ [MapLoad] 사용 가능한 맵 '{available_scene_name}'(을)를 로드합니다...")
                    
                    switch_map = SwitchMap()
                    switch_map.scene_name = available_scene_name
                    _, map_ok, map_msg = await self.robot.switch_map_req(switch_map)
                    
                    if not map_ok:
                        rospy.logerr(f"❌ [MapLoad] 맵 로드 요청 실패: {map_msg}")
                        raise Exception("Map switch request failed")
                    
                    rospy.loginfo("✅ [MapLoad] 맵 로드 요청 성공. 3초 대기...")
                    await asyncio.sleep(3.0)
                else:
                    rospy.loginfo("✅ [MapLoad] 맵이 이미 로드되어 있습니다.")

                # === 3단계: 로봇 초기화 (InitRobot - 핵심!) ===
                rospy.loginfo("➡️ 3. [InitRobot] 로봇을 초기화합니다 (로컬라이제이션 활성화)...")
                from woosh.proto.robot.robot_pack_pb2 import InitRobot
                
                init_robot = InitRobot()
                init_robot.is_record = False  # 기록 모드 비활성화
                # 현재 위치를 맵 상의 초기 위치로 설정
                init_robot.pose.x = pose_speed.pose.x
                init_robot.pose.y = pose_speed.pose.y
                init_robot.pose.theta = pose_speed.pose.theta
                
                _, init_ok, init_msg = await self.robot.init_robot_req(init_robot)
                if not init_ok:
                    rospy.logerr(f"❌ [InitRobot] 초기화 요청 실패: {init_msg}")
                    raise Exception("InitRobot request failed")
                
                rospy.loginfo("✅ [InitRobot] 초기화 성공. 2초 대기...")
                await asyncio.sleep(2.0)

                # === 4단계: 제어 모드를 자동으로 설정 ===
                rospy.loginfo("➡️ 4. [ModeSwitch] 제어 모드를 '자동'으로 전환합니다...")
                switch_mode = SwitchControlMode()
                switch_mode.mode = ControlMode.kAuto
                _, mode_ok, mode_msg = await self.robot.switch_control_mode_req(switch_mode)
                if mode_ok:
                    rospy.loginfo("✅ [ModeSwitch] 자동 모드 전환 성공.")
                else:
                    rospy.logwarn(f"⚠️ [ModeSwitch] 자동 모드 전환 실패: {mode_msg} (이미 자동일 수 있음)")

                # === 5단계: 로봇 위치를 맵에 명시적으로 설정 (SetRobotPose) ===
                # 주의: 로봇의 현재 물리적 위치와 맵 상의 위치를 매핑해야 함
                rospy.loginfo("➡️ 5. [SetPose] 로봇 위치를 맵 상에 명시적으로 설정합니다...")
                
                # 방법 1: 현재 위치 사용 (로봇이 맵의 원점 근처에 있다고 가정)
                set_pose = SetRobotPose()
                set_pose.pose.x = pose_speed.pose.x
                set_pose.pose.y = pose_speed.pose.y
                set_pose.pose.theta = pose_speed.pose.theta
                
                rospy.loginfo(f"   [방법 1] 현재 위치로 설정 시도: X={set_pose.pose.x:.2f}, Y={set_pose.pose.y:.2f}, Theta={set_pose.pose.theta:.2f}")
                _, pose_ok, pose_msg = await self.robot.set_robot_pose_req(set_pose)
                if not pose_ok:
                    rospy.logwarn(f"⚠️ [SetPose] 현재 위치 설정 실패: {pose_msg}")
                else:
                    rospy.loginfo("✅ [SetPose] 현재 위치 설정 성공. 3초 대기...")
                    await asyncio.sleep(3.0)
                    
                    # 설정 후 map_id 확인
                    pose_speed_check, ok_check, _ = await self.robot.robot_pose_speed_req(PoseSpeed())
                    if ok_check and pose_speed_check.map_id != 0:
                        rospy.loginfo(f"   ✅ [방법 1] 성공! 맵 ID: {pose_speed_check.map_id}")
                        pose_speed = pose_speed_check  # 업데이트된 위치 사용
                    else:
                        rospy.logwarn("   ⚠️ [방법 1] 실패. 방법 2 시도...")
                        
                        # 방법 2: 맵 원점(0,0,0)으로 강제 설정
                        rospy.loginfo("   [방법 2] 맵 원점(0,0,0)으로 강제 설정 시도...")
                        set_pose_origin = SetRobotPose()
                        set_pose_origin.pose.x = 0.0
                        set_pose_origin.pose.y = 0.0
                        set_pose_origin.pose.theta = 0.0
                        
                        _, origin_ok, origin_msg = await self.robot.set_robot_pose_req(set_pose_origin)
                        if not origin_ok:
                            rospy.logerr(f"❌ [SetPose] 원점 설정도 실패: {origin_msg}")
                            rospy.logerr("   💡 TR-200 앱에서 수동으로 로봇 위치를 맵에 맞춰주세요.")
                            raise Exception("SetPose failed with both methods")
                        
                        rospy.loginfo("✅ [SetPose] 원점 설정 성공. 5초 대기 (로컬라이제이션 시간 필요)...")
                        await asyncio.sleep(5.0)

                # === 6단계: 상태 재확인 (map_id 및 kTaskable 체크) ===
                rospy.loginfo("➡️ 6. [Verify] 초기화 후 상태를 재확인합니다...")
                
                # map_id 확인
                pose_speed_after, ok_pose, msg_pose = await self.robot.robot_pose_speed_req(PoseSpeed())
                if ok_pose:
                    updated_map_id = pose_speed_after.map_id
                    rospy.loginfo(f"   맵 ID: {updated_map_id}")
                    if updated_map_id == 0:
                        rospy.logerr("   ❌ 맵 ID가 여전히 0입니다. 로컬라이제이션 실패!")
                        rospy.logerr("   📍 가능한 원인:")
                        rospy.logerr("      1. 로봇의 실제 위치가 맵 영역 밖에 있음")
                        rospy.logerr("      2. 로봇 주변 환경이 맵과 일치하지 않음 (벽, 물체 위치 변화)")
                        rospy.logerr("      3. 레이저 스캔 데이터가 맵과 매칭되지 않음")
                        rospy.logerr("")
                        rospy.logerr("   💡 해결 방법:")
                        rospy.logerr("      A. TR-200 앱에서 수동으로 로봇 위치를 맵에 맞춰주세요 (추천)")
                        rospy.logerr("      B. 로봇을 맵 생성 시 시작 위치로 물리적으로 이동")
                        rospy.logerr("      C. 맵을 다시 생성 (현재 환경 반영)")
                        rospy.logerr("")
                        rospy.logerr("   🔧 임시 해결: 맵 없이 상대 거리 이동을 시도합니다...")
                        # 맵 없이도 StepControl은 작동할 수 있음 (로봇 로컬 좌표계 기준)
                    else:
                        rospy.loginfo(f"   ✅ 맵 ID가 {updated_map_id}로 업데이트되었습니다!")
                
                # kTaskable 확인
                state, ok_state, msg_state = await self.robot.robot_operation_state_req(OperationState())
                if not ok_state:
                    rospy.logerr(f"❌ [Verify] 상태 조회 실패: {msg_state}")
                    raise Exception("OperationState check failed")
                
                rospy.loginfo(f"   로봇 상태 비트: robot={bin(state.robot)}, nav={bin(state.nav)}")
                
                is_taskable = hasattr(OperationState, 'RobotBit') and (state.robot & OperationState.RobotBit.kTaskable)
                if is_taskable:
                    rospy.loginfo("   ✅ 로봇이 'kTaskable' 상태입니다!")
                else:
                    rospy.logwarn("   ⚠️ 로봇이 'kTaskable' 상태가 아닙니다.")
                    rospy.loginfo("   🔧 맵 기반 내비게이션은 불가하지만, 상대 거리 이동은 시도 가능합니다.")

                # === 7단계: 상대 거리 이동 (StepControl) - 맵 없이도 가능 ===
                rospy.loginfo("➡️ 7. [StepControl] 현재 위치 기준 0.2m 전진을 시작합니다...")
                rospy.loginfo("   💡 StepControl은 로봇의 로컬 좌표계 기준이므로 맵 없이도 작동할 수 있습니다.")
                
                step_control = StepControl()
                step_control.action = ControlAction.kExecute
                step = step_control.steps.add()
                step.mode = StepControl.Step.Mode.kStraight  # 직진 모드
                step.value = 0.2  # 이동 거리 (m)
                step.speed = 0.1  # 이동 속도 (m/s)
                
                call_action = CallAction(step_control=step_control)
                _, ok, msg = await self.robot.call_action_req(call_action)
                
                if not ok:
                    rospy.logerr(f"❌ [StepControl] 명령 전송 실패: {msg}")
                    rospy.logerr(f"   오류 메시지: {msg}")
                    rospy.logerr("")
                    rospy.logerr("   💡 대안: Twist 속도 제어 방식 (맵 불필요)을 시도합니다...")
                    
                    # 대안: Twist 속도 제어
                    rospy.loginfo("   ➡️ [Twist] 0.1m/s로 2초간 전진 (총 0.2m 이동)")
                    move_twist = Twist(linear=0.1, angular=0.0)
                    _, twist_ok, twist_msg = await self.robot.twist_req(move_twist)
                    
                    if twist_ok:
                        rospy.loginfo("   ✅ [Twist] 전진 시작!")
                        await asyncio.sleep(2.0)  # 2초 대기
                        
                        # 정지
                        stop_twist = Twist(linear=0.0, angular=0.0)
                        await self.robot.twist_req(stop_twist)
                        rospy.loginfo("   ✅ [Twist] 정지 완료.")
                    else:
                        rospy.logerr(f"   ❌ [Twist] 속도 제어도 실패: {twist_msg}")
                else:
                    rospy.loginfo("✅ [StepControl] 명령 전송 성공! 로봇이 0.2m 전진합니다.")
                    rospy.loginfo("   💡 로봇의 이동을 확인하세요.")
                    
                    # 이동 완료 대기 (피드백 구독)
                    rospy.loginfo("   ⏳ 이동 완료를 기다리는 중... (최대 10초)")
                    await asyncio.sleep(10.0)

            except Exception as e:
                rospy.logerr(f"❌ [이동 테스트] 실행 중 예외 발생: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            rospy.loginfo("--------------------")

            # rospy.loginfo("--- [이동 테스트] ---")
            # rospy.loginfo("➡️ [Twist] 0.1m/s 속도로 1초간 전진합니다...")
            
            # try:
            #     # 1. 전진(0.1m/s) 메시지 생성
            #     move_twist = Twist(linear=-0.1, angular=0.0)
                
            #     # 2. 전진 명령 전송
            #     _, ok, msg = await self.robot.twist_req(move_twist)
                
            #     if not ok:
            #         rospy.logerr(f"❌ [Twist] 전진 명령 전송 실패: {msg}")
            #     else:
            #         rospy.loginfo("✅ [Twist] 전진 명령 성공. 1초간 대기...")
                    
            #         # 3. 1초간 대기 (이 시간 동안 로봇이 움직여야 함)
            #         await asyncio.sleep(2.0)

            #         # 4. 정지(0m/s) 메시지 생성
            #         rospy.loginfo("➡️ [Twist] 정지 명령을 전송합니다...")
            #         stop_twist = Twist(linear=0.0, angular=0.0)
                    
            #         # 5. 정지 명령 전송
            #         _, ok, msg = await self.robot.twist_req(stop_twist)
            #         if not ok:
            #             rospy.logerr(f"❌ [Twist] 정지 명령 전송 실패: {msg}")
            #         else:
            #             rospy.loginfo("✅ [Twist] 정지 명령 성공.")

            # except Exception as e:
            #     rospy.logerr(f"❌ [Twist] 실행 중 예외 발생: {e}")
            # rospy.loginfo("--------------------")

            # 5. 'rospy.spin()'의 비동기 버전
            #     Ctrl+C 신호가 들어올 때까지 무한정 대기
            await asyncio.Event().wait()

        except asyncio.CancelledError:
            # Ctrl+C가 입력되면 asyncio.run()이 해당 작업을 취소시킴
            rospy.loginfo("🛑 작업 취소됨. 로봇 연결을 종료합니다.")

        except Exception as e:
            # SDK 실행 중 예기치 않은 오류 발생 시 동작
            rospy.logfatal(f"🔥 치명적 오류 발생: {e}")
        
        finally:
            # 5. Ctrl+C 또는 예외 발생 시 로봇 연결 종료
            if self.robot and hasattr(self.robot, 'is_running') and self.robot.is_running():
            # if self.robot.is_running():
                rospy.loginfo(f"📋 로봇 연결 종료 중...")  
                await self.robot.stop()
                rospy.loginfo(f"📋 로봇 연결 종료 완료!")


# --- 메인 실행 ---
if __name__ == "__main__":
    try:
        # 1. 클래스 인스턴스 생성 (__init__이 실행됨)
        node = RobotConnectionNode()

        # 2. 비동기 메인 루프 실행
        asyncio.run(node.run())

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("👋 노드를 종료합니다.")
    except Exception as e:
        if not rospy.is_shutdown():
            rospy.logfatal(f"💥 노드 초기화 실패: {e}")