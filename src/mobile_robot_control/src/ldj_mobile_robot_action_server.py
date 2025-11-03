#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import asyncio
import threading
from std_msgs.msg import String

# Action 메시지 타입 임포트
from robot_interfaces.msg import MoveMobileAction, MoveMobileGoal, MoveMobileResult, MoveMobileFeedback

# 기존 제어 로직 임포트 (파일을 수정하지 않고 클래스만 가져와 사용)
from mobile_robot_control_node import MobileRobotController, RobotConfig, VelocityProfileConfig

class LDJMobileRobotActionServer:
    def __init__(self):
        rospy.loginfo("🤖 모바일 로봇 액션 서버 초기화 시작...")

        # 로봇 상태를 발행할 퍼블리셔
        self._status_publisher = rospy.Publisher('/mobile_robot/status', String, queue_size=1)

        # 액션 서버 생성
        # 서버 이름: /move_mobile
        # 액션 타입: MoveMobileAction
        # 콜백 함수: self.execute_cb (새로운 목표(goal)가 들어오면 이 함수가 호출됨)
        self._server = actionlib.SimpleActionServer(
            '/move_mobile',
            MoveMobileAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        # 액션 서버 시작
        self._server.start()
        rospy.loginfo("✅ 모바일 로봇 액션 서버가 /move_mobile 토픽에서 목표를 기다리고 있습니다.")

    def execute_cb(self, goal: MoveMobileGoal):
        """
        새로운 Action Goal을 받았을 때 실행되는 메인 콜백 함수.
        asyncio 로직을 별도의 스레드에서 실행하여 ROS 이벤트 루프와 분리합니다.
        """
        rospy.loginfo(f"🎯 새로운 목표 수신: {goal.target_distance:.2f}m 이동 (최대 속도: {goal.max_speed:.2f}m/s)")

        # asyncio 코드를 실행할 별도의 스레드 생성 및 시작
        thread = threading.Thread(target=self.run_async_task, args=(goal,))
        thread.start()

        # 스레드가 완료될 때까지 대기 (이 시간 동안 피드백 수신 가능)
        thread.join()

        rospy.loginfo("- 스레드 작업 완료, 결과 처리 -")

        # 스레드에서 저장한 결과에 따라 Action 서버의 최종 상태 결정
        if hasattr(self, '_thread_result') and self._thread_result.success:
            rospy.loginfo("✅ Action 성공 처리")
            self._server.set_succeeded(self._thread_result)
        else:
            rospy.loginfo("❌ Action 실패 처리")
            # 실패 시에는 빈 결과(기본값)를 전송
            self._server.set_aborted(MoveMobileResult(success=False, final_distance=self._thread_result.final_distance))

    def run_async_task(self, goal: MoveMobileGoal):
        """
        별도의 스레드에서 asyncio 이벤트 루프를 실행하는 함수.
        """
        try:
            # 새 이벤트 루프를 얻거나 생성하여 현재 스레드의 기본 루프로 설정
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # 메인 비동기 로직 실행
            result = loop.run_until_complete(self.handle_movement(goal))
            self._thread_result = result
        except Exception as e:
            rospy.logerr(f"💥 Asyncio 태스크 실행 중 오류 발생: {e}")
            self._thread_result = MoveMobileResult(success=False, final_distance=0.0)
        finally:
            loop.close()

    async def handle_movement(self, goal: MoveMobileGoal) -> MoveMobileResult:
        """
        실제 로봇 연결 및 이동을 처리하는 비동기 함수.
        """
        # 로봇 연결 설정 (IP 등은 실제 환경에 맞게 조정 필요)
        robot_config = RobotConfig(ip='169.254.128.2', port=5480)
        controller = MobileRobotController(robot_config)

        try:
            await controller.connect()

            # 1. 상태 발행: "MOVING"
            self._status_publisher.publish(String(data="MOVING"))

            # 2. 이동 실행 및 피드백 발행
            # 이동이 완료될 때까지 주기적으로 피드백을 발행하는 태스크와
            # 실제 이동을 실행하는 태스크를 동시에 실행
            feedback_task = asyncio.create_task(self.publish_feedback(controller, goal.target_distance))

            motion_result = await controller.move_distance(
                target_distance=goal.target_distance,
                speed=goal.max_speed,
                velocity_config=VelocityProfileConfig(max_speed=abs(goal.max_speed))
            )

            # 피드백 태스크가 완료되도록 잠시 대기 후 취소
            await asyncio.sleep(0.1)
            feedback_task.cancel()

            # 3. 상태 발행: "STOPPED"
            self._status_publisher.publish(String(data="STOPPED"))

            # 4. 최종 결과 생성
            result = MoveMobileResult(
                success=motion_result.success,
                final_distance=motion_result.traveled_distance
            )
            rospy.loginfo(f"📊 이동 완료. 실제 이동 거리: {motion_result.traveled_distance:.3f}m")

        except Exception as e:
            rospy.logerr(f"💥 로봇 이동 처리 중 오류: {e}")
            self._status_publisher.publish(String(data="STOPPED")) # 오류 발생 시에도 정지 상태 발행
            result = MoveMobileResult(success=False, final_distance=0.0)
        finally:
            await controller.disconnect()

        return result

    async def publish_feedback(self, controller: MobileRobotController, target_distance: float):
        """
        주기적으로 로봇의 위치를 확인하고 Action Feedback을 발행하는 비동기 함수.
        """
        start_pose = await controller.get_current_pose()
        if not start_pose:
            rospy.logwarn("피드백 발행을 위한 시작 위치를 얻지 못했습니다.")
            return

        while not self._server.is_preempt_requested():
            await asyncio.sleep(0.2) # 5Hz

            current_pose = controller.current_pose # 콜백으로 업데이트되는 위치 사용
            if current_pose:
                traveled_distance = controller.calculate_distance(start_pose, current_pose)
                remaining = target_distance - traveled_distance

                # 피드백 메시지 생성 및 발행
                feedback = MoveMobileFeedback(distance_remaining=remaining)
                self._server.publish_feedback(feedback)

                # 목표에 거의 도달하면 루프 종료
                if remaining < 0.01:
                    break

        rospy.loginfo("⏹️ 피드백 발행 중단.")


if __name__ == '__main__':
    try:
        rospy.init_node('mobile_robot_action_server')
        server = LDJMobileRobotActionServer()
        rospy.spin() # ROS 이벤트 루프 시작 (콜백 대기)
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 프로그램 종료")
