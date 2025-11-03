#!/usr/bin/env python3
"""
모바일 로봇 ROS 통합 노드
기존 MobileRobotTwistController를 ROS 토픽 기반으로 제어
"""
import sys
import os
import rospy
import asyncio
import threading
from std_msgs.msg import String, Float64MultiArray

# Python 모듈 직접 로드 (import 문제 해결)
import importlib.util

# mobile_robot_twist_control.py 직접 로드
module_path = '/root/catkin_ws/src/mobile_robot_control/src/mobile_robot_twist_control.py'
spec = importlib.util.spec_from_file_location("mobile_robot_twist_control", module_path)
mobile_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mobile_module)

# 클래스 가져오기
MobileRobotTwistController = mobile_module.MobileRobotTwistController


class MobileRobotROSNode:
    """ROS 토픽을 통해 모바일 로봇을 제어하는 노드"""
    
    def __init__(self):
        rospy.init_node('mobile_robot_ros_node', anonymous=False)
        
        # 상태 발행자
        self.status_pub = rospy.Publisher('/mobile/status', String, queue_size=1)
        
        # 명령 구독자
        self.cmd_sub = rospy.Subscriber('/mobile/cmd', Float64MultiArray, self.command_callback)
        
        # 제어 클래스 (비동기 작업용)
        self.controller = None
        self.current_status = "IDLE"
        
        # 비동기 이벤트 루프 (별도 스레드에서 실행)
        self.loop = None
        self.loop_thread = None
        
        rospy.loginfo("🤖 모바일 로봇 ROS 노드 초기화 완료")
        self.publish_status("IDLE")
        
    def publish_status(self, status):
        """상태 발행"""
        self.current_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo(f"📊 상태 발행: {status}")
    
    def command_callback(self, msg):
        """
        명령 수신 콜백
        msg.data = [distance, speed]
        """
        if len(msg.data) < 2:
            rospy.logerr("❌ 잘못된 명령 형식. [distance, speed] 필요")
            return
        
        distance = msg.data[0]
        speed = msg.data[1]
        
        rospy.loginfo(f"🎯 명령 수신: {distance}m 이동, 속도 {speed}m/s")
        
        # 이미 실행 중이면 무시
        if self.current_status == "MOVING":
            rospy.logwarn("⚠️ 이미 이동 중입니다. 명령 무시.")
            return
        
        # 비동기 작업을 별도 스레드에서 실행
        thread = threading.Thread(target=self.execute_movement, args=(distance, speed))
        thread.daemon = True
        thread.start()
    
    def execute_movement(self, distance, speed):
        """
        별도 스레드에서 비동기 이동 실행
        """
        try:
            # 새 이벤트 루프 생성
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 이동 실행
            loop.run_until_complete(self._async_move(distance, speed))
        except Exception as e:
            rospy.logerr(f"💥 이동 중 오류: {e}")
            self.publish_status("ERROR")
        finally:
            loop.close()
    
    async def _async_move(self, distance, speed):
        """실제 비동기 이동 로직"""
        self.publish_status("MOVING")
        
        try:
            # 컨트롤러 생성 및 연결 (init_node=False: 이미 노드 초기화됨)
            controller = MobileRobotTwistController(verbose=False, init_node=False)
            await controller.connect()
            
            # 이동 실행
            success, final_distance = await controller.move_distance(
                target_distance=distance,
                speed=speed,
                accel_distance=0.15,
                decel_distance=0.2
            )
            
            # 연결 종료
            await controller.stop()
            
            # 결과에 따라 상태 변경
            if success:
                rospy.loginfo("✅ 이동 완료!")
                self.publish_status("COMPLETED")
            else:
                rospy.logerr("❌ 이동 실패")
                self.publish_status("ERROR")
                
        except Exception as e:
            rospy.logerr(f"💥 이동 실행 오류: {e}")
            self.publish_status("ERROR")
    
    def run(self):
        """노드 실행 (ROS spin)"""
        rospy.loginfo("🚀 모바일 로봇 ROS 노드 실행 중...")
        rospy.loginfo("   명령 대기: /mobile/cmd [distance, speed]")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MobileRobotROSNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 노드 종료")
