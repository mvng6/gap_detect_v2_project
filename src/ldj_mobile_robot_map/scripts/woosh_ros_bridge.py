#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
woosh 모바일 로봇(TR-200) → ROS 브릿지 노드

패키지: ldj_mobile_robot_map
작성자: LDJ (KATECH)
설명: woosh SDK 데이터를 ROS 토픽으로 변환하여 발행

Phase 2-1: LiDAR 데이터 변환 (ScannerData → /scan)
Phase 2-2: Odometry 데이터 변환 (PoseSpeed → /odom)
"""

import rospy
import asyncio
import math
import sys
import os
from threading import Thread

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

# woosh SDK import
script_dir = os.path.dirname(os.path.abspath(__file__))
woosh_sdk_dir = os.path.join(script_dir, '../../woosh_robot_py')
sys.path.insert(0, os.path.abspath(woosh_sdk_dir))

from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
from woosh.proto.robot.robot_pb2 import (
    RobotInfo, 
    PoseSpeed, 
    ScannerData,
    OperationState
)
from woosh.proto.robot.robot_pack_pb2 import (
    SwitchMap, 
    SetRobotPose, 
    InitRobot, 
    SwitchControlMode,
    BuildMapData
)
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.robot_pb2 import ControlMode


class WooshRosBridge:
    """
    woosh SDK 데이터를 ROS 토픽으로 변환하는 브릿지 노드
    """
    
    def __init__(self):
        # ROS 파라미터 로드
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'ros_bridge')
        
        # 프레임 ID 설정
        self.laser_frame_id = rospy.get_param('~laser_frame_id', 'laser')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        
        # woosh 로봇 객체
        self.robot = None
        self.is_connected = False
        
        # === ROS 퍼블리셔 설정 ===
        # LiDAR 데이터 퍼블리셔
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # Odometry 데이터 퍼블리셔
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        
        # 통계 카운터
        self.scan_count = 0
        self.odom_count = 0
        
        rospy.loginfo("WooshRosBridge 노드 초기화 완료")
        rospy.loginfo(f"  - 로봇 IP: {self.robot_ip}")
        rospy.loginfo(f"  - 로봇 포트: {self.robot_port}")
        rospy.loginfo(f"  - 레이저 프레임: {self.laser_frame_id}")
        rospy.loginfo(f"  - Odom 프레임: {self.odom_frame_id}")
        rospy.loginfo(f"  - Base 프레임: {self.base_frame_id}")

    async def connect(self):
        """로봇 연결"""
        rospy.loginfo("=== 로봇 연결 시작 ===")
        
        settings = CommuSettings(
            addr=self.robot_ip, 
            port=self.robot_port, 
            identity=self.robot_identity
        )
        self.robot = WooshRobot(settings)
        
        try:
            await self.robot.run()
            
            # 연결 확인
            info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
            if not ok:
                rospy.logerr(f"로봇 연결 실패: {msg}")
                return False
            
            self.is_connected = True
            rospy.loginfo(f"✅ 로봇 연결 성공!")
            rospy.loginfo(f"   배터리 잔량: {info.battery.power}%")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"로봇 연결 예외: {e}")
            return False

    async def setup_map(self):
        """맵 로드 및 로컬라이제이션 설정"""
        rospy.loginfo("=== 맵 설정 시작 ===")
        
        # 현재 맵 상태 확인
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logwarn("위치 정보 요청 실패")
            return False
        
        current_map_id = pose_speed.map_id if hasattr(pose_speed, 'map_id') else 0
        
        if current_map_id != 0:
            rospy.loginfo(f"✅ 맵이 이미 로드되어 있습니다. (ID: {current_map_id})")
            return True
        
        # 맵 목록 확인
        scene_list_req = SceneList()
        scene_list, ok, msg = await self.robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        if not ok or not scene_list or not scene_list.scenes:
            rospy.logwarn("사용 가능한 맵이 없습니다.")
            return False
        
        available_scenes = [scene.name for scene in scene_list.scenes]
        rospy.loginfo(f"사용 가능한 맵: {available_scenes}")
        
        # 3번째 맵(인덱스 2) 선택 (ldj_load_map.py와 동일)
        if len(available_scenes) > 2:
            target_scene = available_scenes[2]
        else:
            target_scene = available_scenes[0]
        
        rospy.loginfo(f"맵 로드 시도: {target_scene}")
        
        # 맵 로드
        switch_map = SwitchMap()
        switch_map.scene_name = target_scene
        result, ok, msg = await self.robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
        
        if not ok:
            rospy.logerr(f"맵 로드 실패: {msg}")
            return False
        
        rospy.loginfo(f"✅ 맵 '{target_scene}' 로드 완료")
        
        # 로봇 초기화
        await asyncio.sleep(2)
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            init_robot = InitRobot()
            init_robot.is_record = False
            init_robot.pose.x = pose_speed.pose.x
            init_robot.pose.y = pose_speed.pose.y
            init_robot.pose.theta = pose_speed.pose.theta
            
            await self.robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
            rospy.loginfo("✅ 로봇 초기화 완료")
        
        return True

    def scanner_data_callback(self, data: ScannerData):
        """
        woosh ScannerData를 ROS LaserScan 메시지로 변환하여 발행
        
        Args:
            data: woosh SDK의 ScannerData protobuf 메시지
        """
        try:
            # LaserScan 메시지 생성
            scan_msg = LaserScan()
            
            # 헤더 설정
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = self.laser_frame_id
            
            # 스캔 파라미터 설정
            scan_msg.angle_min = data.angle_min
            scan_msg.angle_max = data.angle_max
            scan_msg.angle_increment = data.angle_increment
            scan_msg.time_increment = data.time_increment
            scan_msg.scan_time = data.scan_time
            scan_msg.range_min = data.range_min
            scan_msg.range_max = data.range_max
            
            # 거리 데이터 변환
            scan_msg.ranges = list(data.ranges)
            
            # intensities는 woosh에서 제공하지 않으므로 빈 리스트
            scan_msg.intensities = []
            
            # 퍼블리시
            self.scan_pub.publish(scan_msg)
            
            # 통계 업데이트
            self.scan_count += 1
            
            # 100회마다 로그 출력
            if self.scan_count % 100 == 0:
                rospy.loginfo(f"[LiDAR] 발행 횟수: {self.scan_count}, "
                            f"범위 데이터 수: {len(scan_msg.ranges)}, "
                            f"각도 범위: [{math.degrees(data.angle_min):.1f}° ~ {math.degrees(data.angle_max):.1f}°]")
                
        except Exception as e:
            rospy.logerr(f"LiDAR 데이터 변환 오류: {e}")

    async def subscribe_scanner_data(self):
        """LiDAR 데이터 구독 시작"""
        rospy.loginfo("LiDAR 데이터 구독 시작...")
        
        success = await self.robot.scanner_data_sub(
            self.scanner_data_callback, 
            NO_PRINT
        )
        
        if success:
            rospy.loginfo("✅ LiDAR 데이터 구독 성공!")
        else:
            rospy.logerr("❌ LiDAR 데이터 구독 실패!")
        
        return success

    def yaw_to_quaternion(self, yaw):
        """
        yaw 각도(라디안)를 quaternion으로 변환
        
        Args:
            yaw: z축 회전 각도 (라디안)
        
        Returns:
            Quaternion 메시지
        """
        # 2D 로봇이므로 roll=0, pitch=0, yaw만 사용
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        return Quaternion(
            x=0.0,
            y=0.0,
            z=sy,
            w=cy
        )

    def pose_speed_callback(self, data: PoseSpeed):
        """
        woosh PoseSpeed를 ROS Odometry 메시지로 변환하여 발행
        
        Args:
            data: woosh SDK의 PoseSpeed protobuf 메시지
        """
        try:
            # Odometry 메시지 생성
            odom_msg = Odometry()
            
            # 헤더 설정
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.base_frame_id
            
            # 위치 설정 (pose)
            odom_msg.pose.pose.position = Point(
                x=data.pose.x,
                y=data.pose.y,
                z=0.0
            )
            
            # 방향 설정 (theta → quaternion)
            odom_msg.pose.pose.orientation = self.yaw_to_quaternion(data.pose.theta)
            
            # 속도 설정 (twist)
            odom_msg.twist.twist.linear = Vector3(
                x=data.twist.linear,
                y=0.0,
                z=0.0
            )
            odom_msg.twist.twist.angular = Vector3(
                x=0.0,
                y=0.0,
                z=data.twist.angular
            )
            
            # Covariance 설정 (불확실성 - 기본값 사용)
            # 대각 성분만 설정 (x, y, z, roll, pitch, yaw)
            pose_cov = [0.0] * 36
            pose_cov[0] = 0.01   # x
            pose_cov[7] = 0.01   # y
            pose_cov[14] = 1e6   # z (사용안함, 큰 값)
            pose_cov[21] = 1e6   # roll (사용안함)
            pose_cov[28] = 1e6   # pitch (사용안함)
            pose_cov[35] = 0.03  # yaw
            odom_msg.pose.covariance = pose_cov
            
            twist_cov = [0.0] * 36
            twist_cov[0] = 0.01   # linear x
            twist_cov[7] = 1e6    # linear y (사용안함)
            twist_cov[14] = 1e6   # linear z (사용안함)
            twist_cov[21] = 1e6   # angular x (사용안함)
            twist_cov[28] = 1e6   # angular y (사용안함)
            twist_cov[35] = 0.03  # angular z
            odom_msg.twist.covariance = twist_cov
            
            # 퍼블리시
            self.odom_pub.publish(odom_msg)
            
            # 통계 업데이트
            self.odom_count += 1
            
            # 100회마다 로그 출력
            if self.odom_count % 100 == 0:
                rospy.loginfo(f"[Odom] 발행 횟수: {self.odom_count}, "
                            f"위치: ({data.pose.x:.2f}, {data.pose.y:.2f}), "
                            f"방향: {math.degrees(data.pose.theta):.1f}°, "
                            f"속도: ({data.twist.linear:.2f} m/s, {math.degrees(data.twist.angular):.1f}°/s)")
                
        except Exception as e:
            rospy.logerr(f"Odometry 데이터 변환 오류: {e}")

    async def poll_pose_speed_data(self):
        """Odometry 데이터를 주기적으로 요청하여 발행 (polling 방식)"""
        # Odometry 발행 주기 설정 (Hz)
        odom_rate = rospy.get_param('~odom_rate', 20)  # 기본 20Hz
        interval = 1.0 / odom_rate
        
        rospy.loginfo(f"Odometry polling 시작 ({odom_rate}Hz)...")
        
        while not rospy.is_shutdown() and self.is_connected:
            try:
                # PoseSpeed 요청
                pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                    PoseSpeed(), NO_PRINT, NO_PRINT
                )
                
                if ok and pose_speed:
                    # 콜백 함수를 직접 호출하여 Odometry 발행
                    self.pose_speed_callback(pose_speed)
                else:
                    # 오류 발생 시 경고 (너무 자주 출력하지 않도록)
                    if self.odom_count % 100 == 0:
                        rospy.logwarn(f"PoseSpeed 요청 실패: {msg}")
                
                await asyncio.sleep(interval)
                
            except Exception as e:
                rospy.logerr(f"Odometry polling 오류: {e}")
                await asyncio.sleep(1.0)  # 오류 발생 시 1초 대기

    async def run(self):
        """메인 실행 루프"""
        # 1. 로봇 연결
        if not await self.connect():
            rospy.logerr("로봇 연결 실패. 종료합니다.")
            return
        
        # 2. 맵 설정
        await self.setup_map()
        
        # 3. LiDAR 데이터 구독 시작
        if not await self.subscribe_scanner_data():
            rospy.logerr("LiDAR 구독 실패. 종료합니다.")
            return
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("✅ WooshRosBridge 실행 중...")
        rospy.loginfo("   LiDAR 데이터 → /scan 토픽 발행 중")
        rospy.loginfo("   Odometry 데이터 → /odom 토픽 발행 중 (polling)")
        rospy.loginfo("=" * 50)
        rospy.loginfo("테스트 명령:")
        rospy.loginfo("  rostopic echo /scan")
        rospy.loginfo("  rostopic echo /odom")
        rospy.loginfo("  rostopic hz /scan /odom")
        rospy.loginfo("=" * 50)
        
        # 4. Odometry polling 루프 시작 (메인 루프)
        await self.poll_pose_speed_data()


def run_asyncio():
    """비동기 메인 함수"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    bridge = WooshRosBridge()
    
    async def main():
        try:
            await bridge.run()
        except Exception as e:
            rospy.logerr(f"브릿지 실행 오류: {e}")

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")
    finally:
        loop.close()


if __name__ == "__main__":
    rospy.init_node('woosh_ros_bridge', anonymous=False)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("woosh → ROS 브릿지 노드 시작")
    rospy.loginfo("=" * 50)
    
    # 비동기 스레드에서 실행
    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()
    
    # ROS 메인 스레드
    rospy.spin()

