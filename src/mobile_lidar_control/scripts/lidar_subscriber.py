#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Subscriber Node

모바일 로봇의 LiDAR 센서 데이터를 수신하여 ROS 토픽으로 발행하는 노드입니다.
WooshRobot SDK를 통해 로봇과 통신하고, ScannerData를 sensor_msgs/LaserScan으로 변환합니다.

Author: KATECH Robotics Team
Date: 2025-12-04
"""

import rospy
import asyncio
import sys
import os
from threading import Thread
from typing import Optional

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# === WooshRobot SDK 경로 추가 ===
# 현재 스크립트 디렉토리 기준으로 woosh_robot_py 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
woosh_sdk_dir = os.path.join(script_dir, '../../woosh_robot_py')
sys.path.insert(0, os.path.abspath(woosh_sdk_dir))

# WooshRobot SDK import
try:
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
    from woosh.proto.robot.robot_pb2 import ScannerData
except ImportError as e:
    print(f"[ERROR] WooshRobot SDK를 찾을 수 없습니다: {e}")
    print(f"[INFO] woosh_robot_py 경로: {woosh_sdk_dir}")
    sys.exit(1)


class LidarSubscriber:
    """
    LiDAR 데이터 구독 및 ROS 토픽 발행 클래스
    
    WooshRobot SDK를 사용하여 모바일 로봇의 LiDAR 센서 데이터를 구독하고,
    ROS sensor_msgs/LaserScan 메시지로 변환하여 발행합니다.
    """
    
    def __init__(self):
        """초기화: ROS 파라미터 로드 및 퍼블리셔 설정"""
        # === ROS 파라미터 로드 ===
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'lidar_subscriber')
        self.frame_id = rospy.get_param('~frame_id', 'laser_frame')
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.verbose = rospy.get_param('~verbose', False)
        
        # === 로봇 연결 객체 ===
        self.robot: Optional[WooshRobot] = None
        self.is_connected = False
        
        # === ROS 퍼블리셔 ===
        self.scan_pub = rospy.Publisher(
            '/mobile_lidar/scan', 
            LaserScan, 
            queue_size=10
        )
        
        # === 통계 정보 ===
        self.scan_count = 0
        self.last_scan_time = None
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("LiDAR Subscriber Node 초기화")
        rospy.loginfo(f"  Robot IP: {self.robot_ip}")
        rospy.loginfo(f"  Robot Port: {self.robot_port}")
        rospy.loginfo(f"  Frame ID: {self.frame_id}")
        rospy.loginfo("=" * 50)
    
    async def connect(self) -> bool:
        """
        로봇에 연결합니다.
        
        Returns:
            bool: 연결 성공 여부
        """
        try:
            rospy.loginfo(f"로봇 연결 시도: {self.robot_ip}:{self.robot_port}")
            
            # SDK 연결 설정
            settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity=self.robot_identity
            )
            
            # 로봇 인스턴스 생성 및 연결
            self.robot = WooshRobot(settings)
            
            if not await self.robot.run():
                rospy.logerr("로봇 연결 실패: run() 반환값 False")
                return False
            
            self.is_connected = True
            rospy.loginfo("✅ 로봇 연결 성공!")
            return True
            
        except Exception as e:
            rospy.logerr(f"로봇 연결 중 예외 발생: {e}")
            self.is_connected = False
            return False
    
    async def scanner_callback(self, data: ScannerData):
        """
        LiDAR 데이터 수신 콜백 함수
        
        WooshRobot SDK로부터 ScannerData를 수신하면 호출됩니다.
        데이터를 ROS LaserScan 메시지로 변환하여 발행합니다.
        
        Args:
            data: WooshRobot SDK의 ScannerData 메시지
        """
        try:
            # ScannerData → LaserScan 변환
            scan_msg = self._convert_to_ros_laserscan(data)
            
            # ROS 토픽 발행
            self.scan_pub.publish(scan_msg)
            
            # 통계 업데이트
            self.scan_count += 1
            self.last_scan_time = rospy.Time.now()
            
            # 상세 로그 (verbose 모드)
            if self.verbose and self.scan_count % 100 == 0:
                rospy.loginfo(f"[LiDAR] 스캔 수신: {self.scan_count}회, "
                             f"ranges: {len(data.ranges)}개")
                
        except Exception as e:
            rospy.logerr(f"LiDAR 콜백 처리 중 오류: {e}")
    
    def _convert_to_ros_laserscan(self, data: ScannerData) -> LaserScan:
        """
        WooshRobot ScannerData를 ROS LaserScan 메시지로 변환합니다.
        
        Args:
            data: WooshRobot SDK의 ScannerData
            
        Returns:
            LaserScan: ROS sensor_msgs/LaserScan 메시지
        """
        msg = LaserScan()
        
        # 헤더 설정
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        
        # 스캔 각도 정보
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        
        # 시간 정보
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        
        # 거리 범위
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        
        # 거리 측정값 (핵심 데이터)
        msg.ranges = list(data.ranges)
        
        # 강도값 (있을 경우)
        # WooshRobot SDK의 ScannerData에 intensities가 있으면 추가
        msg.intensities = []
        
        return msg
    
    async def start_subscription(self) -> bool:
        """
        LiDAR 데이터 구독을 시작합니다.
        
        Returns:
            bool: 구독 시작 성공 여부
        """
        if not self.is_connected or self.robot is None:
            rospy.logerr("로봇이 연결되지 않았습니다. 먼저 connect()를 호출하세요.")
            return False
        
        try:
            rospy.loginfo("LiDAR 데이터 구독 시작...")
            
            # scanner_data_sub()로 LiDAR 데이터 구독
            print_level = FULL_PRINT if self.verbose else NO_PRINT
            success = await self.robot.scanner_data_sub(
                self.scanner_callback, 
                print_level
            )
            
            if success:
                rospy.loginfo("✅ LiDAR 구독 시작 성공!")
            else:
                rospy.logerr("LiDAR 구독 시작 실패")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"LiDAR 구독 시작 중 오류: {e}")
            return False
    
    async def run(self):
        """
        메인 실행 루프
        
        로봇 연결 → LiDAR 구독 → 무한 대기
        """
        # 1. 로봇 연결
        if not await self.connect():
            rospy.logerr("로봇 연결 실패. 프로그램을 종료합니다.")
            return
        
        # 2. LiDAR 구독 시작
        if not await self.start_subscription():
            rospy.logerr("LiDAR 구독 실패. 프로그램을 종료합니다.")
            return
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("LiDAR Subscriber 노드가 실행 중입니다.")
        rospy.loginfo("토픽: /mobile_lidar/scan")
        rospy.loginfo("종료하려면 Ctrl+C를 누르세요.")
        rospy.loginfo("=" * 50)
        
        # 3. 무한 대기 (구독 콜백이 비동기로 호출됨)
        try:
            while not rospy.is_shutdown():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            rospy.loginfo("LiDAR Subscriber 종료 중...")
        
        rospy.loginfo(f"총 수신 스캔: {self.scan_count}회")


# === Asyncio와 ROS 통합을 위한 헬퍼 함수 ===

def run_asyncio_loop(subscriber: LidarSubscriber):
    """
    별도 스레드에서 asyncio 이벤트 루프를 실행합니다.
    
    Args:
        subscriber: LidarSubscriber 인스턴스
    """
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(subscriber.run())
    except Exception as e:
        rospy.logerr(f"Asyncio 루프 오류: {e}")
    finally:
        loop.close()


def main():
    """메인 함수: ROS 노드 초기화 및 실행"""
    # ROS 노드 초기화
    rospy.init_node('lidar_subscriber', anonymous=False)
    
    # LidarSubscriber 인스턴스 생성
    subscriber = LidarSubscriber()
    
    # 별도 스레드에서 asyncio 루프 실행
    asyncio_thread = Thread(target=run_asyncio_loop, args=(subscriber,), daemon=True)
    asyncio_thread.start()
    
    # ROS spin (메인 스레드에서 ROS 콜백 처리)
    rospy.loginfo("ROS spin 시작...")
    rospy.spin()
    
    rospy.loginfo("LiDAR Subscriber 노드 종료")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")
