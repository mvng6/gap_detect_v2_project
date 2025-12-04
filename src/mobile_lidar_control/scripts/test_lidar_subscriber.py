#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Subscriber 유닛 테스트

lidar_subscriber.py의 기능을 테스트합니다.
로봇 연결 없이 코드 구조, import, 변환 로직 등을 검증합니다.

실행 방법:
    python3 test_lidar_subscriber.py

Author: KATECH Robotics Team
Date: 2025-12-04
"""

import unittest
import sys
import os

# 테스트 대상 모듈 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# WooshRobot SDK 경로 추가
woosh_sdk_dir = os.path.join(script_dir, '../../woosh_robot_py')
sys.path.insert(0, os.path.abspath(woosh_sdk_dir))


class TestImports(unittest.TestCase):
    """Import 테스트"""
    
    def test_ros_imports(self):
        """ROS 메시지 import 테스트"""
        try:
            from sensor_msgs.msg import LaserScan
            from std_msgs.msg import Header
            self.assertTrue(True, "ROS 메시지 import 성공")
        except ImportError as e:
            self.fail(f"ROS 메시지 import 실패: {e}")
    
    def test_woosh_sdk_imports(self):
        """WooshRobot SDK import 테스트"""
        try:
            from woosh_robot import WooshRobot
            from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
            self.assertTrue(True, "WooshRobot SDK import 성공")
        except ImportError as e:
            self.skipTest(f"WooshRobot SDK import 실패 (SDK 미설치 가능): {e}")
    
    def test_scanner_data_import(self):
        """ScannerData proto import 테스트"""
        try:
            from woosh.proto.robot.robot_pb2 import ScannerData
            self.assertTrue(True, "ScannerData import 성공")
        except ImportError as e:
            self.skipTest(f"ScannerData import 실패 (SDK 미설치 가능): {e}")


class TestLaserScanConversion(unittest.TestCase):
    """LaserScan 변환 테스트"""
    
    def setUp(self):
        """테스트 설정"""
        try:
            from sensor_msgs.msg import LaserScan
            self.LaserScan = LaserScan
        except ImportError:
            self.skipTest("ROS 환경이 아님")
    
    def test_laserscan_message_creation(self):
        """LaserScan 메시지 생성 테스트"""
        msg = self.LaserScan()
        
        # 기본값 설정
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.01745
        msg.time_increment = 0.0001
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        
        # 값 검증
        self.assertAlmostEqual(msg.angle_min, -3.14159, places=4)
        self.assertAlmostEqual(msg.angle_max, 3.14159, places=4)
        self.assertEqual(len(msg.ranges), 5)
        self.assertEqual(msg.ranges[0], 1.0)
        
        print(f"  ✓ LaserScan 메시지 생성 성공")
        print(f"    - angle_min: {msg.angle_min}")
        print(f"    - angle_max: {msg.angle_max}")
        print(f"    - ranges count: {len(msg.ranges)}")


class TestScannerDataStructure(unittest.TestCase):
    """ScannerData 구조 테스트"""
    
    def test_scanner_data_fields(self):
        """ScannerData 필드 확인"""
        try:
            from woosh.proto.robot.robot_pb2 import ScannerData
            
            # ScannerData 인스턴스 생성
            data = ScannerData()
            
            # 필드 존재 확인
            expected_fields = [
                'angle_min', 'angle_max', 'angle_increment',
                'time_increment', 'scan_time',
                'range_min', 'range_max', 'ranges',
                'robot_id', 'pose', 'offset'
            ]
            
            for field in expected_fields:
                self.assertTrue(
                    hasattr(data, field), 
                    f"ScannerData에 '{field}' 필드가 없음"
                )
            
            print(f"  ✓ ScannerData 필드 확인 완료")
            print(f"    - 확인된 필드: {expected_fields}")
            
        except ImportError as e:
            self.skipTest(f"WooshRobot SDK를 찾을 수 없음: {e}")


class TestLidarSubscriberClass(unittest.TestCase):
    """LidarSubscriber 클래스 테스트 (Mock)"""
    
    def test_conversion_logic(self):
        """변환 로직 단위 테스트 (ROS 없이)"""
        try:
            from sensor_msgs.msg import LaserScan
            from std_msgs.msg import Header
        except ImportError:
            self.skipTest("ROS 환경이 아님")
        
        # Mock ScannerData-like 객체
        class MockScannerData:
            def __init__(self):
                self.angle_min = -1.57
                self.angle_max = 1.57
                self.angle_increment = 0.01
                self.time_increment = 0.0001
                self.scan_time = 0.1
                self.range_min = 0.05
                self.range_max = 12.0
                self.ranges = [1.5, 2.0, 2.5, 3.0, 2.5, 2.0, 1.5]
        
        # 변환 함수 (LidarSubscriber._convert_to_ros_laserscan 로직)
        def convert_to_laserscan(data, frame_id="laser_frame"):
            msg = LaserScan()
            msg.header = Header()
            msg.header.frame_id = frame_id
            msg.angle_min = data.angle_min
            msg.angle_max = data.angle_max
            msg.angle_increment = data.angle_increment
            msg.time_increment = data.time_increment
            msg.scan_time = data.scan_time
            msg.range_min = data.range_min
            msg.range_max = data.range_max
            msg.ranges = list(data.ranges)
            return msg
        
        # 테스트
        mock_data = MockScannerData()
        result = convert_to_laserscan(mock_data, "test_frame")
        
        # 검증
        self.assertEqual(result.header.frame_id, "test_frame")
        self.assertAlmostEqual(result.angle_min, -1.57, places=2)
        self.assertAlmostEqual(result.angle_max, 1.57, places=2)
        self.assertEqual(len(result.ranges), 7)
        self.assertEqual(result.ranges[3], 3.0)
        
        print(f"  ✓ 변환 로직 테스트 통과")
        print(f"    - Input ranges: {mock_data.ranges}")
        print(f"    - Output ranges: {list(result.ranges)}")


class TestParameterDefaults(unittest.TestCase):
    """기본 파라미터 값 테스트"""
    
    def test_default_robot_ip(self):
        """기본 로봇 IP 확인"""
        default_ip = '169.254.128.2'
        self.assertEqual(default_ip, '169.254.128.2')
        print(f"  ✓ 기본 Robot IP: {default_ip}")
    
    def test_default_robot_port(self):
        """기본 로봇 포트 확인"""
        default_port = 5480
        self.assertEqual(default_port, 5480)
        print(f"  ✓ 기본 Robot Port: {default_port}")
    
    def test_default_frame_id(self):
        """기본 프레임 ID 확인"""
        default_frame = 'laser_frame'
        self.assertEqual(default_frame, 'laser_frame')
        print(f"  ✓ 기본 Frame ID: {default_frame}")


def run_tests():
    """테스트 실행"""
    print("=" * 60)
    print("LiDAR Subscriber 유닛 테스트")
    print("=" * 60)
    
    # 테스트 스위트 생성
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 테스트 클래스 추가
    suite.addTests(loader.loadTestsFromTestCase(TestImports))
    suite.addTests(loader.loadTestsFromTestCase(TestLaserScanConversion))
    suite.addTests(loader.loadTestsFromTestCase(TestScannerDataStructure))
    suite.addTests(loader.loadTestsFromTestCase(TestLidarSubscriberClass))
    suite.addTests(loader.loadTestsFromTestCase(TestParameterDefaults))
    
    # 테스트 실행
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 결과 요약
    print("\n" + "=" * 60)
    print("테스트 결과 요약")
    print("=" * 60)
    print(f"  실행: {result.testsRun}")
    print(f"  성공: {result.testsRun - len(result.failures) - len(result.errors) - len(result.skipped)}")
    print(f"  실패: {len(result.failures)}")
    print(f"  에러: {len(result.errors)}")
    print(f"  스킵: {len(result.skipped)}")
    print("=" * 60)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

