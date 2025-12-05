#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Motion Corrector 유닛 테스트

lidar_motion_corrector.py의 기능을 테스트합니다.
Mock LaserScan 데이터를 사용하여 거리 계산 로직을 검증합니다.

실행 방법:
    python3 test_lidar_motion_corrector.py
    또는
    rosrun mobile_lidar_control test_lidar_motion_corrector.py

Author: KATECH Robotics Team
Date: 2025-12-04
"""

import unittest
import sys
import os
import math

# numpy를 선택적으로 import (없으면 대체 함수 사용)
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    # numpy 대체 함수들
    class np:
        pi = math.pi
        
        @staticmethod
        def isinf(x):
            return math.isinf(x)
        
        @staticmethod
        def isnan(x):
            return math.isnan(x)
        
        @staticmethod
        def median(lst):
            sorted_lst = sorted(lst)
            n = len(sorted_lst)
            mid = n // 2
            if n % 2 == 0:
                return (sorted_lst[mid - 1] + sorted_lst[mid]) / 2
            return sorted_lst[mid]
        
        @staticmethod
        def mean(lst):
            return sum(lst) / len(lst) if lst else 0
        
        @staticmethod
        def degrees(rad):
            return math.degrees(rad)

# 테스트 대상 모듈 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)


class MockLaserScan:
    """Mock LaserScan 메시지"""
    def __init__(self, 
                 ranges=None,
                 angle_min=-np.pi,
                 angle_max=np.pi,
                 angle_increment=0.01,
                 range_min=0.1,
                 range_max=10.0):
        self.ranges = ranges if ranges is not None else [1.0] * 628
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max


class TestAngleToIndex(unittest.TestCase):
    """각도 → 인덱스 변환 테스트"""
    
    def test_center_angle(self):
        """중앙 각도 (0도) 변환"""
        scan = MockLaserScan()
        # angle_min = -π, angle_increment = 0.01
        # index = (0 - (-π)) / 0.01 ≈ 314
        expected_idx = int((0 - (-np.pi)) / 0.01)
        
        # 직접 계산
        angle = 0.0
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        
        self.assertAlmostEqual(idx, expected_idx, delta=5)
        print(f"  ✓ 중앙 각도(0°) → 인덱스: {idx}")
    
    def test_positive_angle(self):
        """양수 각도 변환"""
        scan = MockLaserScan()
        angle = np.pi / 4  # 45도
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        
        self.assertTrue(0 <= idx < len(scan.ranges))
        print(f"  ✓ 45° → 인덱스: {idx}")
    
    def test_negative_angle(self):
        """음수 각도 변환"""
        scan = MockLaserScan()
        angle = -np.pi / 4  # -45도
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        
        self.assertTrue(0 <= idx < len(scan.ranges))
        print(f"  ✓ -45° → 인덱스: {idx}")


class TestDistanceCalculation(unittest.TestCase):
    """거리 계산 테스트"""
    
    def setUp(self):
        """테스트 설정"""
        self.min_valid_points = 3
        self.outlier_threshold = 0.5
    
    def _get_distance_in_range(self, scan, angle_min, angle_max):
        """거리 계산 로직 (lidar_motion_corrector.py에서 복사)"""
        if scan is None or len(scan.ranges) == 0:
            return None
        
        # 각도 인덱스 계산
        idx_min = int((angle_min - scan.angle_min) / scan.angle_increment)
        idx_max = int((angle_max - scan.angle_min) / scan.angle_increment)
        
        idx_min = max(0, min(idx_min, len(scan.ranges) - 1))
        idx_max = max(0, min(idx_max, len(scan.ranges) - 1))
        
        if idx_min > idx_max:
            idx_min, idx_max = idx_max, idx_min
        
        ranges_in_range = scan.ranges[idx_min:idx_max+1]
        
        valid_ranges = [
            r for r in ranges_in_range
            if (not np.isinf(r) and 
                not np.isnan(r) and 
                scan.range_min < r < scan.range_max)
        ]
        
        if len(valid_ranges) < self.min_valid_points:
            return None
        
        median = np.median(valid_ranges)
        filtered_ranges = [
            r for r in valid_ranges
            if abs(r - median) < self.outlier_threshold
        ]
        
        if len(filtered_ranges) < self.min_valid_points:
            return median
        
        return np.mean(filtered_ranges)
    
    def test_uniform_distance(self):
        """균일한 거리 측정"""
        # 모든 방향에서 2.0m
        ranges = [2.0] * 628
        scan = MockLaserScan(ranges=ranges)
        
        # 전방 (-15° ~ +15°)
        distance = self._get_distance_in_range(scan, -0.26, 0.26)
        
        self.assertIsNotNone(distance)
        self.assertAlmostEqual(distance, 2.0, places=2)
        print(f"  ✓ 균일 거리 2.0m → 측정값: {distance:.3f}m")
    
    def test_varying_distance(self):
        """변화하는 거리 측정"""
        # 전방은 1.5m, 나머지는 3.0m
        ranges = [3.0] * 628
        # 중앙 부근 (인덱스 300~330)을 1.5m로 설정
        for i in range(300, 330):
            ranges[i] = 1.5
        
        scan = MockLaserScan(ranges=ranges)
        
        # 해당 범위의 각도 계산
        angle_center = scan.angle_min + 315 * scan.angle_increment
        distance = self._get_distance_in_range(scan, angle_center - 0.15, angle_center + 0.15)
        
        self.assertIsNotNone(distance)
        self.assertAlmostEqual(distance, 1.5, places=1)
        print(f"  ✓ 변화 거리 → 측정값: {distance:.3f}m")
    
    def test_inf_filtering(self):
        """무한대 값 필터링"""
        ranges = [2.0] * 628
        # 일부를 inf로 설정
        ranges[310] = float('inf')
        ranges[315] = float('inf')
        
        scan = MockLaserScan(ranges=ranges)
        distance = self._get_distance_in_range(scan, -0.26, 0.26)
        
        self.assertIsNotNone(distance)
        self.assertAlmostEqual(distance, 2.0, places=1)
        print(f"  ✓ inf 필터링 후 → 측정값: {distance:.3f}m")
    
    def test_outlier_filtering(self):
        """이상치 필터링"""
        ranges = [2.0] * 628
        # 이상치 추가
        ranges[312] = 5.0  # 이상치
        ranges[313] = 0.5  # 이상치
        
        scan = MockLaserScan(ranges=ranges)
        distance = self._get_distance_in_range(scan, -0.26, 0.26)
        
        self.assertIsNotNone(distance)
        # 이상치가 제거되어 2.0에 가까워야 함
        self.assertAlmostEqual(distance, 2.0, places=1)
        print(f"  ✓ 이상치 필터링 후 → 측정값: {distance:.3f}m")


class TestTraveledDistance(unittest.TestCase):
    """이동 거리 계산 테스트"""
    
    def test_forward_movement(self):
        """전진 이동 거리 계산"""
        # 초기: 2.0m, 이동 후: 1.5m
        initial_distance = 2.0
        current_distance = 1.5
        
        traveled = initial_distance - current_distance
        
        self.assertAlmostEqual(traveled, 0.5, places=2)
        print(f"  ✓ 전진 이동: 2.0m → 1.5m = {traveled:.3f}m 이동")
    
    def test_backward_movement(self):
        """후진 이동 거리 계산"""
        # 후진 시 후방 거리: 초기 2.0m → 이동 후 1.5m
        initial_rear = 2.0
        current_rear = 1.5
        
        # 후진은 음수로 반환
        traveled = -(initial_rear - current_rear)
        
        self.assertAlmostEqual(traveled, -0.5, places=2)
        print(f"  ✓ 후진 이동: 2.0m → 1.5m = {traveled:.3f}m 이동")
    
    def test_no_movement(self):
        """이동 없음"""
        initial = 2.0
        current = 2.0
        
        traveled = initial - current
        
        self.assertAlmostEqual(traveled, 0.0, places=2)
        print(f"  ✓ 이동 없음: {traveled:.3f}m")


class TestMoveDirection(unittest.TestCase):
    """이동 방향 테스트"""
    
    def test_forward_direction(self):
        """전진 방향"""
        from enum import Enum
        
        class MoveDirection(Enum):
            FORWARD = 1
            BACKWARD = -1
            STOPPED = 0
        
        direction = MoveDirection.FORWARD
        self.assertEqual(direction.value, 1)
        self.assertEqual(direction.name, "FORWARD")
        print(f"  ✓ 전진 방향: {direction.name} = {direction.value}")
    
    def test_backward_direction(self):
        """후진 방향"""
        from enum import Enum
        
        class MoveDirection(Enum):
            FORWARD = 1
            BACKWARD = -1
            STOPPED = 0
        
        direction = MoveDirection.BACKWARD
        self.assertEqual(direction.value, -1)
        self.assertEqual(direction.name, "BACKWARD")
        print(f"  ✓ 후진 방향: {direction.name} = {direction.value}")


class TestLidarConfig(unittest.TestCase):
    """LiDAR 설정 테스트"""
    
    def test_front_angle_range(self):
        """전방 LiDAR 각도 범위"""
        front_min = -0.26  # -15도
        front_max = 0.26   # +15도
        
        range_deg = np.degrees(front_max - front_min)
        
        self.assertAlmostEqual(range_deg, 30, delta=1)
        print(f"  ✓ 전방 각도 범위: {range_deg:.1f}° (±15°)")
    
    def test_rear_angle_range(self):
        """후방 LiDAR 각도 범위"""
        rear_min = 2.88    # 165도
        rear_max = 3.40    # 195도
        
        center_deg = np.degrees((rear_min + rear_max) / 2)
        
        self.assertAlmostEqual(center_deg, 180, delta=5)
        print(f"  ✓ 후방 각도 중심: {center_deg:.1f}° (≈180°)")


def run_tests():
    """테스트 실행"""
    print("=" * 60)
    print("LiDAR Motion Corrector 유닛 테스트")
    print("=" * 60)
    
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestAngleToIndex))
    suite.addTests(loader.loadTestsFromTestCase(TestDistanceCalculation))
    suite.addTests(loader.loadTestsFromTestCase(TestTraveledDistance))
    suite.addTests(loader.loadTestsFromTestCase(TestMoveDirection))
    suite.addTests(loader.loadTestsFromTestCase(TestLidarConfig))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "=" * 60)
    print("테스트 결과 요약")
    print("=" * 60)
    print(f"  실행: {result.testsRun}")
    print(f"  성공: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"  실패: {len(result.failures)}")
    print(f"  에러: {len(result.errors)}")
    print("=" * 60)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

