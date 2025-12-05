#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Motion Corrector Node

LiDAR 센서를 활용하여 로봇의 실제 이동 거리를 측정하고 보정하는 노드입니다.

로봇 LiDAR 배치:
    - 왼쪽 앞 LiDAR (L1): 전진 시 전방 거리 측정
    - 오른쪽 뒤 LiDAR (L2): 후진 시 후방 거리 측정

보정 원리:
    - 이동 전 기준 거리(d₀) 측정
    - 이동 중/후 현재 거리(d₁) 측정
    - 실제 이동 거리 = d₀ - d₁

Author: KATECH Robotics Team
Date: 2025-12-04
"""

import rospy
import numpy as np
from threading import Lock
from typing import Optional, Tuple, List
from enum import Enum
from dataclasses import dataclass

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse


class MoveDirection(Enum):
    """이동 방향"""
    FORWARD = 1   # 전진
    BACKWARD = -1 # 후진
    STOPPED = 0   # 정지


@dataclass
class LidarConfig:
    """LiDAR 설정"""
    # 전방 LiDAR (왼쪽 앞) - 전진 시 사용
    front_angle_min: float = -0.26  # -15도 (rad)
    front_angle_max: float = 0.26   # +15도 (rad)
    
    # 후방 LiDAR (오른쪽 뒤) - 후진 시 사용
    # 각도는 LiDAR 배치에 따라 조정 필요
    rear_angle_min: float = 2.88    # 165도 (rad)
    rear_angle_max: float = 3.40    # 195도 (rad) - π ± 15도
    
    # 필터링 설정
    min_valid_points: int = 3       # 최소 유효 포인트 수
    outlier_threshold: float = 0.5  # 이상치 필터링 임계값 (m)


class LidarMotionCorrector:
    """
    LiDAR 기반 이동 거리 보정 클래스
    
    단방향 거리 모니터링 방식을 사용하여 로봇의 실제 이동 거리를 측정합니다.
    전진 시 전방 LiDAR, 후진 시 후방 LiDAR를 사용합니다.
    """
    
    def __init__(self):
        """초기화"""
        # === ROS 파라미터 로드 ===
        self.scan_topic = rospy.get_param('~scan_topic', '/mobile_lidar/scan')
        
        # LiDAR 각도 설정 (라디안)
        self.front_angle_min = rospy.get_param('~front_angle_min', -0.26)
        self.front_angle_max = rospy.get_param('~front_angle_max', 0.26)
        self.rear_angle_min = rospy.get_param('~rear_angle_min', 2.88)
        self.rear_angle_max = rospy.get_param('~rear_angle_max', 3.40)
        
        # 필터링 설정
        self.min_valid_points = rospy.get_param('~min_valid_points', 3)
        self.outlier_threshold = rospy.get_param('~outlier_threshold', 0.5)
        
        # === 상태 변수 ===
        self.current_scan: Optional[LaserScan] = None
        self.initial_front_distance: Optional[float] = None
        self.initial_rear_distance: Optional[float] = None
        self.current_front_distance: Optional[float] = None
        self.current_rear_distance: Optional[float] = None
        
        self.move_direction = MoveDirection.STOPPED
        self.is_measuring = False
        self.measurement_count = 0
        
        self.lock = Lock()
        
        # === ROS Subscriber/Publisher ===
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_callback,
            queue_size=1
        )
        
        # 보정된 거리 발행
        self.corrected_distance_pub = rospy.Publisher(
            '/mobile_lidar_control/corrected_distance',
            Float32,
            queue_size=10
        )
        
        # 현재 측정 거리 발행 (디버깅용)
        self.front_distance_pub = rospy.Publisher(
            '/mobile_lidar_control/front_distance',
            Float32,
            queue_size=10
        )
        self.rear_distance_pub = rospy.Publisher(
            '/mobile_lidar_control/rear_distance',
            Float32,
            queue_size=10
        )
        
        # === ROS 서비스 ===
        self.start_measurement_srv = rospy.Service(
            '/mobile_lidar_control/start_measurement',
            Trigger,
            self.handle_start_measurement
        )
        
        self.stop_measurement_srv = rospy.Service(
            '/mobile_lidar_control/stop_measurement',
            Trigger,
            self.handle_stop_measurement
        )
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("LiDAR Motion Corrector 초기화")
        rospy.loginfo(f"  Scan Topic: {self.scan_topic}")
        rospy.loginfo(f"  Front Angle: [{np.degrees(self.front_angle_min):.1f}°, {np.degrees(self.front_angle_max):.1f}°]")
        rospy.loginfo(f"  Rear Angle: [{np.degrees(self.rear_angle_min):.1f}°, {np.degrees(self.rear_angle_max):.1f}°]")
        rospy.loginfo("=" * 60)
    
    # =========================================================================
    # LiDAR 데이터 처리
    # =========================================================================
    
    def scan_callback(self, msg: LaserScan):
        """LiDAR 스캔 데이터 콜백"""
        with self.lock:
            self.current_scan = msg
            
            # 전방/후방 거리 계산
            self.current_front_distance = self._get_distance_in_range(
                msg, self.front_angle_min, self.front_angle_max
            )
            self.current_rear_distance = self._get_distance_in_range(
                msg, self.rear_angle_min, self.rear_angle_max
            )
            
            # 디버깅용 토픽 발행
            if self.current_front_distance is not None:
                self.front_distance_pub.publish(Float32(self.current_front_distance))
            if self.current_rear_distance is not None:
                self.rear_distance_pub.publish(Float32(self.current_rear_distance))
            
            # 측정 중이면 보정 거리 계산 및 발행
            if self.is_measuring:
                corrected = self.get_traveled_distance()
                if corrected is not None:
                    self.corrected_distance_pub.publish(Float32(corrected))
    
    def _get_distance_in_range(self, scan: LaserScan, 
                                angle_min: float, angle_max: float) -> Optional[float]:
        """
        지정된 각도 범위 내의 평균 거리를 계산합니다.
        
        Args:
            scan: LaserScan 메시지
            angle_min: 최소 각도 (rad)
            angle_max: 최대 각도 (rad)
            
        Returns:
            평균 거리 (m) 또는 None (유효 데이터 없음)
        """
        if scan is None or len(scan.ranges) == 0:
            return None
        
        # 각도 인덱스 계산
        idx_min = self._angle_to_index(scan, angle_min)
        idx_max = self._angle_to_index(scan, angle_max)
        
        if idx_min is None or idx_max is None:
            return None
        
        # 인덱스 정렬
        if idx_min > idx_max:
            idx_min, idx_max = idx_max, idx_min
        
        # 범위 내 거리값 추출
        ranges_in_range = scan.ranges[idx_min:idx_max+1]
        
        # 유효한 거리값만 필터링
        valid_ranges = [
            r for r in ranges_in_range
            if (not np.isinf(r) and 
                not np.isnan(r) and 
                scan.range_min < r < scan.range_max)
        ]
        
        if len(valid_ranges) < self.min_valid_points:
            return None
        
        # 이상치 제거 (중앙값 기반)
        median = np.median(valid_ranges)
        filtered_ranges = [
            r for r in valid_ranges
            if abs(r - median) < self.outlier_threshold
        ]
        
        if len(filtered_ranges) < self.min_valid_points:
            return median  # 필터링 후 데이터 부족 시 중앙값 반환
        
        return np.mean(filtered_ranges)
    
    def _angle_to_index(self, scan: LaserScan, angle: float) -> Optional[int]:
        """각도를 스캔 인덱스로 변환"""
        if scan.angle_increment == 0:
            return None
        
        # 각도 정규화 (-π ~ π)
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        
        # 스캔 범위 체크
        if angle < scan.angle_min or angle > scan.angle_max:
            # 범위 밖이면 가장 가까운 인덱스 반환
            if angle < scan.angle_min:
                return 0
            else:
                return len(scan.ranges) - 1
        
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        return max(0, min(idx, len(scan.ranges) - 1))
    
    # =========================================================================
    # 측정 제어
    # =========================================================================
    
    def start_measurement(self, direction: MoveDirection = MoveDirection.FORWARD) -> bool:
        """
        측정을 시작합니다. 현재 거리를 기준점으로 저장합니다.
        
        Args:
            direction: 이동 방향 (전진/후진)
            
        Returns:
            성공 여부
        """
        with self.lock:
            if self.current_scan is None:
                rospy.logwarn("LiDAR 데이터가 없습니다.")
                return False
            
            self.move_direction = direction
            self.measurement_count = 0
            
            if direction == MoveDirection.FORWARD:
                # 전진: 전방 거리 기준점 설정
                self.initial_front_distance = self.current_front_distance
                if self.initial_front_distance is None:
                    rospy.logwarn("전방 거리를 측정할 수 없습니다.")
                    return False
                rospy.loginfo(f"[측정 시작] 전진 모드 - 초기 전방 거리: {self.initial_front_distance:.3f}m")
                
            elif direction == MoveDirection.BACKWARD:
                # 후진: 후방 거리 기준점 설정
                self.initial_rear_distance = self.current_rear_distance
                if self.initial_rear_distance is None:
                    rospy.logwarn("후방 거리를 측정할 수 없습니다.")
                    return False
                rospy.loginfo(f"[측정 시작] 후진 모드 - 초기 후방 거리: {self.initial_rear_distance:.3f}m")
            
            self.is_measuring = True
            return True
    
    def stop_measurement(self) -> Tuple[bool, float, str]:
        """
        측정을 종료하고 최종 이동 거리를 반환합니다.
        
        Returns:
            (성공 여부, 이동 거리, 메시지)
        """
        with self.lock:
            if not self.is_measuring:
                return False, 0.0, "측정 중이 아닙니다."
            
            traveled = self.get_traveled_distance()
            
            self.is_measuring = False
            self.move_direction = MoveDirection.STOPPED
            self.initial_front_distance = None
            self.initial_rear_distance = None
            
            if traveled is None:
                return False, 0.0, "거리 계산 실패"
            
            rospy.loginfo(f"[측정 종료] 이동 거리: {traveled:.3f}m")
            return True, traveled, f"측정 완료: {traveled:.3f}m"
    
    def get_traveled_distance(self) -> Optional[float]:
        """
        현재까지 이동한 거리를 계산합니다.
        
        Returns:
            이동 거리 (m) 또는 None
        """
        if self.move_direction == MoveDirection.FORWARD:
            if self.initial_front_distance is None or self.current_front_distance is None:
                return None
            # 전진: 전방 거리 감소량 = 이동 거리
            return self.initial_front_distance - self.current_front_distance
            
        elif self.move_direction == MoveDirection.BACKWARD:
            if self.initial_rear_distance is None or self.current_rear_distance is None:
                return None
            # 후진: 후방 거리 감소량 = 이동 거리 (음수로 반환)
            return -(self.initial_rear_distance - self.current_rear_distance)
        
        return None
    
    def get_current_distances(self) -> Tuple[Optional[float], Optional[float]]:
        """현재 전방/후방 거리 반환"""
        with self.lock:
            return self.current_front_distance, self.current_rear_distance
    
    # =========================================================================
    # ROS 서비스 핸들러
    # =========================================================================
    
    def handle_start_measurement(self, req) -> TriggerResponse:
        """측정 시작 서비스 핸들러 (기본: 전진)"""
        success = self.start_measurement(MoveDirection.FORWARD)
        if success:
            return TriggerResponse(True, f"측정 시작 - 초기 거리: {self.initial_front_distance:.3f}m")
        else:
            return TriggerResponse(False, "측정 시작 실패 - LiDAR 데이터 없음")
    
    def handle_stop_measurement(self, req) -> TriggerResponse:
        """측정 종료 서비스 핸들러"""
        success, distance, msg = self.stop_measurement()
        return TriggerResponse(success, msg)
    
    # =========================================================================
    # 유틸리티
    # =========================================================================
    
    def get_status(self) -> dict:
        """현재 상태 반환"""
        with self.lock:
            return {
                'is_measuring': self.is_measuring,
                'move_direction': self.move_direction.name,
                'current_front_distance': self.current_front_distance,
                'current_rear_distance': self.current_rear_distance,
                'initial_front_distance': self.initial_front_distance,
                'initial_rear_distance': self.initial_rear_distance,
                'traveled_distance': self.get_traveled_distance() if self.is_measuring else None
            }


def main():
    """메인 함수"""
    rospy.init_node('lidar_motion_corrector', anonymous=False)
    
    corrector = LidarMotionCorrector()
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("LiDAR Motion Corrector 실행 중")
    rospy.loginfo("서비스:")
    rospy.loginfo("  - /mobile_lidar_control/start_measurement")
    rospy.loginfo("  - /mobile_lidar_control/stop_measurement")
    rospy.loginfo("토픽:")
    rospy.loginfo("  - /mobile_lidar_control/corrected_distance")
    rospy.loginfo("  - /mobile_lidar_control/front_distance")
    rospy.loginfo("  - /mobile_lidar_control/rear_distance")
    rospy.loginfo("=" * 60)
    
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")

