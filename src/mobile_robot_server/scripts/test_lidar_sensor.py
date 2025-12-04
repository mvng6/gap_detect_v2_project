#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR 센서 유닛 테스트
Woosh Robot의 LiDAR 센서 데이터를 획득하고 검증하는 테스트 코드
"""

import rospy
import asyncio
import sys
import os
import time
from threading import Thread
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# Woosh Robot SDK 임포트
try:
    woosh_robot_py_path = os.path.join(os.path.dirname(__file__), '../../../../woosh_robot_py')
    woosh_robot_py_path = os.path.abspath(woosh_robot_py_path)
    if os.path.exists(woosh_robot_py_path) and woosh_robot_py_path not in sys.path:
        sys.path.insert(0, woosh_robot_py_path)
    
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT
    from woosh.proto.robot.robot_pb2 import ScannerData, RobotInfo
    WOOSH_SDK_AVAILABLE = True
except ImportError as e:
    rospy.logerr(f"Woosh Robot SDK를 임포트할 수 없습니다: {e}")
    WOOSH_SDK_AVAILABLE = False
    WooshRobot = None
    CommuSettings = None
    NO_PRINT = None
    ScannerData = None
    RobotInfo = None


class LidarSensorTest:
    """LiDAR 센서 유닛 테스트 클래스"""
    
    def __init__(self, lidar_id=1):
        """
        Args:
            lidar_id: 테스트할 LiDAR ID (1 또는 2)
        """
        if not WOOSH_SDK_AVAILABLE:
            raise RuntimeError("Woosh Robot SDK를 사용할 수 없습니다.")
        
        self.lidar_id = lidar_id
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', f'lidar_test_{lidar_id}')
        
        self.robot = None
        self.robot_loop = None
        
        # 테스트 결과 저장
        self.received_data_count = 0
        self.first_data_time = None
        self.last_data_time = None
        self.data_samples = []  # 최대 10개 샘플 저장
        self.test_results = {
            'connection': False,
            'data_received': False,
            'data_valid': False,
            'data_count': 0,
            'errors': []
        }
        
        rospy.loginfo(f"=== LiDAR {lidar_id} 센서 테스트 시작 ===")
        rospy.loginfo(f"로봇 IP: {self.robot_ip}:{self.robot_port}")
        rospy.loginfo(f"클라이언트 ID: {self.robot_identity}")
    
    async def connect(self):
        """로봇 연결"""
        rospy.loginfo(f"[테스트] 로봇 연결 시도 중...")
        
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity
        )
        self.robot = WooshRobot(settings)
        
        if not await self.robot.run():
            raise RuntimeError("로봇 연결 실패")
        
        # 연결 검증: 로봇 정보 요청
        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 정보 요청 실패")
        
        self.test_results['connection'] = True
        rospy.loginfo(f"[테스트] ✅ 로봇 연결 성공!")
        rospy.loginfo(f"[테스트] 배터리 잔량: {info.battery.power}%")
    
    def scanner_callback(self, data: ScannerData):
        """LiDAR 데이터 콜백 함수"""
        try:
            current_time = time.time()
            
            # 첫 데이터 시간 기록
            if self.first_data_time is None:
                self.first_data_time = current_time
                rospy.loginfo(f"[테스트] 첫 번째 LiDAR 데이터 수신!")
            
            self.last_data_time = current_time
            self.received_data_count += 1
            
            # 데이터 샘플 저장 (최대 10개)
            if len(self.data_samples) < 10:
                sample = {
                    'timestamp': current_time,
                    'angle_min': data.angle_min,
                    'angle_max': data.angle_max,
                    'angle_increment': data.angle_increment,
                    'range_min': data.range_min,
                    'range_max': data.range_max,
                    'ranges_count': len(data.ranges) if data.ranges else 0,
                    'scan_time': data.scan_time if hasattr(data, 'scan_time') else 0.0,
                    'time_increment': data.time_increment if hasattr(data, 'time_increment') else 0.0
                }
                self.data_samples.append(sample)
            
            # 데이터 검증
            self._validate_data(data)
            
            # 주기적으로 상태 출력 (10개마다)
            if self.received_data_count % 10 == 0:
                rospy.loginfo(f"[테스트] 데이터 수신: {self.received_data_count}개")
                
        except Exception as e:
            error_msg = f"데이터 처리 오류: {e}"
            rospy.logerr(f"[테스트] ❌ {error_msg}")
            self.test_results['errors'].append(error_msg)
    
    def _validate_data(self, data: ScannerData):
        """수신된 데이터의 유효성 검증"""
        try:
            # 기본 검증
            if not hasattr(data, 'ranges') or not data.ranges:
                raise ValueError("ranges 데이터가 없습니다")
            
            if len(data.ranges) == 0:
                raise ValueError("ranges 데이터가 비어있습니다")
            
            # 각도 범위 검증
            if hasattr(data, 'angle_min') and hasattr(data, 'angle_max'):
                if data.angle_min >= data.angle_max:
                    raise ValueError(f"각도 범위 오류: min={data.angle_min}, max={data.angle_max}")
            
            # 범위 값 검증
            if hasattr(data, 'range_min') and hasattr(data, 'range_max'):
                if data.range_min < 0 or data.range_max <= data.range_min:
                    raise ValueError(f"범위 값 오류: min={data.range_min}, max={data.range_max}")
            
            # 데이터 포인트 검증
            valid_ranges = [r for r in data.ranges if data.range_min <= r <= data.range_max]
            if len(valid_ranges) == 0:
                rospy.logwarn(f"[테스트] ⚠️  유효한 범위 데이터가 없습니다")
            
            self.test_results['data_valid'] = True
            
        except Exception as e:
            error_msg = f"데이터 검증 실패: {e}"
            rospy.logwarn(f"[테스트] ⚠️  {error_msg}")
            if error_msg not in self.test_results['errors']:
                self.test_results['errors'].append(error_msg)
    
    async def start_subscription(self, duration=10.0):
        """LiDAR 데이터 구독 시작 및 테스트 실행"""
        rospy.loginfo(f"[테스트] LiDAR 데이터 구독 시작 (지속 시간: {duration}초)")
        
        # 구독 시작
        await self.robot.scanner_data_sub(self.scanner_callback, NO_PRINT)
        
        # 지정된 시간 동안 데이터 수집
        start_time = time.time()
        while (time.time() - start_time) < duration:
            await asyncio.sleep(0.1)
        
        # 결과 업데이트
        self.test_results['data_received'] = self.received_data_count > 0
        self.test_results['data_count'] = self.received_data_count
    
    def print_test_results(self):
        """테스트 결과 출력"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("=== LiDAR 센서 테스트 결과 ===")
        rospy.loginfo("="*60)
        
        # 연결 상태
        status = "✅ 성공" if self.test_results['connection'] else "❌ 실패"
        rospy.loginfo(f"1. 로봇 연결: {status}")
        
        # 데이터 수신 상태
        status = "✅ 성공" if self.test_results['data_received'] else "❌ 실패"
        rospy.loginfo(f"2. 데이터 수신: {status}")
        rospy.loginfo(f"   - 수신된 데이터 개수: {self.test_results['data_count']}개")
        
        if self.first_data_time and self.last_data_time:
            duration = self.last_data_time - self.first_data_time
            if duration > 0:
                rate = self.received_data_count / duration
                rospy.loginfo(f"   - 평균 수신 주기: {1.0/rate:.3f}초 ({rate:.2f} Hz)")
        
        # 데이터 유효성
        status = "✅ 유효" if self.test_results['data_valid'] else "❌ 무효"
        rospy.loginfo(f"3. 데이터 유효성: {status}")
        
        # 샘플 데이터 출력
        if self.data_samples:
            rospy.loginfo("\n--- 샘플 데이터 (첫 번째) ---")
            sample = self.data_samples[0]
            rospy.loginfo(f"각도 범위: {sample['angle_min']:.3f} ~ {sample['angle_max']:.3f} rad")
            rospy.loginfo(f"각도 증분: {sample['angle_increment']:.6f} rad")
            rospy.loginfo(f"거리 범위: {sample['range_min']:.3f} ~ {sample['range_max']:.3f} m")
            rospy.loginfo(f"데이터 포인트 수: {sample['ranges_count']}개")
            rospy.loginfo(f"스캔 시간: {sample['scan_time']:.3f}초")
        
        # 오류 목록
        if self.test_results['errors']:
            rospy.logwarn("\n--- 발견된 오류 ---")
            for i, error in enumerate(self.test_results['errors'], 1):
                rospy.logwarn(f"{i}. {error}")
        else:
            rospy.loginfo("\n--- 오류 없음 ---")
        
        rospy.loginfo("="*60 + "\n")
    
    async def run_test(self, duration=10.0):
        """전체 테스트 실행"""
        try:
            # 1. 연결
            await self.connect()
            
            # 2. 데이터 구독 및 수집
            await self.start_subscription(duration)
            
            # 3. 결과 출력
            self.print_test_results()
            
        except Exception as e:
            rospy.logerr(f"[테스트] ❌ 테스트 실행 중 오류: {e}")
            self.test_results['errors'].append(str(e))
            raise
    
    def run(self):
        """비동기 테스트 실행 (스레드에서)"""
        def run_asyncio():
            self.robot_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.robot_loop)
            
            async def main():
                try:
                    # 테스트 지속 시간 (기본 10초)
                    test_duration = rospy.get_param('~test_duration', 10.0)
                    await self.run_test(duration=test_duration)
                    
                    # 테스트 완료 후 종료
                    rospy.loginfo("[테스트] 테스트 완료. 노드를 종료합니다.")
                    rospy.signal_shutdown("테스트 완료")
                    
                except Exception as e:
                    rospy.logerr(f"[테스트] 테스트 실패: {e}")
                    rospy.signal_shutdown("테스트 실패")
            
            try:
                self.robot_loop.run_until_complete(main())
            except KeyboardInterrupt:
                rospy.loginfo("[테스트] 사용자에 의해 중단됨")
            finally:
                if self.robot_loop:
                    self.robot_loop.close()
        
        thread = Thread(target=run_asyncio, daemon=False)
        thread.start()
        return thread


if __name__ == '__main__':
    rospy.init_node('lidar_sensor_test', anonymous=False)
    
    # LiDAR ID 파라미터
    lidar_id = rospy.get_param('~lidar_id', 1)
    
    # 테스트 실행
    test = LidarSensorTest(lidar_id=lidar_id)
    test_thread = test.run()
    
    # ROS 스핀 (테스트 완료까지 대기)
    rospy.spin()
    
    # 스레드 종료 대기
    test_thread.join(timeout=5.0)

