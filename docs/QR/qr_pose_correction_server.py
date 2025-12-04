#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
================================================================================
QR 코드 위치/자세 보정 서버 (ROS Service Server)
================================================================================

파일명: qr_pose_correction_server.py


【역할】
이 파일은 ROS Service Server로 동작합니다.
로봇 제어 노드(testbed_operation_client_all_with_camera.py)에서 
요청이 오면, QR 코드를 분석하고 보정값을 반환합니다.

【통신 구조】
    로봇 제어 노드 (Client)              이 파일 (Server)
    ┌──────────────────┐               ┌──────────────────┐
    │ request_qr_pose_ │   Request     │                  │
    │ correction()     │ ────────────► │ handle_request() │
    │                  │               │                  │
    │ correction_data  │   Response    │ QR 분석 수행       │
    │ 수신              │ ◄──────────── │                  │
    └──────────────────┘               └──────────────────┘

【실행 방법】
    cd ~/catkin_ws
    source devel/setup.bash
    rosrun testbed_operation qr_pose_correction_server.py

【테스트 방법】
    # 서버 실행 후, 다른 터미널에서:
    rosservice call /qr_pose_correction "{robot_ready: true, current_joint_pos: [90, 0, 90, 0, 90, -90], measurement_point_id: 'test'}"
"""

# ==============================================================================
# 1. Import - ROS 관련 (필수)
# ==============================================================================
import rospy

# 서비스 타입 Import (testbed_operation 패키지에서 자동 생성됨)
# catkin_make 후 사용 가능
from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionResponse

# ==============================================================================
# 2. Import - 팀원이 사용할 라이브러리 (QR 분석용)
# ==============================================================================
import numpy as np
# import cv2                        # OpenCV (QR 검출용)
# from cv_bridge import CvBridge    # ROS 이미지 ↔ OpenCV 변환
# from sensor_msgs.msg import Image # 카메라 이미지 메시지


# ==============================================================================
# 3. QR 분석 클래스 (팀원이 구현할 부분) - 순수 Python
# ==============================================================================
class QRAnalyzer:
    """
    QR 코드 분석 클래스 (순수 Python - ROS와 독립적!)
    
    팀원은 이 클래스의 메서드들을 구현하면 됩니다.
    ROS 없이 독립적으로 테스트 가능합니다.
    """
    
    def __init__(self):
        """
        초기화
        - 카메라 연결
        - QR 검출기 초기화
        - 캘리브레이션 데이터 로드 등
        """
        rospy.loginfo("[QRAnalyzer] 초기화 시작...")
        
        # TODO: 팀원이 구현할 부분
        # ─────────────────────────────────────────────────────────────────────
        # self.camera = cv2.VideoCapture(0)  # 카메라 연결
        # self.qr_detector = cv2.QRCodeDetector()  # QR 검출기
        # self.camera_matrix = np.load('camera_matrix.npy')  # 카메라 내부 파라미터
        # self.dist_coeffs = np.load('dist_coeffs.npy')  # 왜곡 계수
        # ─────────────────────────────────────────────────────────────────────
        
        rospy.loginfo("[QRAnalyzer] 초기화 완료")
    
    def capture_image(self):
        """
        카메라에서 이미지 캡처
        
        Returns:
            numpy.ndarray: 캡처된 이미지 (BGR 형식)
        """
        # TODO: 팀원이 구현할 부분
        # ─────────────────────────────────────────────────────────────────────
        # ret, frame = self.camera.read()
        # if ret:
        #     return frame
        # return None
        # ─────────────────────────────────────────────────────────────────────
        
        # 테스트용 더미 이미지 반환
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def detect_qr(self, image):
        """
        QR 코드 검출
        
        Args:
            image: 입력 이미지 (numpy.ndarray)
        
        Returns:
            dict: {
                'detected': bool,      # 검출 성공 여부
                'corners': ndarray,    # QR 코드 코너 좌표 (4개)
                'data': str            # QR 코드 내용
            }
        """
        # TODO: 팀원이 구현할 부분
        # ─────────────────────────────────────────────────────────────────────
        # data, corners, _ = self.qr_detector.detectAndDecode(image)
        # if corners is not None and len(corners) > 0:
        #     return {'detected': True, 'corners': corners, 'data': data}
        # return {'detected': False, 'corners': None, 'data': None}
        # ─────────────────────────────────────────────────────────────────────
        
        # 테스트용: 항상 검출 성공으로 반환
        return {
            'detected': True,
            'corners': np.array([[100, 100], [200, 100], [200, 200], [100, 200]]),
            'data': 'TEST_QR_CODE'
        }
    
    def calculate_pose(self, corners):
        """
        QR 코드 코너로부터 위치/자세 계산 (PnP 알고리즘)
        
        Args:
            corners: QR 코드 코너 좌표 (4개)
        
        Returns:
            dict: {
                'delta_x': float,      # X축 보정량 [mm]
                'delta_y': float,      # Y축 보정량 [mm]
                'delta_z': float,      # Z축 보정량 [mm]
                'delta_rx': float,     # X축 회전 보정량 [deg]
                'delta_ry': float,     # Y축 회전 보정량 [deg]
                'delta_rz': float,     # Z축 회전 보정량 [deg]
                'rotation_matrix': list,  # 회전 행렬 (9개 요소)
                'confidence': float    # 신뢰도 [0.0 ~ 1.0]
            }
        """
        # TODO: 팀원이 구현할 부분 (PnP 알고리즘)
        # ─────────────────────────────────────────────────────────────────────
        # # QR 코드 실제 크기 (예: 50mm x 50mm)
        # qr_size = 50.0  # mm
        # object_points = np.array([
        #     [-qr_size/2, -qr_size/2, 0],
        #     [qr_size/2, -qr_size/2, 0],
        #     [qr_size/2, qr_size/2, 0],
        #     [-qr_size/2, qr_size/2, 0]
        # ], dtype=np.float32)
        # 
        # # PnP 풀이
        # success, rvec, tvec = cv2.solvePnP(
        #     object_points, corners, 
        #     self.camera_matrix, self.dist_coeffs
        # )
        # 
        # # 회전 벡터 → 회전 행렬
        # rotation_matrix, _ = cv2.Rodrigues(rvec)
        # 
        # # 회전 행렬 → 오일러 각도
        # euler_angles = self.rotation_matrix_to_euler(rotation_matrix)
        # 
        # return {
        #     'delta_x': tvec[0][0],
        #     'delta_y': tvec[1][0],
        #     'delta_z': tvec[2][0],
        #     'delta_rx': euler_angles[0],
        #     'delta_ry': euler_angles[1],
        #     'delta_rz': euler_angles[2],
        #     'rotation_matrix': rotation_matrix.flatten().tolist(),
        #     'confidence': 0.95
        # }
        # ─────────────────────────────────────────────────────────────────────
        
        # 테스트용 더미 값 반환
        return {
            'delta_x': 2.5,       # mm
            'delta_y': -1.3,      # mm
            'delta_z': 0.8,       # mm
            'delta_rx': 0.1,      # deg
            'delta_ry': -0.2,     # deg
            'delta_rz': 0.5,      # deg
            'rotation_matrix': [1, 0, 0, 0, 1, 0, 0, 0, 1],  # 단위 행렬
            'confidence': 0.95
        }
    
    def analyze(self):
        """
        전체 분석 파이프라인 실행
        (이미지 캡처 → QR 검출 → 자세 계산)
        
        Returns:
            dict: 보정값 딕셔너리 또는 None (실패시)
        """
        rospy.loginfo("[QRAnalyzer] 분석 시작...")
        
        # 1. 이미지 캡처
        image = self.capture_image()
        if image is None:
            rospy.logwarn("[QRAnalyzer] 이미지 캡처 실패")
            return None
        
        # 2. QR 코드 검출
        qr_result = self.detect_qr(image)
        if not qr_result['detected']:
            rospy.logwarn("[QRAnalyzer] QR 코드 검출 실패")
            return None
        
        rospy.loginfo("[QRAnalyzer] QR 코드 검출 성공: %s", qr_result['data'])
        
        # 3. 위치/자세 계산
        pose = self.calculate_pose(qr_result['corners'])
        
        rospy.loginfo("[QRAnalyzer] 분석 완료")
        return pose


# ==============================================================================
# 4. ROS Service Server 클래스 (통신 담당 - 핵심!)
# ==============================================================================
class QRPoseCorrectionServer:
    """
    ROS Service Server
    
    로봇 제어 노드의 요청을 받아 QR 분석 결과를 반환합니다.
    서버가 실행되면 요청이 올 때까지 대기합니다.
    """
    
    def __init__(self):
        """서버 초기화"""
        # ──────────────────────────────────────────────────────────────────────
        # ROS 노드 초기화 (필수! 프로그램 시작 시 1번만 실행)
        # 'qr_pose_correction_server' = 노드 이름 (rosnode list에서 확인 가능)
        # ──────────────────────────────────────────────────────────────────────
        rospy.init_node('qr_pose_correction_server')
        
        # ──────────────────────────────────────────────────────────────────────
        # QR 분석기 인스턴스 생성
        # ──────────────────────────────────────────────────────────────────────
        self.analyzer = QRAnalyzer()
        
        # ──────────────────────────────────────────────────────────────────────
        # Service Server 생성 (핵심!)
        # 
        # '/qr_pose_correction' = 서비스 이름 
        #   → 클라이언트에서 이 이름으로 호출합니다
        #   → rosservice list 에서 확인 가능
        #
        # QRPoseCorrection = 서비스 타입 
        #   → srv/QRPoseCorrection.srv 파일에서 정의됨
        #   → Request/Response 형식 지정
        #
        # self.handle_request = 콜백 함수 
        #   → 요청이 오면 이 함수가 자동으로 호출됩니다
        # ──────────────────────────────────────────────────────────────────────
        self.service = rospy.Service(
            '/qr_pose_correction',      # 서비스 이름 (클라이언트와 동일해야 함!)
            QRPoseCorrection,           # 서비스 타입
            self.handle_request         # 요청 처리 함수
        )
        
        # 시작 메시지 출력
        rospy.loginfo("=" * 60)
        rospy.loginfo("  QR Pose Correction Server 시작!")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  서비스 이름: /qr_pose_correction")
        rospy.loginfo("  서비스 타입: testbed_operation/QRPoseCorrection")
        rospy.loginfo("")
        rospy.loginfo("  [대기중] 로봇 제어 노드의 요청을 기다리고 있습니다...")
        rospy.loginfo("=" * 60)
    
    def handle_request(self, req):
        """
        요청 처리 함수 (콜백)
        
        로봇 제어 노드에서 서비스를 호출하면 이 함수가 자동으로 실행됩니다.
        
        Args:
            req (QRPoseCorrectionRequest): 로봇에서 보낸 요청
                - req.robot_ready (bool): 로봇 이동 완료 여부
                - req.current_joint_pos (float64[6]): 현재 관절 각도 [deg]
                - req.measurement_point_id (string): 측정 포인트 ID
        
        Returns:
            QRPoseCorrectionResponse: 보정값 응답
                - success (bool): 성공 여부
                - message (string): 상태 메시지
                - delta_x, delta_y, delta_z (float64): 위치 보정값 [mm]
                - delta_rx, delta_ry, delta_rz (float64): 자세 보정값 [deg]
                - rotation_matrix (float64[9]): 회전 행렬
                - qr_confidence (float64): 신뢰도
                - qr_position (float64[3]): QR 위치
        """
        rospy.loginfo("")
        rospy.loginfo("-" * 60)
        rospy.loginfo("  [요청 수신] 로봇 제어 노드로부터 요청이 도착했습니다!")
        rospy.loginfo("-" * 60)
        rospy.loginfo("  측정 포인트 ID: %s", req.measurement_point_id)
        rospy.loginfo("  로봇 준비 상태: %s", "준비됨" if req.robot_ready else "준비 안됨")
        rospy.loginfo("  현재 관절 각도: %s", list(req.current_joint_pos))
        
        # ──────────────────────────────────────────────────────────────────────
        # 응답 객체 생성 (반드시 QRPoseCorrectionResponse 타입!)
        # ──────────────────────────────────────────────────────────────────────
        response = QRPoseCorrectionResponse()
        
        # ──────────────────────────────────────────────────────────────────────
        # 로봇 준비 상태 확인
        # ──────────────────────────────────────────────────────────────────────
        if not req.robot_ready:
            response.success = False
            response.message = "로봇이 준비되지 않았습니다"
            response.qr_confidence = 0.0
            rospy.logwarn("  [거부] 로봇 준비 안됨 - 요청 거부")
            return response
        
        # ──────────────────────────────────────────────────────────────────────
        # QR 코드 분석 수행 (핵심 로직!)
        # ──────────────────────────────────────────────────────────────────────
        rospy.loginfo("  [분석중] QR 코드 분석을 시작합니다...")
        result = self.analyzer.analyze()
        
        # ──────────────────────────────────────────────────────────────────────
        # 분석 실패 시 응답
        # ──────────────────────────────────────────────────────────────────────
        if result is None:
            response.success = False
            response.message = "QR 코드를 찾을 수 없습니다"
            response.qr_confidence = 0.0
            rospy.logwarn("  [실패] QR 코드 검출 실패")
            return response
        
        # ──────────────────────────────────────────────────────────────────────
        # 분석 성공 시 응답 구성
        # ──────────────────────────────────────────────────────────────────────
        response.success = True
        response.message = "보정값 계산 완료"
        
        # 위치 보정값 (mm)
        response.delta_x = result['delta_x']
        response.delta_y = result['delta_y']
        response.delta_z = result['delta_z']
        
        # 자세 보정값 (deg)
        response.delta_rx = result['delta_rx']
        response.delta_ry = result['delta_ry']
        response.delta_rz = result['delta_rz']
        
        # 회전 행렬 (9개 요소)
        response.rotation_matrix = result['rotation_matrix']
        
        # QR 코드 정보
        response.qr_confidence = result['confidence']
        response.qr_position = [result['delta_x'], result['delta_y'], result['delta_z']]
        
        # 결과 로그 출력
        rospy.loginfo("-" * 60)
        rospy.loginfo("  [성공] 분석 완료! 응답을 전송합니다.")
        rospy.loginfo("-" * 60)
        rospy.loginfo("  위치 보정값:")
        rospy.loginfo("    delta_x: %.3f mm", response.delta_x)
        rospy.loginfo("    delta_y: %.3f mm", response.delta_y)
        rospy.loginfo("    delta_z: %.3f mm", response.delta_z)
        rospy.loginfo("  자세 보정값:")
        rospy.loginfo("    delta_rx: %.3f deg", response.delta_rx)
        rospy.loginfo("    delta_ry: %.3f deg", response.delta_ry)
        rospy.loginfo("    delta_rz: %.3f deg", response.delta_rz)
        rospy.loginfo("  신뢰도: %.2f", response.qr_confidence)
        rospy.loginfo("-" * 60)
        rospy.loginfo("")
        rospy.loginfo("  [대기중] 다음 요청을 기다리고 있습니다...")
        
        return response
    
    def run(self):
        """
        서버 실행 (무한 대기)
        
        이 함수가 호출되면 프로그램이 종료될 때까지
        클라이언트의 요청을 기다립니다.
        """
        # rospy.spin() = 프로그램 종료(Ctrl+C)까지 계속 실행
        rospy.spin()


# ==============================================================================
# 5. 메인 실행부
# ==============================================================================
if __name__ == '__main__':
    try:
        # 서버 인스턴스 생성 및 실행
        server = QRPoseCorrectionServer()
        server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("QR Pose Correction Server 종료")

