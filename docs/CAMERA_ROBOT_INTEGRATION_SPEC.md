# 📋 갭 단차 측정 시스템 - 카메라-로봇 통합 작업 지시서

## 📌 문서 정보
| 항목 | 내용 |
|------|------|
| **프로젝트명** | 갭 단차 측정 시스템 (Gap Detection System) |
| **문서 버전** | v1.0 |
| **작성일** | 2025-12-02 |
| **작성자** | KATECH 스마트제조기술연구센터 |
| **관련 파일** | `testbed_operation_client_all_with_camera.py` |

---

## 1. 📖 시스템 개요

### 1.1 목적
협동로봇(Doosan A0912)의 엔드이펙터에 부착된 3D 카메라를 활용하여 QR 코드 기반 위치/자세 보정을 수행하고, 정밀한 갭 단차 측정을 실현한다.

### 1.2 역할 분담

| 역할 | 담당자 | 담당 노드 | 주요 업무 |
|------|--------|-----------|-----------|
| **로봇 제어** | User | `testbed_operation_client_all_with_camera.py` | 로봇 이동, 보정값 적용 |
| **카메라 보정** | 팀원 | `qr_pose_correction_node.py` (신규) | QR 인식, 위치/자세 계산 |

### 1.3 워크플로우

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        갭 단차 측정 워크플로우                                │
└─────────────────────────────────────────────────────────────────────────────┘

[로봇 제어 노드]                              [카메라 보정 노드]
      │                                              │
      │  1. QR 코드 위치로 로봇 이동                   │
      │     dsr_move_joint(qr_position)              │
      │                                              │
      ▼                                              │
 ┌─────────┐                                         │
 │이동 완료  │                                         │
 └────┬────┘                                         │
      │                                              │
      │  2. 보정 요청 Service 호출 ─────────────────► │
      │     /qr_pose_correction                      │
      │     Request: { robot_ready: true }           │
      │                                              ▼
      │                                        ┌──────────┐
      │                                        │QR 코드   │
      │                                        │촬영 및   │
      │                                        │분석      │
      │                                        └────┬─────┘
      │                                              │
      │  3. 보정값 응답 수신 ◄─────────────────────── │
      │     Response: { position, rotation_matrix }  │
      │                                              │
      ▼                                              │
 ┌─────────┐                                         │
 │보정값     │                                         │
 │적용      │                                         │
 └────┬────┘                                         │
      │                                              │
      │  4. 보정된 위치로 로봇 이동                    │
      │     dsr_move_joint(corrected_position)       │
      │                                              │
      ▼                                              │
 ┌─────────┐                                         │
 │측정 수행  │                                         │
 └─────────┘                                         │
```

---

## 2. 🔗 통신 방식 선정

### 2.1 통신 방식 비교 분석

| 방식 | 장점 | 단점 | 적합성 |
|------|------|------|--------|
| **Topic** | 비동기, 다대다 통신 | 타이밍 관리 복잡, 응답 보장 없음 | ❌ 부적합 |
| **Service** | 동기적, 요청-응답 보장 | 블로킹, 장시간 작업시 timeout | ✅ **최적** |
| **Action** | 피드백 제공, 취소 가능 | 구현 복잡, 오버엔지니어링 | ⚠️ 과도함 |

### 2.2 선정 결과: **Service 방식**

**선정 이유:**
1. **동기적 통신**: 로봇이 이동 완료 후 보정값을 받을 때까지 대기해야 함
2. **응답 보장**: 보정값을 반드시 수신해야 다음 단계 진행 가능
3. **구현 간소화**: 기존 코드베이스와 일관된 패턴 유지
4. **타이밍 제어**: 자연스러운 순차 실행 보장

---

## 3. 📡 통신 구조 상세

### 3.1 노드 구성도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ROS 노드 구성도                                 │
└─────────────────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────────────────┐
                    │         ROS Master              │
                    └─────────────────────────────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         │                         │                         │
         ▼                         ▼                         ▼
┌─────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│  DSR Launcher   │    │   Robot Control     │    │  QR Pose Correction │
│  (두산로봇연결) │    │   (로봇 제어 노드)   │    │  (카메라 보정 노드)  │
│                 │    │                     │    │                     │
│ Node:           │    │ Node:               │    │ Node:               │
│ /dsr01a0912     │    │ /robot_controller   │    │ /qr_pose_corrector  │
│                 │    │                     │    │                     │
│ Services:       │    │ Service Client:     │    │ Service Server:     │
│ /move_joint     │◄───│ /move_joint         │    │ /qr_pose_correction │
│                 │    │ /qr_pose_correction │───►│                     │
│                 │    │                     │    │ Subscribers:        │
│                 │    │                     │    │ /camera/image_raw   │
│                 │    │                     │    │ /camera/depth       │
└─────────────────┘    └─────────────────────┘    └─────────────────────┘
        ▲                                                   │
        │                                                   │
        │              ┌─────────────────────┐              │
        │              │    3D Camera Node   │              │
        └──────────────│  (카메라 드라이버)   │◄─────────────┘
                       │                     │
                       │ Publishers:         │
                       │ /camera/image_raw   │
                       │ /camera/depth       │
                       │ /camera/pointcloud  │
                       └─────────────────────┘
```

### 3.2 서비스 정의

#### 3.2.1 서비스 파일: `QRPoseCorrection.srv`

```
# 파일 위치: testbed_operation/srv/QRPoseCorrection.srv
# 설명: QR 코드 기반 위치/자세 보정 서비스

#------------------------------------------------------------------------------
# Request (로봇 제어 노드 → 카메라 보정 노드)
#------------------------------------------------------------------------------
bool robot_ready                    # 로봇 이동 완료 플래그
float64[6] current_joint_pos        # 현재 로봇 관절 각도 [deg]
string measurement_point_id         # 측정 포인트 ID (예: "A-point1", "B-point2")

---

#------------------------------------------------------------------------------
# Response (카메라 보정 노드 → 로봇 제어 노드)
#------------------------------------------------------------------------------
bool success                        # 보정 성공 여부
string message                      # 상태 메시지

# 위치 보정값 (카메라 좌표계 기준)
float64 delta_x                     # X축 보정량 [mm]
float64 delta_y                     # Y축 보정량 [mm]
float64 delta_z                     # Z축 보정량 [mm]

# 자세 보정값 (Rotation Matrix → Euler Angles로 변환)
float64 delta_rx                    # X축 회전 보정량 [deg]
float64 delta_ry                    # Y축 회전 보정량 [deg]
float64 delta_rz                    # Z축 회전 보정량 [deg]

# 원본 Rotation Matrix (3x3 → 9개 요소, row-major order)
float64[9] rotation_matrix          # [r11, r12, r13, r21, r22, r23, r31, r32, r33]

# QR 코드 인식 정보
float64 qr_confidence               # QR 코드 인식 신뢰도 [0.0 ~ 1.0]
float64[3] qr_position              # QR 코드 위치 (카메라 좌표계) [mm]
```

### 3.3 메시지 흐름 시퀀스

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           메시지 흐름 시퀀스 다이어그램                        │
└─────────────────────────────────────────────────────────────────────────────┘

  Robot Controller               QR Pose Corrector              3D Camera
        │                               │                           │
        │   1. 로봇 이동 완료            │                           │
        │                               │                           │
        │   ──── Service Request ─────► │                           │
        │   {                           │                           │
        │     robot_ready: true,        │                           │
        │     current_joint_pos: [...], │                           │
        │     measurement_point_id: "A1"│                           │
        │   }                           │                           │
        │                               │                           │
        │                               │   2. 이미지 요청           │
        │                               │   ───────────────────────►│
        │                               │                           │
        │                               │   3. 이미지 수신           │
        │                               │   ◄───────────────────────│
        │                               │   /camera/image_raw       │
        │                               │   /camera/depth           │
        │                               │                           │
        │                               │   4. QR 코드 검출 및       │
        │                               │      위치/자세 계산        │
        │                               │                           │
        │   ◄─── Service Response ───── │                           │
        │   {                           │                           │
        │     success: true,            │                           │
        │     delta_x: 2.5,             │                           │
        │     delta_y: -1.3,            │                           │
        │     delta_z: 0.8,             │                           │
        │     delta_rx: 0.5,            │                           │
        │     delta_ry: -0.2,           │                           │
        │     delta_rz: 1.1,            │                           │
        │     rotation_matrix: [...],   │                           │
        │     qr_confidence: 0.95       │                           │
        │   }                           │                           │
        │                               │                           │
        │   5. 보정값 적용 및 로봇 이동   │                           │
        │                               │                           │
        ▼                               ▼                           ▼
```

---

## 4. 🛠️ 구현 상세

### 4.1 로봇 제어 노드 수정사항 (User 담당)

#### 4.1.1 파일: `testbed_operation_client_all_with_camera.py`

**추가할 Import:**
```python
from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionRequest
from geometry_msgs.msg import Pose
import numpy as np
```

**추가할 함수:**

```python
def request_qr_pose_correction(current_joint_pos, measurement_point_id="default"):
    """
    QR 코드 기반 위치/자세 보정을 요청하는 함수
    
    Args:
        current_joint_pos (list): 현재 로봇 관절 각도 [6개, deg]
        measurement_point_id (str): 측정 포인트 식별자
    
    Returns:
        dict: 보정값 딕셔너리 또는 None (실패시)
            {
                'delta_x': float,      # mm
                'delta_y': float,      # mm
                'delta_z': float,      # mm
                'delta_rx': float,     # deg
                'delta_ry': float,     # deg
                'delta_rz': float,     # deg
                'rotation_matrix': list,  # 9개 요소
                'qr_confidence': float
            }
    """
    service_name = '/qr_pose_correction'
    rospy.loginfo("Waiting for QR pose correction service: %s", service_name)
    
    try:
        rospy.wait_for_service(service_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("QR pose correction service not available (timeout)")
        return None
    
    try:
        qr_correction = rospy.ServiceProxy(service_name, QRPoseCorrection)
        req = QRPoseCorrectionRequest()
        req.robot_ready = True
        req.current_joint_pos = current_joint_pos
        req.measurement_point_id = measurement_point_id
        
        rospy.loginfo("Requesting QR pose correction for point: %s", measurement_point_id)
        resp = qr_correction(req)
        
        if resp.success:
            rospy.loginfo("QR pose correction succeeded (confidence: %.2f)", resp.qr_confidence)
            return {
                'delta_x': resp.delta_x,
                'delta_y': resp.delta_y,
                'delta_z': resp.delta_z,
                'delta_rx': resp.delta_rx,
                'delta_ry': resp.delta_ry,
                'delta_rz': resp.delta_rz,
                'rotation_matrix': list(resp.rotation_matrix),
                'qr_confidence': resp.qr_confidence
            }
        else:
            rospy.logwarn("QR pose correction failed: %s", resp.message)
            return None
            
    except rospy.ServiceException as e:
        rospy.logerr("QR pose correction service call failed: %s", e)
        return None


def apply_pose_correction(current_joint_pos, correction_data, vel=30.0, acc=30.0):
    """
    보정값을 적용하여 로봇 자세를 보정하는 함수
    
    Args:
        current_joint_pos (list): 현재 관절 각도 [6개, deg]
        correction_data (dict): 보정값 딕셔너리
        vel (float): 이동 속도 [deg/s]
        acc (float): 가속도 [deg/s^2]
    
    Returns:
        bool: 보정 성공 여부
    
    Note:
        현재는 간단한 관절 각도 보정만 수행.
        실제 구현 시 Inverse Kinematics를 사용하여
        카테시안 보정값을 관절 각도로 변환해야 함.
    """
    # TODO: IK를 사용한 정밀 보정 구현
    # 현재는 엔드이펙터 기준 보정값을 관절 각도로 근사 변환
    
    # 간단한 보정 적용 (6번 관절에 회전 보정)
    corrected_joint_pos = list(current_joint_pos)
    
    # Z축 회전 보정을 6번 관절에 적용 (근사)
    corrected_joint_pos[5] += correction_data['delta_rz']
    
    rospy.loginfo("Applying pose correction: delta_rz = %.3f deg", correction_data['delta_rz'])
    rospy.loginfo("Corrected joint position: %s", corrected_joint_pos)
    
    return dsr_move_joint(pos=corrected_joint_pos, vel=vel, acc=acc)
```

**수정할 main() 함수:**

```python
def main():
    rospy.init_node('integrated_robot_client', anonymous=True)

    # DSR 로봇 속도, 가속도 설정
    dsr_vel = 30.0
    dsr_acc = 30.0

    # 1. 홈 포지션으로 이동
    rospy.loginfo("=== Step 1: Moving to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to home position.")
        return

    rospy.sleep(1.0)  # 안정화 대기

    # 2. QR 코드 측정 위치로 이동
    rospy.loginfo("=== Step 2: Moving to QR measurement position ===")
    qr_measurement_pos = [49.54, 31.27, 87.67, 0.0, 61.06, -130.46]  # A-point1
    if not dsr_move_joint(pos=qr_measurement_pos, vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to QR measurement position.")
        return

    rospy.sleep(0.5)  # 안정화 대기

    # 3. QR 코드 기반 위치/자세 보정 요청
    rospy.loginfo("=== Step 3: Requesting QR pose correction ===")
    correction_data = request_qr_pose_correction(
        current_joint_pos=qr_measurement_pos,
        measurement_point_id="A-point1"
    )

    if correction_data is None:
        rospy.logerr("QR pose correction failed. Aborting.")
        return

    # 4. 보정 신뢰도 확인
    if correction_data['qr_confidence'] < 0.8:
        rospy.logwarn("QR confidence too low (%.2f). Skipping correction.",
                      correction_data['qr_confidence'])
    else:
        # 5. 보정값 적용
        rospy.loginfo("=== Step 4: Applying pose correction ===")
        if not apply_pose_correction(qr_measurement_pos, correction_data, 
                                     vel=dsr_vel, acc=dsr_acc):
            rospy.logerr("Failed to apply pose correction.")
            return

    rospy.sleep(0.5)  # 안정화 대기

    # 6. 갭 단차 측정 수행 (추후 구현)
    rospy.loginfo("=== Step 5: Gap measurement (TBD) ===")

    # 7. 홈 포지션으로 복귀
    rospy.loginfo("=== Step 6: Returning to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to return to home position.")
        return

    rospy.loginfo("=== All operations completed successfully! ===")
```

### 4.2 카메라 보정 노드 구현 (팀원 담당)

#### 4.2.1 파일: `qr_pose_correction_node.py` (신규 생성)

**노드 구조:**
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
QR 코드 기반 위치/자세 보정 노드

이 노드는 로봇 제어 노드로부터 보정 요청을 받아
3D 카메라로 QR 코드를 인식하고 위치/자세 보정값을 계산하여 반환합니다.
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionResponse


class QRPoseCorrectionNode:
    def __init__(self):
        rospy.init_node('qr_pose_corrector', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera image storage
        self.current_image = None
        self.current_depth = None
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(
            '/camera/depth', Image, self.depth_callback)
        
        # Service Server
        self.correction_service = rospy.Service(
            '/qr_pose_correction',
            QRPoseCorrection,
            self.handle_correction_request
        )
        
        rospy.loginfo("QR Pose Correction Node initialized.")
        rospy.loginfo("Waiting for correction requests on /qr_pose_correction")
    
    def image_callback(self, msg):
        """RGB 이미지 콜백"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", e)
    
    def depth_callback(self, msg):
        """Depth 이미지 콜백"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            rospy.logerr("Depth conversion failed: %s", e)
    
    def detect_qr_code(self, image):
        """
        QR 코드 검출 및 위치 추출
        
        Returns:
            tuple: (detected, corners, data) or (False, None, None)
        """
        # TODO: 실제 QR 코드 검출 로직 구현
        # OpenCV의 QRCodeDetector 또는 pyzbar 라이브러리 사용
        detector = cv2.QRCodeDetector()
        data, corners, _ = detector.detectAndDecode(image)
        
        if corners is not None and len(corners) > 0:
            return True, corners, data
        return False, None, None
    
    def calculate_pose_from_qr(self, corners, depth_image):
        """
        QR 코드 코너로부터 위치/자세 계산
        
        Returns:
            dict: 위치 및 회전 정보
        """
        # TODO: 실제 PnP 알고리즘을 사용한 자세 추정 구현
        # cv2.solvePnP() 사용
        
        # 임시 반환값 (실제 구현 필요)
        return {
            'position': [0.0, 0.0, 0.0],
            'rotation_matrix': np.eye(3).flatten().tolist(),
            'euler_angles': [0.0, 0.0, 0.0]
        }
    
    def handle_correction_request(self, req):
        """
        보정 요청 처리 핸들러
        
        Args:
            req: QRPoseCorrectionRequest
        
        Returns:
            QRPoseCorrectionResponse
        """
        resp = QRPoseCorrectionResponse()
        
        rospy.loginfo("Received correction request for point: %s", 
                      req.measurement_point_id)
        rospy.loginfo("Robot ready: %s", req.robot_ready)
        rospy.loginfo("Current joint position: %s", list(req.current_joint_pos))
        
        # 이미지 확인
        if self.current_image is None:
            resp.success = False
            resp.message = "No camera image available"
            return resp
        
        # QR 코드 검출
        detected, corners, qr_data = self.detect_qr_code(self.current_image)
        
        if not detected:
            resp.success = False
            resp.message = "QR code not detected"
            resp.qr_confidence = 0.0
            return resp
        
        # 위치/자세 계산
        pose_info = self.calculate_pose_from_qr(corners, self.current_depth)
        
        # 응답 구성
        resp.success = True
        resp.message = "Pose correction calculated successfully"
        
        # 위치 보정값 (예시 - 실제 계산 필요)
        resp.delta_x = pose_info['position'][0]
        resp.delta_y = pose_info['position'][1]
        resp.delta_z = pose_info['position'][2]
        
        # 자세 보정값
        resp.delta_rx = pose_info['euler_angles'][0]
        resp.delta_ry = pose_info['euler_angles'][1]
        resp.delta_rz = pose_info['euler_angles'][2]
        
        # Rotation Matrix
        resp.rotation_matrix = pose_info['rotation_matrix']
        
        # QR 정보
        resp.qr_confidence = 0.95  # 실제 신뢰도 계산 필요
        resp.qr_position = pose_info['position']
        
        rospy.loginfo("Correction calculated: dx=%.3f, dy=%.3f, dz=%.3f",
                      resp.delta_x, resp.delta_y, resp.delta_z)
        
        return resp
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = QRPoseCorrectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
```

---

## 5. 📁 파일 구조

### 5.1 수정/생성 파일 목록

```
testbed_operation/
├── CMakeLists.txt          # 수정: 새 srv 파일 추가
├── package.xml             # 확인: 의존성 추가 필요시
├── srv/
│   ├── MoveJoint.srv       # 기존
│   ├── MobilePositionTwist.srv  # 기존
│   └── QRPoseCorrection.srv     # 신규 생성 ★
├── scripts/
│   ├── testbed_operation_client_all.py           # 기존
│   ├── testbed_operation_client_all_with_camera.py  # 수정 ★
│   └── qr_pose_correction_node.py                   # 신규 생성 (팀원) ★
└── launch/
    └── gap_measurement_with_camera.launch    # 신규 생성 (선택) ★
```

### 5.2 CMakeLists.txt 수정사항

```cmake
## Service 파일 추가
add_service_files(
  FILES
  MoveJoint.srv
  MobilePositionTwist.srv
  QRPoseCorrection.srv  # 추가
)
```

### 5.3 Launch 파일 (선택사항)

```xml
<!-- gap_measurement_with_camera.launch -->
<launch>
    <!-- DSR Robot Connection -->
    <include file="$(find dsr_launcher)/launch/single_robot.launch">
        <arg name="mode" value="real"/>
        <arg name="host" value="192.168.137.100"/>
        <arg name="port" value="12345"/>
        <arg name="model" value="a0912"/>
    </include>
    
    <!-- 3D Camera Node (예: RealSense) -->
    <!-- <include file="$(find realsense2_camera)/launch/rs_camera.launch"/> -->
    
    <!-- QR Pose Correction Node -->
    <node pkg="testbed_operation" type="qr_pose_correction_node.py" 
          name="qr_pose_corrector" output="screen"/>
    
    <!-- Robot Control Node -->
    <node pkg="testbed_operation" type="testbed_operation_client_all_with_camera.py" 
          name="robot_controller" output="screen"/>
</launch>
```

---

## 6. ⚙️ 설정 및 파라미터

### 6.1 ROS 파라미터 (선택사항)

```yaml
# config/gap_measurement.yaml
qr_pose_correction:
  camera_topic: "/camera/image_raw"
  depth_topic: "/camera/depth"
  service_timeout: 10.0
  min_confidence: 0.8
  
robot_control:
  default_velocity: 30.0
  default_acceleration: 30.0
  correction_velocity: 10.0  # 보정시 저속 이동
```

### 6.2 좌표계 정의

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              좌표계 정의                                     │
└─────────────────────────────────────────────────────────────────────────────┘

1. 로봇 베이스 좌표계 (Robot Base Frame)
   - 원점: 로봇 베이스 중심
   - X: 전방 (로봇 정면)
   - Y: 좌측
   - Z: 상방

2. 엔드이펙터 좌표계 (End-Effector Frame)
   - 원점: 엔드이펙터 TCP (Tool Center Point)
   - 카메라가 부착된 위치 기준

3. 카메라 좌표계 (Camera Frame)
   - 원점: 카메라 광학 중심
   - X: 이미지 우측
   - Y: 이미지 하방
   - Z: 카메라 전방 (광축 방향)

4. QR 코드 좌표계 (QR Frame)
   - 원점: QR 코드 중심
   - X: QR 코드 우측
   - Y: QR 코드 상방
   - Z: QR 코드 표면에서 수직으로 올라오는 방향
```

---

## 7. 🧪 테스트 계획

### 7.1 단위 테스트

| 테스트 항목 | 테스트 방법 | 예상 결과 |
|------------|------------|----------|
| 서비스 서버 실행 | `rosservice list` | `/qr_pose_correction` 확인 |
| 서비스 호출 (Mock) | `rosservice call` | Response 수신 확인 |
| QR 코드 검출 | 테스트 이미지 사용 | 코너 좌표 반환 |
| 자세 계산 | 알려진 QR 위치 사용 | 오차 5mm 이내 |

### 7.2 통합 테스트

```bash
# 1. DSR 로봇 연결
roslaunch dsr_launcher single_robot.launch mode:=real host:=192.168.137.100 port:=12345 model:=a0912

# 2. 카메라 노드 실행 (팀원 담당)
rosrun testbed_operation qr_pose_correction_node.py

# 3. 로봇 제어 노드 실행
rosrun testbed_operation testbed_operation_client_all_with_camera.py

# 4. 서비스 수동 테스트
rosservice call /qr_pose_correction "{robot_ready: true, current_joint_pos: [90, 0, 90, 0, 90, -90], measurement_point_id: 'test'}"
```

### 7.3 예상 출력

```
[Robot Controller]
[INFO] === Step 1: Moving to home position ===
[INFO] DSR move_home succeeded.
[INFO] === Step 2: Moving to QR measurement position ===
[INFO] DSR move_joint succeeded.
[INFO] === Step 3: Requesting QR pose correction ===
[INFO] Waiting for QR pose correction service: /qr_pose_correction
[INFO] Requesting QR pose correction for point: A-point1
[INFO] QR pose correction succeeded (confidence: 0.95)
[INFO] === Step 4: Applying pose correction ===
[INFO] Applying pose correction: delta_rz = 1.100 deg
[INFO] DSR move_joint succeeded.
[INFO] === All operations completed successfully! ===
```

---

## 8. 📋 체크리스트

### 8.1 로봇 제어 (User) 체크리스트

- [ ] `QRPoseCorrection.srv` 파일 생성
- [ ] `CMakeLists.txt`에 srv 파일 추가
- [ ] `catkin_make` 빌드 성공
- [ ] `request_qr_pose_correction()` 함수 구현
- [ ] `apply_pose_correction()` 함수 구현
- [ ] `main()` 함수 수정
- [ ] 단위 테스트 통과

### 8.2 카메라 보정 (팀원) 체크리스트

- [ ] `qr_pose_correction_node.py` 파일 생성
- [ ] 카메라 토픽 구독 확인
- [ ] QR 코드 검출 로직 구현
- [ ] PnP 기반 자세 추정 구현
- [ ] 서비스 서버 테스트
- [ ] 보정값 정확도 검증

---

## 9. 📝 향후 확장 계획

### 9.1 Phase 2: 정밀 보정
- Inverse Kinematics를 활용한 카테시안 좌표 보정
- 다중 QR 마커 사용한 정밀도 향상

### 9.2 Phase 3: 갭 측정 통합
- 갭 단차 측정 알고리즘 연동
- 측정 결과 저장 및 리포팅

### 9.3 Phase 4: 자동화
- 전체 측정 시퀀스 자동화
- 에러 복구 로직 추가



