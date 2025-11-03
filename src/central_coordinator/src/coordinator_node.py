#!/usr/bin/env python3
"""
중앙 관제 노드 (Topic 기반)
모바일 로봇과 두산 로봇을 순차적으로 제어

시퀀스:
  [초기화] 두산 로봇이 STANDBY 상태가 될 때까지 대기 → 홈 위치 (명령 99)
  [사이클]
    1. 모바일 로봇 전진 (distance, speed)
    2. 두산 로봇 작업 자세 (명령 1)
    3. 모바일 로봇 후진 (distance, -speed)
    4. 두산 로봇 홈 위치 (명령 99)
"""
import rospy
from std_msgs.msg import String, Int32, Float64MultiArray
from dsr_msgs.msg import RobotState


class CentralCoordinator:
    """중앙 관제 노드 - 두 로봇의 왕복 협업 제어"""
    
    # 두산 로봇 상태 상수
    STATE_INITIALIZING = 0
    STATE_STANDBY = 1
    STATE_MOVING = 2
    STATE_SAFE_OFF = 3
    STATE_TEACHING = 4
    STATE_SAFE_STOP = 5
    STATE_EMERGENCY_STOP = 6
    STATE_HOMMING = 8
    
    def __init__(self):
        rospy.init_node('central_coordinator', anonymous=False)
        
        # Publishers (명령 발행)
        self.mobile_cmd_pub = rospy.Publisher('/mobile/cmd', Float64MultiArray, queue_size=1)
        self.doosan_cmd_pub = rospy.Publisher('/katech/robot_command', Int32, queue_size=1)
        
        # Subscribers (상태 수신)
        self.mobile_status_sub = rospy.Subscriber('/mobile/status', String, self.mobile_status_callback)
        self.doosan_status_sub = rospy.Subscriber('/doosan/status', String, self.doosan_status_callback)
        
        # 두산 로봇 실제 상태 구독 (드라이버에서 직접 발행)
        self.doosan_robot_state_sub = rospy.Subscriber('/dsr01a0912/state', RobotState, self.doosan_robot_state_callback)
        
        # 상태 변수
        self.mobile_status = "UNKNOWN"
        self.doosan_status = "UNKNOWN"
        self.doosan_robot_state = -1  # 실제 로봇 상태 (0~16)
        self.doosan_robot_state_str = "UNKNOWN"
        
        # 시나리오 파라미터 (User가 수정 가능)
        self.mobile_distance = rospy.get_param('~mobile_distance', 0.3)  # 이동 거리 (m)
        self.mobile_speed = rospy.get_param('~mobile_speed', 0.2)        # 이동 속도 (m/s)
        self.cycle_delay = rospy.get_param('~cycle_delay', 5.0)         # 사이클 간 대기 (초)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎮 중앙 관제 노드 시작")
        rospy.loginfo(f"   모바일: {self.mobile_distance}m, {self.mobile_speed}m/s")
        rospy.loginfo(f"   사이클 대기: {self.cycle_delay}초")
        rospy.loginfo("=" * 60)
        
        # 초기화 대기 (노드들이 준비될 시간)
        rospy.sleep(2.0)
    
    def mobile_status_callback(self, msg):
        """모바일 로봇 상태 업데이트"""
        self.mobile_status = msg.data
        rospy.loginfo(f"📱 모바일 상태: {msg.data}")
    
    def doosan_status_callback(self, msg):
        """두산 로봇 동작 상태 업데이트 (우리가 만든 move_robot_node에서 발행)"""
        self.doosan_status = msg.data
        rospy.loginfo(f"🦾 두산 동작 상태: {msg.data}")
    
    def doosan_robot_state_callback(self, msg):
        """두산 로봇 실제 상태 업데이트 (드라이버에서 발행)"""
        self.doosan_robot_state = msg.robot_state
        self.doosan_robot_state_str = msg.robot_state_str
        # 상태 변경 시에만 로그 출력
        if hasattr(self, '_last_robot_state') and self._last_robot_state != msg.robot_state:
            rospy.loginfo(f"🤖 두산 시스템 상태: {msg.robot_state} ({msg.robot_state_str})")
        self._last_robot_state = msg.robot_state
    
    def wait_for_doosan_ready(self, timeout=60.0):
        """
        두산 로봇이 STANDBY 상태가 될 때까지 대기
        
        Args:
            timeout: 최대 대기 시간 (초)
        
        Returns:
            bool: 성공 여부
        """
        rospy.loginfo("⏳ 두산 로봇이 STANDBY 상태가 될 때까지 대기 중...")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(5)  # 5Hz
        safe_off_warning_shown = False
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            # STANDBY 상태 확인
            if self.doosan_robot_state == self.STATE_STANDBY:
                rospy.loginfo(f"✅ 두산 로봇 준비 완료! 상태: STANDBY")
                return True
            
            # SAFE_OFF 상태에서 5초 이상 멈춰있으면 경고 표시
            if self.doosan_robot_state == self.STATE_SAFE_OFF and elapsed > 5.0 and not safe_off_warning_shown:
                rospy.logwarn("\n" + "⚠️ " * 20)
                rospy.logwarn("⚠️  두산 로봇이 SAFE_OFF 상태입니다!")
                rospy.logwarn("⚠️  ")
                rospy.logwarn("⚠️  다음 작업을 수행하세요:")
                rospy.logwarn("⚠️  1. 티치 펜던트 또는 로봇 제어판 확인")
                rospy.logwarn("⚠️  2. '서보 온(Servo On)' 버튼 누르기")
                rospy.logwarn("⚠️  3. 로봇 상태가 STANDBY로 변경될 때까지 대기")
                rospy.logwarn("⚠️  ")
                rospy.logwarn("⚠️ " * 20 + "\n")
                safe_off_warning_shown = True
            
            # 타임아웃 체크
            if elapsed > timeout:
                rospy.logerr(f"❌ 타임아웃 ({timeout}초 초과)")
                rospy.logerr(f"   최종 상태: {self.doosan_robot_state} ({self.doosan_robot_state_str})")
                rospy.logerr("   로봇이 SAFE_OFF 상태라면 서보 온을 해야 합니다!")
                return False
            
            # 주기적으로 상태 출력 (5초마다)
            if int(elapsed) % 5 == 0 and elapsed > 0:
                remaining = int(timeout - elapsed)
                rospy.loginfo(f"   대기 중... ({int(elapsed)}초 경과, 남은 시간: {remaining}초, 현재: {self.doosan_robot_state_str})")
            
            rate.sleep()
        
        return False
    
    def wait_for_status(self, robot_name, target_status, timeout=60.0):
        """
        특정 로봇이 목표 상태가 될 때까지 대기
        
        Args:
            robot_name: "mobile" 또는 "doosan"
            target_status: 기다릴 상태 (예: "COMPLETED", "IDLE")
            timeout: 최대 대기 시간 (초)
        
        Returns:
            bool: 성공 여부
        """
        rospy.loginfo(f"⏳ {robot_name} 로봇이 '{target_status}' 상태가 될 때까지 대기...")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 현재 상태 확인
            if robot_name == "mobile":
                current_status = self.mobile_status
            elif robot_name == "doosan":
                current_status = self.doosan_status
            else:
                rospy.logerr(f"❌ 알 수 없는 로봇 이름: {robot_name}")
                return False
            
            # 목표 상태 도달
            if current_status == target_status:
                rospy.loginfo(f"✅ {robot_name} 로봇 상태: {target_status}")
                return True
            
            # 에러 상태 체크
            if current_status == "ERROR":
                rospy.logerr(f"❌ {robot_name} 로봇 에러 발생!")
                return False
            
            # 타임아웃 체크
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logerr(f"❌ 타임아웃 ({timeout}초 초과)")
                return False
            
            rate.sleep()
        
        return False
    
    def send_mobile_command(self, distance, speed):
        """모바일 로봇에게 이동 명령 전송"""
        rospy.loginfo(f"➡️  모바일 로봇 명령: {distance}m, {speed}m/s")
        
        cmd = Float64MultiArray()
        cmd.data = [distance, speed]
        
        # 명령 발행 (여러 번 전송하여 확실히 전달)
        for _ in range(3):
            self.mobile_cmd_pub.publish(cmd)
            rospy.sleep(0.1)
    
    def send_doosan_command(self, command_id):
        """두산 로봇에게 동작 명령 전송"""
        command_name = {99: "홈", 0: "자세0", 1: "자세1"}.get(command_id, f"명령{command_id}")
        rospy.loginfo(f"➡️  두산 로봇 명령: {command_id} ({command_name})")
        
        cmd = Int32()
        cmd.data = command_id
        
        # 명령 발행 (여러 번 전송하여 확실히 전달)
        for _ in range(3):
            self.doosan_cmd_pub.publish(cmd)
            rospy.sleep(0.1)
    
    def initialize_robots(self):
        """
        로봇 초기화
        1. 두산 로봇이 STANDBY 상태가 될 때까지 대기
        2. 두산 로봇을 홈 위치로 이동
        """
        rospy.loginfo("\n" + "🏠" * 30)
        rospy.loginfo("🏠 시스템 초기화 시작")
        rospy.loginfo("🏠" * 30 + "\n")
        
        # 1단계: 두산 로봇 STANDBY 대기
        rospy.loginfo("[1/2] 두산 로봇 연결 및 서보 온 대기 중...")
        rospy.loginfo("      로봇이 SAFE_OFF 상태라면 티치 펜던트에서 서보 온을 해주세요.")
        if not self.wait_for_doosan_ready(timeout=60.0):
            rospy.logerr("❌ 초기화 실패: 두산 로봇이 STANDBY 상태가 되지 않음")
            rospy.logerr("   📌 확인 사항:")
            rospy.logerr("      1. 로봇 전원이 켜져 있는지 확인")
            rospy.logerr("      2. 네트워크 연결 확인 (192.168.137.100)")
            rospy.logerr("      3. 티치 펜던트에서 '서보 온' 버튼을 눌렀는지 확인")
            rospy.logerr("      4. 비상 정지가 눌려있지 않은지 확인")
            return False
        
        rospy.loginfo("✅ 두산 로봇 연결 완료\n")
        rospy.sleep(1.0)
        
        # 2단계: 두산 로봇 홈 위치 이동
        rospy.loginfo("[2/2] 두산 로봇 홈 위치로 이동 중...")
        self.send_doosan_command(99)
        
        # 완료 대기
        if not self.wait_for_status("doosan", "COMPLETED", timeout=60.0):
            rospy.logerr("❌ 초기화 실패: 두산 로봇 홈 이동 실패")
            return False
        
        rospy.loginfo("✅ 초기화 완료: 두산 로봇이 홈 위치에 도달했습니다.\n")
        rospy.sleep(2.0)
        return True
    
    def run_sequence(self):
        """
        메인 시퀀스 실행
        
        시퀀스:
          1. 모바일 전진
          2. 두산 작업 자세
          3. 모바일 후진 (복귀)
          4. 두산 홈 자세
        """
        # 초기화
        if not self.initialize_robots():
            rospy.logerr("초기화 실패. 프로그램 종료.")
            return
        
        cycle_count = 1
        
        while not rospy.is_shutdown():
            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo(f"🔄 사이클 {cycle_count} 시작")
            rospy.loginfo("=" * 60)
            
            # ============================================================
            # 1단계: 모바일 로봇 전진
            # ============================================================
            rospy.loginfo("\n[1/4] 📱 모바일 로봇 전진")
            self.send_mobile_command(self.mobile_distance, self.mobile_speed)
            
            # 완료 대기
            if not self.wait_for_status("mobile", "COMPLETED", timeout=60.0):
                rospy.logerr("모바일 로봇 전진 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 모바일 로봇 전진 완료")
            rospy.sleep(1.0)
            
            # ============================================================
            # 2단계: 두산 로봇 작업 자세 (명령 1)
            # ============================================================
            rospy.loginfo("\n[2/4] 🦾 두산 로봇 작업 자세")
            self.send_doosan_command(1)  # 자세 1
            
            # 완료 대기
            if not self.wait_for_status("doosan", "COMPLETED", timeout=60.0):
                rospy.logerr("두산 로봇 작업 동작 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 두산 로봇 작업 완료")
            rospy.sleep(1.0)
            
            # ============================================================
            # 3단계: 모바일 로봇 후진 (복귀)
            # ============================================================
            rospy.loginfo("\n[3/4] 📱 모바일 로봇 후진 (복귀)")
            self.send_mobile_command(self.mobile_distance, -self.mobile_speed)  # 속도 반대
            
            # 완료 대기
            if not self.wait_for_status("mobile", "COMPLETED", timeout=60.0):
                rospy.logerr("모바일 로봇 후진 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 모바일 로봇 후진 완료")
            rospy.sleep(1.0)
            
            # ============================================================
            # 4단계: 두산 로봇 홈 위치 복귀 (명령 99)
            # ============================================================
            rospy.loginfo("\n[4/4] 🦾 두산 로봇 홈 위치 복귀")
            self.send_doosan_command(99)  # 홈
            
            # 완료 대기
            if not self.wait_for_status("doosan", "COMPLETED", timeout=60.0):
                rospy.logerr("두산 로봇 홈 복귀 실패. 10초 후 재시도...")
                rospy.sleep(10.0)
                continue
            
            rospy.loginfo("✅ 두산 로봇 홈 복귀 완료")
            
            # ============================================================
            # 사이클 완료
            # ============================================================
            rospy.loginfo(f"\n✅✅✅ 사이클 {cycle_count} 완료! ✅✅✅")
            rospy.loginfo(f"⏸️  {self.cycle_delay}초 대기 후 다음 사이클 시작...\n")
            
            cycle_count += 1
            rospy.sleep(self.cycle_delay)


if __name__ == '__main__':
    try:
        coordinator = CentralCoordinator()
        coordinator.run_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("👋 관제 노드 종료")
