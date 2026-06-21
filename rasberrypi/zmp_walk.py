#!/usr/bin/env python3
import serial
import math
import time
import numpy as np

# 다이나믹셀 변환 상수
RAD_TO_DXL_STEP = 2048.0 / math.pi

class PureZMPWalkWithIMU:
    def __init__(self):
        # 1. OpenCR과 직접 시리얼 연결 (보레이트 고속 1Mbps 세팅)
        self.port = "/dev/ttyACM0"
        self.baud = 1000000
        
        try:
            # timeout과 write_timeout을 타이트하게 잡아 루프 지연 방지
            self.py_serial = serial.Serial(
                port=self.port, 
                baudrate=self.baud, 
                timeout=0.001, 
                write_timeout=0.001
            )
            print(f"➔ [SUCCESS] Connected to OpenCR at {self.baud} bps!")
        except Exception as e:
            print(f"➔ [ERROR] Failed to connect OpenCR: {e}")
            exit(1)
            
        # 보행 제어 파라미터 (미터 단위)
        self.com_height = 0.19      # 로봇 골반 높이 (zc)
        self.step_length = 0.03     # 보폭
        self.step_width = 0.06      # 양발 사이 간격
        self.step_height = 0.05    # 발 들어올릴 높이
        self.g = 9.81
        
        self.gait_period = 1.5      # 한 걸음 주기 (초)
        self.loop_rate = 50         # 제어 주파수 (50Hz = 0.02초 주기)
        self.dt = 1.0 / self.loop_rate
        
        # 실시간 IMU 상태 변수 (OpenCR 피드백 저장용)
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        
        self.last_sent_steps = [0] * 12

    def read_imu(self):
        """ 시리얼 수신 버퍼를 비동기식으로 스캔하여 실시간 IMU 데이터 파싱 """
        # 버퍼에 데이터가 쌓여 있을 때만 루프를 돌며 읽어옴 (Non-blocking)
        while self.py_serial.in_waiting > 0:
            try:
                line = self.py_serial.readline().decode('utf-8').strip()
                
                # OpenCR 아두이노가 보낸 "IMU:Roll,Pitch,Yaw" 머리글 확인
                if line.startswith("IMU:"):
                    data_str = line.split(":")[1]
                    parts = data_str.split(",")
                    
                    if len(parts) == 3:
                        self.imu_roll  = float(parts[0])
                        self.imu_pitch = float(parts[1])
                        self.imu_yaw   = float(parts[2])
            except Exception:
                # 시리얼 순간 노이즈나 쪼개진 패킷 디코딩 에러는 과감히 패스
                pass

    def generate_trajectory(self, t, support_leg='left'):
        T = self.gait_period
        t_norm = t % T
        phase_ratio = t_norm / T
        swing_sin = math.sin(phase_ratio * math.pi)

        stride  = self.step_length
        swing_h = self.step_height
        half_w  = self.step_width / 2

        # IMU 피드백 보정 (degree → meter)
        pitch_correction = -math.radians(self.imu_pitch) * 0.05  # 앞으로 기울면 발을 앞으로
        roll_correction  = -math.radians(self.imu_roll)  * 0.05  # 옆으로 기울면 sway 보정

        support_x =  stride #- stride * phase_ratio  # +0.01 → -0.01
        swing_x   = -stride #+ stride * phase_ratio  # -0.01 → +0.01

        if support_leg == 'left':
            left_foot  = np.array([support_x, 0.0, -self.com_height])
            right_foot = np.array([swing_x,   0.0, -self.com_height + swing_h * swing_sin])
            com_y = -half_w + roll_correction  # sway + roll 보정
        else:
            right_foot = np.array([support_x, 0.0, -self.com_height])
            left_foot  = np.array([swing_x,   0.0, -self.com_height + swing_h * swing_sin])
            com_y = half_w + roll_correction

        com_offset = np.array([pitch_correction, com_y, 0.0])
        target_left  = left_foot  - com_offset
        target_right = right_foot - com_offset

        return target_left, target_right

    def analytical_ik(self, x, y, z):
        L1, L2 = 0.09, 0.09
        base_yaw = 0.0

        # z는 항상 음수 (아래 방향)
        # hip_roll: y와 z의 크기로만 계산 (부호 별도 처리)
        leg_len_yz = math.sqrt(y**2 + z**2)  # YZ 평면 거리
        hip_roll   = math.atan2(y, -z)        # y 양수면 roll 양수
        ankle_roll = -hip_roll

        # XZ 평면 유효 길이: YZ 평면 성분 제거
        # z_plane은 항상 음수여야 함
        z_plane = -leg_len_yz  # YZ 평면 거리를 Z축으로 투영

        R = math.sqrt(x**2 + z_plane**2)
        R = min(R, L1 + L2 - 1e-6)

        cos_knee = (L1**2 + L2**2 - R**2) / (2 * L1 * L2)
        cos_knee = max(min(cos_knee, 1.0), -1.0)
        knee_pitch = math.pi - math.acos(cos_knee)

        alpha = math.atan2(x, -z_plane)
        beta  = math.asin(max(min(L2 * math.sin(knee_pitch) / R, 1.0), -1.0))
        hip_pitch   = alpha - beta
        ankle_pitch = -(hip_pitch + knee_pitch)

        return [base_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]


    def run(self):
        t = 0.0
        support = 'left'
        print("➔ Starting Pure ZMP Engine with IMU... Press Ctrl+C to stop.")
        
        try:
            while True:
                start_time = time.time()
                
                # 1. 최신 IMU 데이터 수신 및 스캔
                self.read_imu()
                
                # 2. IMU 피드백이 누적 반영된 실시간 궤적 계산
                pos_L, pos_R = self.generate_trajectory(t, support_leg=support)

                print(f"[RAW] t={t:.3f} swing_x={-self.step_length/2 + self.step_length*(t/self.gait_period):.4f}")
                print(f"target_L xyz: {pos_L}")
                print(f"target_R xyz: {pos_R}")

                # 3. 역기하학 관절 각도 계산
                angles_L = self.analytical_ik(pos_L[0], pos_L[1], pos_L[2])
                angles_R = self.analytical_ik(pos_R[0], pos_R[1], pos_R[2])
                print("ang_L:",angles_L)
                print("ang_R:",angles_R)
                all_angles = angles_L + angles_R  
                
                # 4. 다이나믹셀 엔코더 스텝 패킷 빌드
                cmd_tokens = []
                any_changed = False
                
                for idx, radian in enumerate(all_angles):
                    joint_id = idx + 1 

                    if joint_id in [3,4,11]:
                        radian = -radian
                    
                    step_offset = int(radian * RAD_TO_DXL_STEP)
                    
                    # 데드존 필터 (5스텝 미만의 진동성 과부하 차단)
                    if abs(step_offset - self.last_sent_steps[joint_id - 1]) > 5:
                        cmd_tokens.append(f"{joint_id},{step_offset}")
                        self.last_sent_steps[joint_id - 1] = step_offset
                        any_changed = True
                
                # 5. OpenCR로 시리얼 문자열 직접 전송
                if any_changed and cmd_tokens:
                    serial_cmd = "/".join(cmd_tokens) + "\n"
                    try:
                        self.py_serial.write(serial_cmd.encode())
                    except serial.SerialTimeoutException:
                        pass
                
                # 보행 주기 시간 제어 및 지지발 전환
                t += self.dt
                if t >= self.gait_period:
                    t = 0.0
                    support = 'right' if support == 'left' else 'left'
                    # 루프에 방해되지 않도록 발이 바뀔 때만 한 번씩 데이터 정기 모니터링 출력
                    print(f"📡 [DEBUG] Leg Switched. Current IMU -> Roll: {self.imu_roll:.2f} | Pitch: {self.imu_pitch:.2f}")
                
                # 정밀한 50Hz(0.02초) 타이밍 동기화
                elapsed = time.time() - start_time
                sleep_time = max(0.0, self.dt - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n➔ Stopping robot and closing port safely.")
            self.py_serial.close()

if __name__ == '__main__':
    walker = PureZMPWalkWithIMU()
    walker.run()
