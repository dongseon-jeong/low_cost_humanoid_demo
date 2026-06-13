
## 3d 프린팅 부품 설계

- onshape이나 fusion 사용
- op3의 stl 파일이 있으나 본래 부분은 금속소재로 플라스틱 출력 시의 강도가 예상되지 않아, 새로 적당한 두께로 설계
- 모터 stl를 불러와서 사용하고 다른 부분의 스케치가 없어서 대략적으로 스케치하여 모델링 

![이미지](../image/Pasted%20image%2020251129122659.png)

## 3d 조립 완성 및 단순구조 변환

![이미지](../image/Pasted%20image%2020251129122817.png)

- 단순 구조로 변환
	- 모터의 회전 조인트 및 링크 이름 설정 및 조인트 한계각도 설정, 무게 설정도 지금해도 됨  
	- fusion2urdf repo에 유의사항을 체크하여 아래와 같이 변환해야 urdf 변환 오류가 발생안됨

![이미지](../image/Pasted%20image%2020251129123000.png)

https://github.com/dheena2k2/fusion2urdf-ros2
https://github.com/runtimerobotics/fusion360-urdf-ros2

## urdf 변환 및 usd 파일 생성

![이미지](../image/Pasted%20image%2020251129135616.png)

isaacsim에서 urdf import하여 inertial collision friction damping stiffness 등 설정하여 usd 저장  
legs를 default prim 설정  
collider 등 기본 설정이 잘못되면 학습도 안됨  
root joint는 고정 암일 경우나 필요  

![이미지](../image/Pasted%20image%2020251129123345.png)

## isaaclab 걷기 학습

환경 생성
```bash
./isaaclab.sh --new
Task type : Externel
Project path : d:/making/dynamixel/leg
Project name : leg
Direct | single-agent
rsl_rel, skrl
AMP, PPO
```

- 기본으로 생성하면 다음과 같이 생성됨
- cart pole로 되어 있어 커스텀 코드로 수정

![이미지](../image/Pasted%20image%2020251129135716.png)

- usd 파일
leg\source\leg\leg\tasks\direct\leg\legs_cfg.py


- 기본 환경 설정
leg\source\leg\leg\tasks\direct\leg\leg_env_cfg.py

- 액션 스페이스 및 보상 등 중요한 환경 요소 수정
leg\source\leg\leg\tasks\direct\leg\leg_env.py

- 모델 관련  
leg\source\leg\leg\tasks\direct\leg\agents\rsl_rl_ppo_cfg.py  
leg\source\leg\leg\tasks\direct\leg\agents\skrl_amp_cfg.yaml  
leg\source\leg\leg\tasks\direct\leg\agents\skrl_ppo_cfg.yaml  

- 그 외 확인할 것  
leg\source\leg\leg\tasks\direct\leg\__init__.py
leg\scripts\rsl_rl\train.py



- 학습 실행
다리 사이 간격과 com 위치 속도 등이 보상에 크게 중요하고 학습 시 보상이 더 이상 커지지 않는 경우 파라미터 조정이 필요함

```bash
# 환경 변수 추가
export BASE_PATH="복사한경로"

python scripts/rsl_rl/train.py --task=Template-Leg-Direct-v0 --num_envs=1000 --max_iterations=10000
```


![이미지](../image/Pasted%20image%2020251129123624.png)

```d
################################################################################
                      Learning iteration 748/10000

                       Computation: 2228 steps/s (collection: 1.283s, learning 0.153s)
             Mean action noise std: 0.41
          Mean value_function loss: 62.6343
               Mean surrogate loss: -0.0087
                 Mean entropy loss: 5.4457
                       Mean reward: 1.85
               Mean episode length: 6.97
--------------------------------------------------------------------------------
                   Total timesteps: 2396800
                    Iteration time: 1.44s
                      Time elapsed: 00:17:08
                               ETA: 03:31:46
```

- 추론
```bash
python scripts/rsl_rl/play.py --task Template-Leg-Direct-v0 --num_envs 20 --checkpoint logs/rsl_rl/leg/model_99999.pt
```

https://github.com/user-attachments/assets/80280dde-8090-4f91-8ec1-0bcc60204a35


## 3d 프린팅 조립
- 각 stl 파일 프린팅 > 구조 강도 개선과 조립 위해 특정 부품은 새롭게 모델링 진행  
프린팅 시 나사 구멍 등이 수축됨, 히트 인서트를 모든 나사 구멍에 삽입하는 것은 매우 힘든 일이므로 테스트 출력으로 나사 구멍 사이즈 조정 필요
- 각 부품 무게 측정 > 실제 무게로 usd 파일 설정

![이미지](../image/20260530_184327.jpg)


## 아두이노 opencr 모터컨트롤

**다이나밀셀 위저드로도 기본 설정 가능  
- 아두이노 ide에 opencr 설치 후 usb 연결하면 보드확인 및 라이브러리 사용 가능함  
- 처음 연결하면 공장 초기화 모터아이디는 모두 1번임, 모터 체인에 따라 차례대로 번호 세팅을 해야함  
  
여기에서는 왼발 위쪽 부터 차례대로 1~6번  
오른발 위쪽 부터 차례대로 7~12번 세팅  

```arduino
check_motor_id.io 보드를 usb 연결 후 코드 실행 하면 시리얼 모니터로 모터 아이디 번호 확인 가능
edit_motor_id.io 실행 하면 모터 아이디를 수정 가능
check_motor_position.io 현 모터의 포지션 수치를 시리얼 모니터로 확인
```

**중요
urdf에 저장된 조인트 라디안 min max값을 적용하면 실제 모터 방향과 다르게 움직여 다음과 같이 적용함 (fusion에서 접합 구동으로 로봇을 조정 후 urdf 파일로 변환한 이유일 것으로 추측)  

```arduino
const float URDF_MIN_RAD[12] = {
  -1.57,   -1.57,   -1.57,    0.0,   -0.43, -1.04, // 왼발 (1번: -1.57)
  0.0,     -0.43,   -0.43,   -1.39,  -1.57, -1.57  // 오른발 (7번: 0.0)
};

const float URDF_MAX_RAD[12] = {
  0.0,    0.43,     0.43,     1.39,   1.57,  1.57,  // 왼발 (1번: 0.0)
  1.57,   1.57,     1.57,     0.0,    0.43,  1.04   // 오른발 (7번: 1.57)
};
```

다음값을 각 모터에 입력하면 라디안 변환하여 움직여짐(180도에 해당하는 모터 값은 2048임, 1000은 약 90도)  
```arduino
1: -1000 ~ 0
2: -1000 ~ +200
3: +1000 ~ -200
4: -1000 ~ 0
5: -1000 ~ +200
6: -1000 ~ +300

7: 0 ~ +1000
8: -200 ~ +1000
9: -1000 ~ +200
10: 0 ~ +1000
11: -200 ~ +1000
12: -300 ~ +1000
```



```arduino
motor_control.io 라즈베리파이에서 값을 python 스크립트나 ros로 통신하기 위한 코드, usb 연결하여 실행 후 업로드 완료하면 라즈베리파이에 usb연결하여 사용
```



## rasberrypi opencr 연결
라즈베리파이 파이썬 스크립트 컨트롤 >


ros 컨트롤 >
```bash
# 초기 세팅 외부 호스트
# 1. 모든 로컬 사용자의 디스플레이 접근 제한을 완전히 해제 (보안 비활성화) 
xhost + 
# 2. Xauthority 인증 파일 권한을 모두가 읽을 수 있도록 개방 
chmod 644 ~/.Xauthority

# 초기 세팅 컨테이너 내부
apt update
apt install -y ros-noetic-joint-state-publisher-gui ros-noetic-robot-state-publisher ros-noetic-rviz
apt install -y python-is-python3
apt install -y python3-pip 
pip3 install pyserial



# 컨테이너 외부 호스트
# 1. 디스플레이 :0 환경변수를 임시 지정하여 xhost 무력화
DISPLAY=:0 xhost +local:docker

# 2. 만약 위 명령어가 실패하면 디바이스 전체 개방으로 재시도
DISPLAY=:0 xhost +

# 컨테이너 내부
source /opt/ros/noetic/setup.bash

cd /workspace/catkin_ws 
catkin_make 
source devel/setup.bash # 런치 파일 실행 

# 1. 완벽한 소프트웨어 렌더링 강제 (Mesa softpipe 지정) 
export LIBGL_ALWAYS_SOFTWARE=1 
export GALLIUM_DRIVER=softpipe 
# 2. X11 공유 메모리 버퍼 완전히 비활성화 (Segmentation Fault 차단 핵심) 
export QT_X11_NO_MITSHM=1 
export MITSHM=0 
# 3. 만약 하이딩된 세션 에러 방지를 위해 디스플레이 다시 선언 
export DISPLAY=:0
roslaunch opencr_rcm_control display_and_control.launch
```


## isaacsim sim2real 구현

isaacsim에서 조인트 damping stiffness 설정 필요

```
[WSL2 - isaac_bin conda]
  Isaac Sim 5.0
  → ROS2 /joint_states 퍼블리시
  → roslibpy 클라이언트
        ↕ WebSocket (LAN)
[라즈베리파이 Docker: ros_noetic]
  rosbridge_server (WebSocket :9090)
  → roscore
  → 로봇 (/dev/ttyACM0)
```

ros1 rasberrypi 컨테이너
```bash
apt update 
apt install -y ros-noetic-rosbridge-suite 
source /opt/ros/noetic/setup.bash 
roslaunch rosbridge_server rosbridge_websocket.launch
```

conda
```bash
pip install roslibpy


isaacsim --enable isaacsim.ros2.bridge --enable isaacsim.ros2.sim_control --enable omni.isaac.ros2_bridge
```

leg_w.usd 파일 로드
플레이 후 scripts editor 실행
```bash
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import serial
import math

RAD_TO_DXL_STEP = 2048.0 / math.pi

class ROSSerialBridgeFast:
    def __init__(self):
        rospy.init_node('ros_serial_bridge', anonymous=False)
        
        # 1. 초고속 통신을 위해 1Mbps로 보레이트 상향 (OpenCR 소스도 맞춰야 함)
        self.port = "/dev/ttyACM0"
        self.baud = 115200  
        
        try:
            # write_timeout을 추가하여 시리얼 전송 지연이 메인 루프를 막는 현상 방지
            self.py_serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.001, write_timeout=0.001)
            rospy.loginfo("➔ [FAST] Connected to OpenCR at 1Mbps!")
        except Exception as e:
            rospy.logerr(f"➔ Failed to connect OpenCR: {e}")
            return

        self.last_sent_steps = [0] * 12

        self.joint_id_map = {
            "lbase_joint": 1, "ll1_joint": 2, "ll2_joint": 3, "ll3_joint": 4, "ll4_joint": 5, "ll5_joint": 6,
            "rbase_joint": 7, "rl1_joint": 8, "rl2_joint": 9, "rl3_joint": 10, "rl4_joint": 11, "rl5_joint": 12
        }

        # 2. tcp_nodelay=True를 추가하여 ROS 네트워크 패킷 지연(Nagle 알고리즘)을 원천 차단
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("➔ [FAST] ROS-Isaac Serial Bridge Running...")

    def joint_state_callback(self, msg):
        cmd_tokens = []
        any_changed = False

        for i, name in enumerate(msg.name):
            if name in self.joint_id_map:
                joint_id = self.joint_id_map[name]
                radian = msg.position[i]
                
                if joint_id == 1:
                    radian = -radian
                
                step_offset = int(radian * RAD_TO_DXL_STEP)

                # 3. 필터 컷오프를 5스텝(약 0.4도)으로 상향하여 무의미한 고주파 트래픽 차단
                if abs(step_offset - self.last_sent_steps[joint_id - 1]) > 10:
                    cmd_tokens.append(f"{joint_id},{step_offset}")
                    self.last_sent_steps[joint_id - 1] = step_offset
                    any_changed = True

        if any_changed and cmd_tokens:
            serial_cmd = "/".join(cmd_tokens) + "\n"
            try:
                self.py_serial.write(serial_cmd.encode())
                # 속도 최적화를 위해 실시간 주크박스 로그(rospy.loginfo)는 주석 처리합니다.
                # 터미널에 프린트를 찍는 행위 자체가 파이썬 연산 속도를 엄청나게 갉아먹습니다.
            except serial.SerialTimeoutException:
                pass # 시리얼 버퍼 밀림 발생 시 과감히 패킷 드랍하여 실시간성 유지

    def shutdown_hook(self):
        if hasattr(self, 'py_serial') and self.py_serial.is_open:
            self.py_serial.close()

if __name__ == '__main__':
    bridge = ROSSerialBridgeFast()
    rospy.on_shutdown(bridge.shutdown_hook)
    rospy.spin()
```

rostopic echo /joint_states
```bash
cd /workspace/catkin_ws 
source devel/setup.bash 
chmod +x src/opencr_rcm_control/scripts/ros_serial_bridge.py # 파이썬 중계 노드 단독 기동! 

rosrun opencr_rcm_control ros_serial_bridge.py
```

## locomotion 구현

+zmp  
지면과 발바닥 사이에 마찰력이 없어서 발바닥에 고무 부착 필요

https://github.com/user-attachments/assets/6223d0d4-1970-4561-afc3-cd891884fd33  


강화학습모델 realworld 적용  
navigation  

