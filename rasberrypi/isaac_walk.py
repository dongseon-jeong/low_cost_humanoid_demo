#!/usr/bin/env python3

import serial
import math
import time
import numpy as np
import torch


RAD_TO_DXL_STEP = 2048.0 / math.pi
DEFAULT_Q = np.array([
    0,0,0,0,0,0,
    0,0,0,0,0,0
])

class RLLegController:

    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baud = 1000000
        try:
            self.py_serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=None
            )
            print(
                f"[SUCCESS] Connected OpenCR {self.baud}"
            )
        except Exception as e:
            print(e)
            exit(1)

        # --------------------
        # IMU
        # --------------------
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        # gyro(rad/s)
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.gyro = np.zeros(3)
        # --------------------
        # RL policy
        # --------------------

        self.policy = torch.jit.load(
            "./policy_jit.pt"
        )
        self.policy.eval()

        # --------------------
        # Robot state
        # --------------------

        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)
        self.last_action = np.zeros(12)

        # Isaac cfg와 동일
        self.base_height = 0.16
        self.last_sent_steps = [
            0
        ] * 12
        self.dt = 0.02   # 50Hz
        self.default_joint_pos = np.zeros(12)

        self.isaac_to_dxl = [
            0,  # lbase -> 1
            2,  # ll1   -> 2
            4,  # ll2   -> 3
            6,  # ll3   -> 4
            8,  # ll4   -> 5
            10, # ll5   -> 6

            1,  # rbase -> 7
            3,  # rl1   -> 8
            5,  # rl2   -> 9
            7,  # rl3   -> 10
            9,  # rl4   -> 11
            11  # rl5   -> 12
        ]
        self.dxl_to_isaac = np.argsort(
            self.isaac_to_dxl
        )
    # --------------------
    # OpenCR feedback
    # --------------------

    def read_feedback(self):
        while self.py_serial.in_waiting > 0:
            try:
                line = (
                    self.py_serial
                    .readline()
                    .decode()
                    .strip()
                )
                # IMU:roll,pitch,yaw
                if line.startswith("IMU:"):
                    data = line.split(":")[1]
                    vals = data.split(",")
                    if len(vals)==6:

                        self.imu_roll=float(vals[0])
                        self.imu_pitch=float(vals[1])
                        self.imu_yaw=float(vals[2])


                        self.gyro=np.deg2rad(
                            np.array(
                                vals[3:6],
                                dtype=float
                            )
                        )

                # JOINT_POS:p1,p2....
                elif line.startswith("POS:"):
                    data=line.split(":")[1]
                    vals=np.array(
                        data.split(","),
                        dtype=float
                    )
                    if len(vals)==12:
                        self.joint_pos = vals[self.dxl_to_isaac]

                # JOINT_VEL:v1,v2....
                elif line.startswith("VEL:"):
                    data=line.split(":")[1]
                    vals=np.array(
                        data.split(","),
                        dtype=float
                    )
                    if len(vals)==12:
                        vals = vals * 0.229 * 2*np.pi / 60
                        self.joint_vel = vals[self.dxl_to_isaac]
            except:
                pass

    # --------------------
    # IsaacLab observation
    # --------------------
    def make_obs(self):
        q = self.joint_pos
        qd = self.joint_vel
        base_lin_vel = np.array([
            0.0,
            0.0,
            0.0
        ])
        base_ang_vel = self.gyro
        roll = np.deg2rad(self.imu_roll)
        pitch = np.deg2rad(self.imu_pitch)

        tilt_angle = np.sqrt(
            roll**2 +
            pitch**2
        )

        obs = np.concatenate(
            [
                q,                     #12
                qd,                    #12
                base_lin_vel,           #3
                base_ang_vel,           #3
                [self.base_height],     #1
                [tilt_angle],           #1
                self.last_action,       #12
                [0.0],                 #forward_vel
                [0.0],                 #x_pos
                [0.0],                 #y_pos
                [
                self.base_height-0.19
                ]
            ]
        )

        return obs.astype(
            np.float32
        )

    # --------------------
    # RL inference
    # --------------------
    def inference(self,obs):
        # obs = self.make_obs()
        obs_tensor = torch.from_numpy(
            obs
        ).unsqueeze(0)
        with torch.no_grad():
            action = self.policy(
                obs_tensor
            )

        action = (
            action
            .cpu()
            .numpy()[0]
        )
        # action = np.clip(
        #     action,
        #     -1.0,
        #     1.0
        # )
        action_dxl = action[self.isaac_to_dxl]
        return action_dxl

    # --------------------
    # send DXL
    # --------------------
    def send_action(self, action):

        cmd_tokens = []

        # 학습 default pose + action
        target_rad = (
            self.default_joint_pos
            +
            action * 0.3
        )


        for i in range(12):

            joint_id = i + 1


            step = int(
                target_rad[i]
                *
                RAD_TO_DXL_STEP
            )


            cmd_tokens.append(
                f"{joint_id},{step}"
            )


        msg = "/".join(cmd_tokens) + "\n"


        print(msg)


        self.py_serial.write(
            msg.encode()
        )
            
    def run(self):
        print(
            "Start RL walking"
        )
        try:
            while True:
                t=time.time()
                # 1. feedback
                self.read_feedback()
                # 2. policy
                obs = self.make_obs()
                print(obs)
                action=self.inference(obs)
                # 3. motor command
                self.send_action(
                    action
                )
                self.last_action = action.copy()
                print(action)
                dt=time.time()-t
                time.sleep(
                    max(
                        0,
                        self.dt-dt
                    )
                )

        except KeyboardInterrupt:
            self.py_serial.close()

if __name__=="__main__":

    robot=RLLegController()
    robot.run()