#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import serial
import math

# 180도 = 2048 step => 1 rad = 2048 / pi
RAD_TO_DXL_STEP = 2048.0 / math.pi

class ROSSerialBridge:
    def __init__(self):
        rospy.init_node('ros_serial_bridge', anonymous=False)
        
        self.port = "/dev/ttyACM0"
        self.baud = 115200
        
        try:
            self.py_serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
            rospy.loginfo("➔ Connected to OpenCR successfully!")
        except Exception as e:
            rospy.logerr(f"➔ Failed to connect OpenCR: {e}")
            return

        self.last_sent_steps = [0] * 12

        self.joint_id_map = {
            "lbase_joint": 1,
            "ll1_joint":   2,
            "ll2_joint":   3,
            "ll3_joint":   4,
            "ll4_joint":   5,
            "ll5_joint":   6,
            
            "rbase_joint": 7,
            "rl1_joint":   8,
            "rl2_joint":   9,
            "rl3_joint":   10,
            "rl4_joint":   11,
            "rl5_joint":   12
        }

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)
        rospy.loginfo("➔ ROS Serial Bridge Node Ready.")

    def joint_state_callback(self, msg):
        cmd_tokens = []
        any_changed = False

        for i, name in enumerate(msg.name):
            if name in self.joint_id_map:
                joint_id = self.joint_id_map[name]
                
                radian = msg.position[i]
                if joint_id == 1 or joint_id == 2 or joint_id == 3 or joint_id == 4 or joint_id == 5 or joint_id == 6:
                    radian = -radian


                step_offset = int(radian * RAD_TO_DXL_STEP)

                if abs(step_offset - self.last_sent_steps[joint_id - 1]) > 2:
                    cmd_tokens.append(f"{joint_id},{step_offset}")
                    self.last_sent_steps[joint_id - 1] = step_offset
                    any_changed = True

        if any_changed and cmd_tokens:
            serial_cmd = "/".join(cmd_tokens) + "\n"
            
            rospy.loginfo(f"➔ Sending: {serial_cmd.strip()}")
            
            self.py_serial.write(serial_cmd.encode())

    def shutdown_hook(self):
        rospy.loginfo("Shutting down ROS Serial Bridge. Resetting motors to center...")
        if hasattr(self, 'py_serial') and self.py_serial.is_open:
            reset_cmd = "/".join([f"{i},0" for i in range(1, 13)]) + "\n"
            self.py_serial.write(reset_cmd.encode())
            self.py_serial.close()

if __name__ == '__main__':
    bridge = ROSSerialBridge()
    rospy.on_shutdown(bridge.shutdown_hook)
    rospy.spin()
