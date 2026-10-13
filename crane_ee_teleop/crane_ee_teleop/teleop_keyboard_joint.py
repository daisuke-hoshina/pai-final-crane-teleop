#!/usr/bin/env python3
"""関節空間キーボードテレオペ（比較実験の対照群）。

matsuolab/physicalai_teleop の physicalai_teleop/teleop_keyboard.py
（コミット 0ef2a00fd6d7bb39377969d10f3d26268e447fd9、MIT License）をコピーしたもの。
変更点は 2 か所だけ:
  1. グリッパーの指令の time_from_start を 0.1 秒 → GRIPPER_TIME_FROM_START（0.01 秒）
  2. X で戻る姿勢を直立 (0, 0, 0, 0) → 手先空間テレオペと同じホーム（机の上 10cm、手先はほぼ真下向き）

2 の理由: 比較実験で、どちらのテレオペでもエピソードを同じ姿勢から始めるため。

理由: 20Hz で指令を送り直すと、コントローラーは毎回「今の角度から time_from_start かけて目標へ」と
軌道を作り直す。0.1 秒だとグリッパーの目標が今の角度のすぐ先にしかならず、ブロックに当たると
弱い力でしか握れずに滑る（2026-10-12 に Gazebo で確認）。手先空間テレオペ（teleop_keyboard_ee）と
同じ値にそろえ、比較実験で変える条件をテレオペ方式だけにする。

  ros2 run crane_ee_teleop teleop_keyboard_joint
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from crane_ee_teleop.teleop_keyboard_ee import HOME_Q, to_arm_order

HELP = """
Teleop Crane+ V2 (Direct Joint Control):
---------------------------------------
Q/E: Select joint (1-4)
W/S: + / - selected joint
R/F: + / - all joints (coarse)
O:   Open gripper
C:   Close gripper
X:   Go to home (same pose as teleop_keyboard_ee)
ESC: Quit
"""

ARM_JOINT_NAMES = [
    "crane_plus_joint1",
    "crane_plus_joint2",
    "crane_plus_joint4",
    "crane_plus_joint3",
]
GRIPPER_JOINT_NAME = "crane_plus_joint_hand"
# 変更点 2（元は [0.0, 0.0, 0.0, 0.0]）。並びは ARM_JOINT_NAMES と同じ (j1, j2, j4, j3)
ARM_INIT = to_arm_order(HOME_Q)
GRIPPER_OPEN = -0.5236  # -30 deg
GRIPPER_CLOSE = 0.1745  # 10 deg
ARM_STEP = 0.05
COARSE_STEP = 0.2
GRIPPER_TIME_FROM_START = 0.01  # [s] 変更点。docstring を参照


class KeyboardInput:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class TeleopNode(Node):
    def __init__(self):
        super().__init__("crane_plus_joint_teleop")
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10
        )
        self.arm_pos = ARM_INIT[:]
        self.gripper_pos = GRIPPER_OPEN
        self.selected = 0  # which arm joint to control
        print(HELP)
        print(f"Selected joint: {ARM_JOINT_NAMES[self.selected]}")
        self.running = True
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.start()
        self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz

    def keyboard_loop(self):
        with KeyboardInput() as kb:
            while self.running:
                key = kb.get_key()
                if key:
                    if key.lower() == "q":
                        self.selected = (self.selected - 1) % len(ARM_JOINT_NAMES)
                        print(f"Selected joint: {ARM_JOINT_NAMES[self.selected]}")
                    elif key.lower() == "e":
                        self.selected = (self.selected + 1) % len(ARM_JOINT_NAMES)
                        print(f"Selected joint: {ARM_JOINT_NAMES[self.selected]}")
                    elif key.lower() == "w":
                        self.arm_pos[self.selected] += ARM_STEP
                    elif key.lower() == "s":
                        self.arm_pos[self.selected] -= ARM_STEP
                    elif key.lower() == "r":
                        self.arm_pos = [x + COARSE_STEP for x in self.arm_pos]
                    elif key.lower() == "f":
                        self.arm_pos = [x - COARSE_STEP for x in self.arm_pos]
                    elif key.lower() == "o":
                        self.gripper_pos = GRIPPER_OPEN
                        print("[INFO] Gripper: OPEN")
                    elif key.lower() == "c":
                        self.gripper_pos = GRIPPER_CLOSE
                        print("[INFO] Gripper: CLOSE")
                    elif key.lower() == "x":
                        self.arm_pos = ARM_INIT[:]
                        self.gripper_pos = GRIPPER_OPEN
                        print("[INFO] Robot reset to vertical (all-zero) position.")
                    elif ord(key) == 27:  # ESC
                        self.running = False
                        rclpy.shutdown()
                        break

    def publish_commands(self):
        # Arm
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = self.arm_pos
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)  # 0.1s
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        # Gripper
        jt_g = JointTrajectory()
        jt_g.joint_names = [GRIPPER_JOINT_NAME]
        pt_g = JointTrajectoryPoint()
        pt_g.positions = [self.gripper_pos]
        pt_g.time_from_start.sec = 0
        pt_g.time_from_start.nanosec = int(GRIPPER_TIME_FROM_START * 1e9)  # 変更点（元は 0.1）
        jt_g.points.append(pt_g)
        self.gripper_pub.publish(jt_g)

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
