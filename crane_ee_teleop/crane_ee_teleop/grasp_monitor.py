"""手先からブロックまでの距離を表示する（テレオペの補助）。

/joint_states から順運動学で手先点を求め、Gazebo から読んだブロックの位置との差を出す。
指令は何も送らないので、関節空間テレオペ・手先空間テレオペのどちらとも一緒に使える。

  ros2 run crane_ee_teleop grasp_monitor
"""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from crane_ee_teleop.gazebo_objects import model_position_in_robot_frame
from crane_ee_teleop.kinematics import fk

GRIPPER_JOINT_NAME = "crane_plus_joint_hand"
PRINT_PERIOD = 0.5  # [s]


class GraspMonitor(Node):
    def __init__(self):
        super().__init__("crane_plus_grasp_monitor")
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.joints = None
        self.cube = None
        self.running = True
        threading.Thread(target=self._read_cube_loop, daemon=True).start()
        self.create_timer(PRINT_PERIOD, self._print)

    def _on_state(self, msg):
        self.joints = dict(zip(msg.name, msg.position, strict=True))

    def _read_cube_loop(self):
        while self.running:
            self.cube = model_position_in_robot_frame() or self.cube
            time.sleep(0.1)

    def _print(self):
        if self.joints is None or self.cube is None:
            print("/joint_states と Gazebo のブロックの位置を待っています", flush=True)
            return
        try:
            q = tuple(self.joints[f"crane_plus_joint{k}"] for k in (1, 2, 3, 4))
        except KeyError:
            return
        hand = fk(q)
        dx, dy, dz = (c - h for c, h in zip(self.cube, (hand.x, hand.y, hand.z), strict=True))
        grip = self.joints.get(GRIPPER_JOINT_NAME, 0.0)
        print(
            f"手先→ブロック  前後 {100 * dx:+5.1f}cm  左右 {100 * dy:+5.1f}cm  上下 {100 * dz:+5.1f}cm"
            f"  距離 {100 * math.dist((0, 0, 0), (dx, dy, dz)):4.1f}cm"
            f" | 手先の傾き {math.degrees(hand.pitch):4.0f}deg（180=真下）"
            f" | グリッパー {math.degrees(grip):+4.0f}deg"
            f" | ブロックの高さ {100 * self.cube[2]:+5.1f}cm",
            flush=True,
        )


def main():
    rclpy.init()
    node = GraspMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
