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
GRASP_TOL = 0.01  # [m] 前後・左右・上下がすべてこれ以内なら「つかめる位置」と表示する


class GraspMonitor(Node):
    def __init__(self):
        super().__init__("crane_plus_grasp_monitor")
        print(
            "手先からブロックまでの位置の差を表示します。前後・左右・上下はロボットの向きが基準"
            "（前＝ロボットの正面、左右＝ロボットから見た左右）。手先は指の間の点。"
        )
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
            print(
                "\r\033[K/joint_states と Gazebo のブロックの位置を待っています", end="", flush=True
            )
            return
        try:
            q = tuple(self.joints[f"crane_plus_joint{k}"] for k in (1, 2, 3, 4))
        except KeyError:
            return
        hand = fk(q)
        # ロボット基準（+x が正面、+y が左、+z が上）で、手先からブロックまでの差
        dx, dy, dz = (c - h for c, h in zip(self.cube, (hand.x, hand.y, hand.z), strict=True))
        tilt_from_down = abs(math.degrees(math.pi - hand.pitch))
        grip = "閉" if self.joints.get(GRIPPER_JOINT_NAME, 0.0) > 0.0 else "開"
        ok = max(abs(dx), abs(dy), abs(dz)) < GRASP_TOL
        print(
            "\r\033[K"
            f"ブロックは手先から見て  {_dir(dx, '前', '後ろ')} / {_dir(dy, '左', '右')} / {_dir(dz, '上', '下')}"
            f"  （距離 {100 * math.dist((0, 0, 0), (dx, dy, dz)):.1f}cm）"
            f"   手先の向き: 真下から {tilt_from_down:.0f}°   グリッパー: {grip}"
            + ("   ← つかめる位置です" if ok else ""),
            end="",
            flush=True,
        )


def _dir(d, plus, minus):
    """差 d [m] を「前に 1.2cm」のような言葉にする。"""
    if abs(d) < 0.0005:
        return f"{plus}{minus}は一致"
    return f"{plus if d > 0 else minus}に {100 * abs(d):4.1f}cm"


def main():
    rclpy.init()
    node = GraspMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
