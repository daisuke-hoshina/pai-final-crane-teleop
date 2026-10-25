"""手先から目標点までの位置の差を表示する（テレオペの補助）。

指令は何も送らないので、関節空間テレオペ・手先空間テレオペのどちらとも一緒に使える。

  ros2 run crane_ee_teleop task_monitor                 # grasp: 目標点はつかむ位置。浮き上がりも出す
  ros2 run crane_ee_teleop task_monitor --task reach    # reach: 目標点はブロックの真上
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from crane_ee_teleop.task import (
    LIFT_SUCCESS,
    SUCCESS_TOL,
    TASKS,
    arm_q,
    cube_lift,
    hand_position,
    target_position,
)

PRINT_PERIOD = 0.3  # [s]


class TaskMonitor(Node):
    def __init__(self, task):
        super().__init__("crane_plus_task_monitor")
        self.task = task
        where = "つかむ位置＝ブロックの中心の 1cm 上" if task == "grasp" else "ブロックの真上"
        print(
            f"目標点（{where}）が手先から見てどこにあるかを表示します。"
            "前＝ロボットの正面、左右＝ロボットから見た左右。"
        )
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.q = None
        self.target = None
        self.lift = None
        self.running = True
        threading.Thread(target=self._read_target_loop, daemon=True).start()
        self.create_timer(PRINT_PERIOD, self._print)

    def _on_state(self, msg):
        try:
            self.q = arm_q(msg.name, msg.position)
        except KeyError:
            pass

    def _read_target_loop(self):
        while self.running:
            self.target = target_position(self.task) or self.target
            if self.task == "grasp":
                self.lift = cube_lift()
            time.sleep(0.2)

    def _print(self):
        if self.q is None or self.target is None:
            print("\r\033[K/joint_states とブロックの位置を待っています", end="", flush=True)
            return
        d = [t - h for t, h in zip(self.target, hand_position(self.q), strict=True)]
        dist = math.dist((0, 0, 0), d)
        line = (
            f"目標点は手先から見て  {_dir(d[0], '前', '後ろ')} / {_dir(d[1], '左', '右')}"
            f" / {_dir(d[2], '上', '下')}   （距離 {100 * dist:4.1f}cm）"
        )
        if self.task == "grasp":
            lift = self.lift or 0.0
            line += f"   ブロックの浮き {100 * lift:4.1f}cm"
            if lift >= LIFT_SUCCESS:
                line += "   ★ 持ち上げ成功"
            elif dist < SUCCESS_TOL:
                line += "   ★ つかむ位置"
        elif dist < SUCCESS_TOL:
            line += "   ★ 目標に到達"
        print("\r\033[K" + line, end="", flush=True)


def _dir(d, plus, minus):
    """差 d [m] を「前に 1.2cm」のような言葉にする。"""
    if abs(d) < 0.0005:
        return f"{plus}{minus} 0.0cm"
    return f"{plus if d > 0 else minus}に {100 * abs(d):4.1f}cm"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=TASKS, default="grasp")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = TaskMonitor(args.task)
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
