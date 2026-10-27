"""手先から目標点までの位置の差を表示する（テレオペの補助）。

差は手先空間テレオペのキーと同じ、根元を中心にした向きで出す: 根元のまわりに回る（A/D）・
根元から遠く / 近く（W/S）・上 / 下（R/F）。
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
            "回る＝根元のまわり（A/D）、遠く・近く＝今の向きのまま根元から（W/S）、上下（R/F）。"
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
        hand = hand_position(self.q)
        dist = math.dist(self.target, hand)
        dth, arc, dr, dz = cylindrical_diff(self.target, hand)
        line = (
            f"目標点は  回る {_turn(dth, arc)} / {_dir(dr, '遠く', '近く', 'W', 'S')}"
            f" / {_dir(dz, '上', '下', 'R', 'F')}   （距離 {100 * dist:4.1f}cm）"
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


def cylindrical_diff(target, hand):
    """手先から目標点への差を、根元を中心にした向きで返す。

    (根元のまわりの角度 [rad、左 = 正], その角度ぶん手先が円に沿って動く長さ [m],
    根元からの距離の差 [m、遠く = 正], 高さの差 [m、上 = 正])
    """
    r_t, r_h = math.hypot(target[0], target[1]), math.hypot(hand[0], hand[1])
    dth = math.atan2(target[1], target[0]) - math.atan2(hand[1], hand[0])
    dth = (dth + math.pi) % (2 * math.pi) - math.pi
    return dth, r_h * dth, r_t - r_h, target[2] - hand[2]


def _turn(dth, arc):
    """角度の差を「右に 7.0°（1.2cm、D）」のような言葉にする。"""
    if abs(arc) < 0.0005:
        return "左右 0.0°"
    side, key = ("左", "A") if dth > 0 else ("右", "D")
    return f"{side}に {math.degrees(abs(dth)):4.1f}°（{100 * abs(arc):4.1f}cm、{key}）"


def _dir(d, plus, minus, key_plus, key_minus):
    """差 d [m] を「遠くに 1.2cm（W）」のような言葉にする。"""
    if abs(d) < 0.0005:
        return f"{plus}{minus} 0.0cm"
    if d > 0:
        return f"{plus}に {100 * d:4.1f}cm（{key_plus}）"
    return f"{minus}に {100 * -d:4.1f}cm（{key_minus}）"


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
