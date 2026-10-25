"""CRANE+ V2 の手先空間キーボードテレオペ。

キーで手先の位置（前後・左右・上下）だけを動かす。手先の向きは操作せず、いつも
「その位置で届く範囲でいちばん真下に近い向き」に自動で決める（上からつかむため）。
逆運動学で 4 関節の目標角度に変換して送る。

送るトピック・形式・周期（20Hz）は physicalai_teleop（関節空間テレオペ）と同じ。
記録・変換・学習・推論の流れをそのまま使い、比較実験で変える条件をテレオペ方式だけにするため。

  ros2 run crane_ee_teleop teleop_keyboard_ee
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from crane_ee_teleop.gazebo_objects import reset_cube
from crane_ee_teleop.kinematics import Pose, fk, ik

# physicalai_teleop と同じ並び・値（関節名の並びは joint1, 2, 4, 3）
ARM_JOINT_NAMES = [
    "crane_plus_joint1",
    "crane_plus_joint2",
    "crane_plus_joint4",
    "crane_plus_joint3",
]
GRIPPER_JOINT_NAME = "crane_plus_joint_hand"
GRIPPER_OPEN = -0.5236  # -30 deg
GRIPPER_CLOSE = 0.1745  # 10 deg
PUBLISH_PERIOD = 0.05  # 20Hz
ARM_TIME_FROM_START = 0.1  # [s] physicalai_teleop と同じ
# [s] グリッパーは短くする。20Hz で送り直すたびに軌道が今の角度から作り直されるので、0.1 秒だと目標が
# 今の角度のすぐ先にしかならず、ブロックに当たると弱い力でしか握れない（teleop_keyboard_joint も同じ値）
GRIPPER_TIME_FROM_START = 0.01

POS_STEP = 0.005  # [m] 1 回のキーで手先を動かす距離
# X で戻るホーム（机の上 10cm、手先はほぼ真下向き）。直立は腕が伸びきって前へ出られず、
# 真上では横に動かすと根元が大きく回るので、ホームにしない。teleop_keyboard_joint も同じ姿勢に戻る
HOME_POSITION = (0.10, 0.0, 0.10)
# [m] 手先をこれより下げない。手先点は指先より 2〜3cm 上にあり、3.1cm で指先が机に着く。
# 着いたまま下げ続けると、指令だけが机の下へ進み、腕が机の上を滑って外へ伸びる
MIN_HAND_Z = 0.032
MAX_JOINT_JUMP = 0.5  # [rad] 1 回のキーでこれ以上動く関節があれば、その動きは受け付けない
PITCH_CANDIDATES = [math.pi - 0.05 * k for k in range(0, 40)]  # 真下向きから順に試す向き

HELP = """
Teleop CRANE+ V2 (Hand-space control)
-------------------------------------
W/S: hand forward / backward
A/D: hand left / right
R/F: hand up / down
O/C: open / close gripper
X:   go to home (10 cm above the table, hand pointing down)
Z:   put the block back to its start position (before recording only)
ESC: quit
The hand always points as straight down as it can.
The hand stops 3.2 cm above the table (the fingertips touch the table below that).
"""

KEY_MOVES = {
    "w": (POS_STEP, 0.0, 0.0),
    "s": (-POS_STEP, 0.0, 0.0),
    "a": (0.0, POS_STEP, 0.0),
    "d": (0.0, -POS_STEP, 0.0),
    "r": (0.0, 0.0, POS_STEP),
    "f": (0.0, 0.0, -POS_STEP),
}


def solve_pointing_down(x, y, z, q_ref=None):
    """(x, y, z) に手先を置き、届く範囲でいちばん真下に近い向きにする関節角。届かなければ None。"""
    for pitch in PITCH_CANDIDATES:
        q = ik(Pose(x, y, z, pitch), q_ref=q_ref)
        if q is not None:
            return q
    return None


HOME_Q = solve_pointing_down(*HOME_POSITION)


class KeyboardInput:
    def __enter__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self, timeout=0.1):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None


def to_arm_order(q):
    """kinematics の (q1, q2, q3, q4) を ARM_JOINT_NAMES の並び (j1, j2, j4, j3) にする。"""
    q1, q2, q3, q4 = q
    return [q1, q2, q4, q3]


class EETeleopNode(Node):
    def __init__(self):
        super().__init__("crane_plus_ee_teleop")
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.lock = threading.Lock()
        self.q = None  # 指令中の関節角 (q1, q2, q3, q4)。最初の /joint_states で初期化する
        self.target = None  # 指令中の手先の位置 (x, y, z)
        self.gripper_pos = GRIPPER_OPEN
        self.running = True
        print(HELP)
        self.create_timer(PUBLISH_PERIOD, self._publish)
        self.thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.thread.start()

    def _on_state(self, msg):
        if self.q is not None:
            return
        pos = dict(zip(msg.name, msg.position, strict=True))
        try:
            q = tuple(pos[f"crane_plus_joint{k}"] for k in (1, 2, 3, 4))
        except KeyError:
            return
        with self.lock:
            p = fk(q)
            self.q, self.target = q, (p.x, p.y, p.z)
            self.gripper_pos = pos.get(GRIPPER_JOINT_NAME, GRIPPER_OPEN)
        self._print_status("start. Press X to go home")

    def _print_status(self, note=""):
        x, y, z = self.target
        print(
            f"hand x={100 * x:+5.1f}cm y={100 * y:+5.1f}cm z={100 * z:+5.1f}cm"
            + (f"  [{note}]" if note else ""),
            flush=True,
        )

    def _move(self, dx, dy, dz):
        x, y, z = self.target
        t = (x + dx, y + dy, z + dz)
        if t[2] < MIN_HAND_Z:
            self._print_status("too low: the fingertips would hit the table")
            return
        q = solve_pointing_down(*t, q_ref=self.q)
        if q is None or max(abs(a - b) for a, b in zip(q, self.q, strict=True)) > MAX_JOINT_JUMP:
            self._print_status("cannot move there")
            return
        self.q, self.target = q, t
        self._print_status()

    def _keyboard_loop(self):
        with KeyboardInput() as kb:
            while self.running and rclpy.ok():
                key = kb.get_key()
                if key is None:
                    continue
                if ord(key) == 27:  # ESC
                    self.running = False
                    break
                if key.lower() == "z":
                    # Gazebo への問い合わせに 0.5 秒ほどかかるので、送信を止めないようロックの外で行う
                    ok = reset_cube()
                    print("block: back to start" if ok else "block: reset FAILED", flush=True)
                    continue
                with self.lock:
                    if self.q is None:
                        print("waiting for /joint_states", flush=True)
                        continue
                    k = key.lower()
                    if k in KEY_MOVES:
                        self._move(*KEY_MOVES[k])
                    elif k == "o":
                        self.gripper_pos = GRIPPER_OPEN
                        print("gripper: OPEN", flush=True)
                    elif k == "c":
                        self.gripper_pos = GRIPPER_CLOSE
                        print("gripper: CLOSE", flush=True)
                    elif k == "x":
                        self.q, self.target = HOME_Q, HOME_POSITION
                        self.gripper_pos = GRIPPER_OPEN
                        self._print_status("home")

    def _publish(self):
        with self.lock:
            if self.q is None:
                return
            arm, gripper = to_arm_order(self.q), self.gripper_pos
        for pub, names, positions, duration in (
            (self.arm_pub, ARM_JOINT_NAMES, arm, ARM_TIME_FROM_START),
            (self.gripper_pub, [GRIPPER_JOINT_NAME], [gripper], GRIPPER_TIME_FROM_START),
        ):
            jt = JointTrajectory()
            jt.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in positions]
            pt.time_from_start.nanosec = int(duration * 1e9)
            jt.points.append(pt)
            pub.publish(jt)


def main():
    rclpy.init()
    node = EETeleopNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
