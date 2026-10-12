"""CRANE+ V2 の手先空間キーボードテレオペ。

キーで手先の位置と傾きを動かし、逆運動学で 4 関節の目標角度に変換して送る。
送るトピック・形式・周期は physicalai_teleop（関節空間テレオペ）と同じにしてある。
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

from crane_ee_teleop.kinematics import L2, L3, SHOULDER_HEIGHT, TOOL_LENGTH, Pose, fk, ik

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

POS_STEP = 0.005  # [m]  1 回のキーで手先を動かす距離
PITCH_STEP = 0.05  # [rad] 1 回のキーで手先を傾ける角度
MAX_JOINT_JUMP = 0.5  # [rad] 1 回のキーでこれ以上動く関節があれば、特異点の近くとみなして無視する
PITCH_SEARCH_STEPS = (
    12  # 位置だけを動かして届かないとき、傾きを PITCH_STEP 刻みで最大 ±0.6 rad まで変えて探す
)

HELP = """
Teleop CRANE+ V2 (Hand-space control):
---------------------------------------
W/S: hand forward / backward (+x / -x)
A/D: hand left / right       (+y / -y)
R/F: hand up / down          (+z / -z)
T/G: tilt hand down / up     (pitch + / -)
V:   point the hand straight down as far as reachable
     (W/S/A/D/R/F also tilt the hand toward straight-down little by little)
O:   Open gripper
C:   Close gripper
X:   Reset robot to vertical (all-zero) position
     (vertical = arm fully stretched: press F to lower the hand before W/A/D)
ESC: Quit
"""

KEY_MOVES = {
    "w": (POS_STEP, 0.0, 0.0, 0.0),
    "s": (-POS_STEP, 0.0, 0.0, 0.0),
    "a": (0.0, POS_STEP, 0.0, 0.0),
    "d": (0.0, -POS_STEP, 0.0, 0.0),
    "r": (0.0, 0.0, POS_STEP, 0.0),
    "f": (0.0, 0.0, -POS_STEP, 0.0),
    "t": (0.0, 0.0, 0.0, PITCH_STEP),
    "g": (0.0, 0.0, 0.0, -PITCH_STEP),
}


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
        self.pose = None
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
            self.q, self.pose = q, fk(q)
            self.gripper_pos = pos.get(GRIPPER_JOINT_NAME, GRIPPER_OPEN)
        self._print_status("start")

    def _print_status(self, note=""):
        p = self.pose
        print(
            f"hand x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} pitch={math.degrees(p.pitch):+6.1f}deg"
            + (f"  [{note}]" if note else ""),
            flush=True,
        )

    def _solve(self, target):
        """target を実現する関節角。届かない・関節が飛ぶなら None。"""
        q = ik(target, q_ref=self.q)
        if q is None:
            return None, "unreachable"
        if max(abs(a - b) for a, b in zip(q, self.q, strict=True)) > MAX_JOINT_JUMP:
            return None, "jump"
        return q, ""

    def _stretched(self):
        """腕がほぼ伸びきっているか（肩から手先までの距離が、届く最大距離の 98% 以上）。"""
        p = self.pose
        d = math.hypot(math.hypot(p.x, p.y), p.z - SHOULDER_HEIGHT)
        return d > 0.98 * (L2 + L3 + TOOL_LENGTH)

    def _move(self, dx, dy, dz, dpitch):
        p = self.pose
        if dpitch != 0.0:
            # T/G: 傾きだけを変える
            candidates = [(p.pitch + dpitch, "")]
        else:
            # 位置を優先する。まず傾きを真下向き (pi) へ 1 段近づけた姿勢を試し、だめなら今の傾き、
            # それでも届かなければ傾きを少ない順に変えて探す（直立は腕が伸びきっているので、
            # 傾きを保ったままでは前へ出られない）。つかむ動作では真下向きが要るため、
            # 動かしているうちに手先が自然に下を向くようにしている
            toward_down = p.pitch + max(-PITCH_STEP, min(PITCH_STEP, math.pi - p.pitch))
            candidates = [(toward_down, ""), (p.pitch, "")]
            for k in range(1, PITCH_SEARCH_STEPS + 1):
                for sign in (1.0, -1.0):
                    candidates.append(
                        (
                            p.pitch + sign * k * PITCH_STEP,
                            f"傾きを自動で {math.degrees(sign * k * PITCH_STEP):+.0f}deg 変更",
                        )
                    )
        q, why, target, note = None, "unreachable", None, ""
        for pitch, n in candidates:
            t = Pose(p.x + dx, p.y + dy, p.z + dz, pitch)
            sol, w = self._solve(t)
            if sol is not None:
                q, target, note = sol, t, n
                break
            if w == "jump":
                why = "jump"
        if q is None:
            if why == "jump":
                self._print_status("特異点の近く（真上付近）。先に W で前へ出してください")
            elif self._stretched():
                self._print_status("腕が伸びきっています。まず F で手先を下げてください")
            else:
                self._print_status("届かない・可動範囲外")
            return
        self.q, self.pose = q, target
        self._print_status(note)

    def _point_down(self):
        """V: 今の手先の位置のまま、届く範囲で最も真下向きに近い傾きにする。"""
        p = self.pose
        best = None
        for k in range(0, 64):
            pitch = p.pitch + (k / 63) * (math.pi - p.pitch)  # 今の傾きから真下向きまでを細かく試す
            sol, _ = self._solve(Pose(p.x, p.y, p.z, pitch))
            if sol is not None:
                best = (sol, pitch)
        if best is None:
            self._print_status("この位置では傾きを変えられません")
            return
        self.q, self.pose = best[0], Pose(p.x, p.y, p.z, best[1])
        self._print_status("真下向きに近づけました")

    def _keyboard_loop(self):
        with KeyboardInput() as kb:
            while self.running and rclpy.ok():
                key = kb.get_key()
                if key is None:
                    continue
                if ord(key) == 27:  # ESC
                    self.running = False
                    break
                with self.lock:
                    if self.q is None:
                        print("/joint_states を待っています", flush=True)
                        continue
                    k = key.lower()
                    if k in KEY_MOVES:
                        self._move(*KEY_MOVES[k])
                    elif k == "v":
                        self._point_down()
                    elif k == "o":
                        self.gripper_pos = GRIPPER_OPEN
                        print("[INFO] Gripper: OPEN", flush=True)
                    elif k == "c":
                        self.gripper_pos = GRIPPER_CLOSE
                        print("[INFO] Gripper: CLOSE", flush=True)
                    elif k == "x":
                        self.q = (0.0, 0.0, 0.0, 0.0)
                        self.pose = fk(self.q)
                        self.gripper_pos = GRIPPER_OPEN
                        self._print_status("reset to vertical")

    def _publish(self):
        with self.lock:
            if self.q is None:
                return
            arm, gripper = to_arm_order(self.q), self.gripper_pos
        for pub, names, positions in (
            (self.arm_pub, ARM_JOINT_NAMES, arm),
            (self.gripper_pub, [GRIPPER_JOINT_NAME], [gripper]),
        ):
            jt = JointTrajectory()
            jt.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in positions]
            pt.time_from_start.nanosec = int(0.1 * 1e9)  # physicalai_teleop と同じ
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
