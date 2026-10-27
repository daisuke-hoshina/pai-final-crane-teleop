#!/usr/bin/env python3
# 出典: https://github.com/matsuolab/pai_training（commit b4d6b54、Physical AI 応用編1 の演習用リポジトリ）の scripts/infer.py を写して変えたもの。
# 講義版との違いは「予測した指令の列（chunk）をどう使うか」と、下のオプション（どれも付けなければ講義版と同じ動き）。
#   学習設定の action の delta_timestamps は [-0.1, 0.0, 0.1, ..., 1.4] 秒なので、chunk の i 番目は (i - 1) * 0.1 秒後の指令。
#   講義版（select_action, n_action_steps 1）は毎ティック chunk[0]（= 0.1 秒前の指令）を出す。
#   --index I : chunk の I 番目から使う（0 = 講義版と同じ、1 = 今の時刻）
#   --exec K  : 1 回の予測で I〜I+K-1 番目の K 個を順に送り、送り終えたら予測し直す（1 = 毎ティック予測し直す、講義版と同じ）
#   --hold H  : 各指令を H ティック（1 ティック 0.05 秒）ずつ送る。chunk の刻み 0.1 秒に合わせるなら 2
#   既定値（0, 1, 1）で講義版と同じ動きになる
#   --mask-gripper V : 観測のグリッパーを V に置き換える（G8: 学習データの観測のグリッパーを 0.0 にしたモデル用）。付けなければ講義版と同じ
#   --rule-grip : グリッパーだけルールで閉じる（R）。手先（FK）の高さが --grip-z 未満で、直近 --grip-still-sec 秒の手先の移動が
#                 --grip-still-m 未満になったら、GRIPPER_CLOSE を time_from_start GRIP_TFS で送り、以後は閉じたまま。
#                 閉じる前のグリッパーと腕は方策の指令のまま。値はテレオペ（teleop_keyboard_ee.py）と同じ
#   --after-grip I,K,H : --rule-grip で閉じた瞬間に、chunk の使い方を index I / exec K / hold H に切り替える（積んである指令は捨てる）
#   --vel-env : observation.environment_state に、関節角の複製ではなく「0.1 秒前からの関節角の変化」を入れる（G15: 変換
#               convert_v2_train_trimstill_vel と同じ量）。0.1 秒前は、毎ティック記録した関節角のうち時刻がいちばん近いもの。最初の 0.1 秒は 0
#   --ensemble C : 時間的アンサンブル（G18、ACT の論文の推論）。hold ティックごと（hold 2 で 0.1 秒ごと）に予測し、過去の予測のうち
#                  「今」に当たる要素（j 回前の予測なら chunk[index + j]）を重み exp(-C * k) で平均して送る（k = 0 がいちばん古い予測、
#                  ACT・LeRobot と同じ向き）。覚えておく予測は chunk_size - index 個まで。C は LeRobot の temporal_ensemble_coeff の
#                  既定 0.01。--exec は使わない
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import torch
import argparse
import numpy as np

from collections import deque
import importlib.util
import sys
import time

# 手先の位置は提出用リポジトリの kinematics.py（ROS に依存しない）で計算する
_spec = importlib.util.spec_from_file_location(
    "crane_kinematics", "/workspace/pai-final-crane-teleop/crane_ee_teleop/crane_ee_teleop/kinematics.py")
kinematics = importlib.util.module_from_spec(_spec); sys.modules["crane_kinematics"] = kinematics; _spec.loader.exec_module(kinematics)
GRIPPER_CLOSE = 0.1745  # teleop_keyboard_ee.py と同じ（10 deg）
GRIP_TFS = 0.01  # teleop_keyboard_ee.py の GRIPPER_TIME_FROM_START と同じ。0.1 では握る力が出ない（notes 10/13）

from lerobot.common.policies.act.modeling_act import ACTPolicy

ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'

class InferenceNode(Node):
    def __init__(self, model_path, index=0, exec_steps=1, hold=1, mask_gripper=None, rule_grip=None, vel_env=False, ensemble=None):
        super().__init__('crane_plus_act_infer')
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.device = torch.device('cpu')
        self.policy.to(self.device)
        self.last_action = np.zeros(5, dtype=np.float32)
        self.timer = self.create_timer(0.05, self.publish_action)  # 20Hz
        self.latest_joint_state = None
        self.index, self.exec_steps, self.hold = index, exec_steps, hold
        self.mask_gripper = mask_gripper
        self.rule_grip = rule_grip  # None または (grip_z, still_sec, still_m)
        self.hand_hist = deque()  # (時刻, 手先の位置)
        self.grip_closed = False
        self.after_grip = None  # main() で設定
        self.queue = deque()
        self.vel_env = vel_env
        self.state_hist = deque()  # (時刻, 関節角)。--vel-env 用
        self.ensemble = ensemble
        self.chunks = deque(maxlen=self.policy.config.chunk_size - index)  # 過去の予測（古い順）。--ensemble 用

    def joint_state_callback(self, msg):
        # Extract joint positions in the order of ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        name_to_pos = dict(zip(msg.name, msg.position))
        arm_pos = [name_to_pos.get(j, 0.0) for j in ARM_JOINT_NAMES]
        gripper_pos = name_to_pos.get(GRIPPER_JOINT_NAME, 0.0)
        self.latest_joint_state = np.array(arm_pos + [gripper_pos], dtype=np.float32)

    def publish_action(self):
        if self.latest_joint_state is None:
            return
        state = self.latest_joint_state.copy()
        if self.mask_gripper is not None:
            state[4] = self.mask_gripper
        # Prepare observation dict for ACTPolicy
        obs = {
            'observation.state': torch.tensor(state, dtype=torch.float32).unsqueeze(0),  # (1, 5)
            'observation.environment_state': torch.tensor(
                self.velocity(state) if self.vel_env else state, dtype=torch.float32).unsqueeze(0),  # (1, 5)
        }
        if self.vel_env:
            print("vel:", np.round(obs['observation.environment_state'][0].numpy(), 4), flush=True)
        if not self.queue and self.ensemble is not None:
            self.queue.extend([self.ensemble_action(obs)] * self.hold)
        elif not self.queue:
            chunk = self.predict_chunk(obs)  # (chunk_size, 5)
            for a in chunk[self.index:self.index + self.exec_steps]:
                self.queue.extend([a] * self.hold)
        action = self.queue.popleft()
        grip_cmd, grip_tfs = float(action[4]), 0.1
        if self.rule_grip is not None:
            grip_cmd, grip_tfs = self.apply_rule_grip(grip_cmd)
        self.last_action = action
        # Publish to arm and gripper
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = action[:4].tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        jt_g = JointTrajectory()
        jt_g.joint_names = [GRIPPER_JOINT_NAME]
        pt_g = JointTrajectoryPoint()
        pt_g.positions = [grip_cmd]
        pt_g.time_from_start.sec = 0
        pt_g.time_from_start.nanosec = int(grip_tfs * 1e9)
        jt_g.points.append(pt_g)
        self.gripper_pub.publish(jt_g)
        print("action:", action[:4], "gripper:", grip_cmd)

def _predict_chunk(self, obs):
    # ACTPolicy.select_action の中身と同じ計算（正規化 → model → 逆正規化）で、chunk 全体を返す
    with torch.no_grad():
        batch = self.policy.normalize_inputs(obs)
        actions = self.policy.model(batch)[0]  # (1, chunk_size, 5)
        actions = self.policy.unnormalize_outputs({"action": actions})["action"]
    return actions[0].cpu().numpy()


InferenceNode.predict_chunk = _predict_chunk


def _ensemble_action(self, obs):
    # 新しく予測し、覚えている予測それぞれの「今」に当たる要素を平均する
    self.chunks.append(self.predict_chunk(obs))
    n = len(self.chunks)
    acts = np.stack([c[self.index + n - 1 - k] for k, c in enumerate(self.chunks)])  # k 番目は n-1-k 回前の予測
    w = np.exp(-self.ensemble * np.arange(n))
    return (w[:, None] * acts).sum(0) / w.sum()


InferenceNode.ensemble_action = _ensemble_action


VEL_LAG_SEC = 0.1  # 変換の VEL_LAG（2 フレーム、20Hz）と同じ


def _velocity(self, state):
    # 0.1 秒前からの関節角の変化。毎ティック（20Hz）記録し、0.1 秒前に時刻がいちばん近い記録との差を取る
    now = time.time()
    self.state_hist.append((now, state.copy()))
    while len(self.state_hist) > 2 and now - self.state_hist[1][0] >= VEL_LAG_SEC:
        self.state_hist.popleft()
    t_old, s_old = min(self.state_hist, key=lambda h: abs(now - h[0] - VEL_LAG_SEC))
    if now - t_old < 0.5 * VEL_LAG_SEC:
        return np.zeros_like(state)
    return state - s_old


InferenceNode.velocity = _velocity


def _apply_rule_grip(self, grip_cmd):
    # R: 手先がつかむ高さで止まったら閉じる。一度閉じたら閉じたまま
    if self.grip_closed:
        return GRIPPER_CLOSE, GRIP_TFS
    grip_z, still_sec, still_m = self.rule_grip
    s = self.latest_joint_state  # [joint1, joint2, joint4, joint3, gripper]
    p = kinematics.fk((float(s[0]), float(s[1]), float(s[3]), float(s[2])))
    now = time.time()
    self.hand_hist.append((now, np.array([p.x, p.y, p.z])))
    while self.hand_hist and now - self.hand_hist[0][0] > still_sec:
        self.hand_hist.popleft()
    span = now - self.hand_hist[0][0]
    moved = max(np.linalg.norm(h - self.hand_hist[-1][1]) for _, h in self.hand_hist)
    if p.z < grip_z and span > 0.9 * still_sec and moved < still_m:
        self.grip_closed = True
        if self.after_grip is not None:
            self.index, self.exec_steps, self.hold = self.after_grip
            self.queue.clear()
            print(f"rule_grip: chunk の使い方を index {self.index} / exec {self.exec_steps} / hold {self.hold} に切り替える", flush=True)
        print(f"rule_grip: 閉じる（手先 {np.round(np.array([p.x, p.y, p.z]) * 100, 1).tolist()}cm、直近 {span:.1f} 秒の移動 {moved * 1000:.1f}mm）", flush=True)
        return GRIPPER_CLOSE, GRIP_TFS
    return grip_cmd, 0.1


InferenceNode.apply_rule_grip = _apply_rule_grip


def main():
    parser = argparse.ArgumentParser(description='ACTPolicy inference node for CRANE+ V2 (ROS2)')
    parser.add_argument('--model-path', type=str, required=True, help='Path to trained ACTPolicy model directory')
    parser.add_argument('--index', type=int, default=0)
    parser.add_argument('--exec', dest='exec_steps', type=int, default=1)
    parser.add_argument('--hold', type=int, default=1)
    parser.add_argument('--mask-gripper', type=float, default=None)
    parser.add_argument('--rule-grip', action='store_true')
    parser.add_argument('--grip-z', type=float, default=0.045)
    parser.add_argument('--grip-still-sec', type=float, default=1.0)
    parser.add_argument('--grip-still-m', type=float, default=0.003)
    parser.add_argument('--after-grip', type=str, default=None)
    parser.add_argument('--vel-env', action='store_true')
    parser.add_argument('--ensemble', type=float, default=None)
    args = parser.parse_args()
    print(f"chunk の使い方: index {args.index} / exec {args.exec_steps} / hold {args.hold} / 観測のグリッパー {'そのまま' if args.mask_gripper is None else args.mask_gripper}", flush=True)
    rclpy.init()
    rule = (args.grip_z, args.grip_still_sec, args.grip_still_m) if args.rule_grip else None
    if rule:
        print(f"rule_grip: 手先 z < {args.grip_z} m で、直近 {args.grip_still_sec} 秒の移動 < {args.grip_still_m} m なら閉じる", flush=True)
    node = InferenceNode(args.model_path, args.index, args.exec_steps, args.hold, args.mask_gripper, rule, args.vel_env, args.ensemble)
    if args.ensemble is not None:
        print(f"時間的アンサンブル: {args.hold} ティックごとに予測し、最大 {node.chunks.maxlen} 個の予測を重み exp(-{args.ensemble} k) で平均", flush=True)
    print(f"environment_state: {'0.1 秒前からの関節角の変化' if args.vel_env else '関節角（講義版と同じ）'}", flush=True)
    if args.after_grip:
        node.after_grip = tuple(int(v) for v in args.after_grip.split(','))
        print(f"rule_grip: 閉じたら chunk の使い方を {node.after_grip} に切り替える", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
