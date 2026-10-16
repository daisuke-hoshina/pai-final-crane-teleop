"""デモを 1 エピソード記録する。

  ros2 run crane_ee_teleop record_episode --method ee      # 手先空間テレオペのデモ
  ros2 run crane_ee_teleop record_episode --method joint   # 関節空間テレオペのデモ

1. アームがホームで止まっているか確かめる（止まっていなければ記録しない）
2. ~/dataset/rosbags/reach_<method>/epN に、変換に使うトピックだけを記録する（N は自動で決める）
3. Ctrl+C で止める
4. 長さ・最後の手先と目標点の距離・成功したかを表示する
"""

import argparse
import glob
import os
import signal
import sqlite3
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from crane_ee_teleop.task import (
    SUCCESS_TOL,
    arm_q,
    at_home,
    distance_to_target,
    target_position,
)

TOPICS = [
    "/joint_states",
    "/crane_plus_arm_controller/joint_trajectory",
    "/crane_plus_gripper_controller/joint_trajectory",
]


def wait_until(cond, timeout, node):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cond():
            return True
    return False


def summarize(bag_dir, target):
    dbs = glob.glob(f"{bag_dir}/*.db3")
    if not dbs:
        print(f"!! {bag_dir} に記録がありません。撮り直してください")
        return
    c = sqlite3.connect(dbs[0])
    row = c.execute("select id from topics where name=?", ("/joint_states",)).fetchone()
    rows = []
    if row:
        rows = c.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (row[0],)
        ).fetchall()
    if not rows:
        print(f"!! {bag_dir} に /joint_states がありません。撮り直してください")
        return
    first = deserialize_message(rows[0][1], JointState)
    last = deserialize_message(rows[-1][1], JointState)
    dur = (rows[-1][0] - rows[0][0]) / 1e9
    d = distance_to_target(arm_q(last.name, last.position), target)
    started_home = at_home(arm_q(first.name, first.position))
    ok = d < SUCCESS_TOL and started_home
    print(
        f"===== {os.path.basename(bag_dir)}: 長さ {dur:.1f} 秒 / 最後の手先と目標点の距離 {100 * d:.1f}cm"
        f" / {'成功' if ok else '失敗'}"
        + ("" if started_home else "（ホームから始まっていません）")
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["ee", "joint"], required=True)
    ap.add_argument("--root", default=os.path.expanduser("~/dataset/rosbags"))
    args = ap.parse_args()
    out_dir = os.path.join(args.root, f"reach_{args.method}")
    os.makedirs(out_dir, exist_ok=True)

    # Ctrl+C は rclpy に処理させず、記録を止める合図として自分で受ける
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("record_episode")
    state = {}
    node.create_subscription(JointState, "/joint_states", lambda m: state.update(msg=m), 10)
    if not wait_until(lambda: "msg" in state, 5, node):
        print("!! /joint_states が届きません。Gazebo は起動していますか？")
        return
    m = state["msg"]
    vel = [abs(v) for n, v in zip(m.name, m.velocity, strict=True) if "hand" not in n]
    if not at_home(arm_q(m.name, m.position), vel or None):
        print(
            "!! アームがホームで止まっていません。テレオペで X を押し、止まってから実行してください"
        )
        return
    target = target_position()
    if target is None:
        print("!! Gazebo からブロックの位置を読めません")
        return

    n = 0
    while os.path.exists(f"{out_dir}/ep{n}"):
        n += 1
    bag_dir = f"{out_dir}/ep{n}"
    base = node.count_subscribers("/joint_states")
    bag = subprocess.Popen(
        ["ros2", "bag", "record", "-o", bag_dir, "--log-level", "warn", *TOPICS],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,  # Ctrl+C はこのスクリプトが受け、レコーダーへは明示的に送る
    )
    try:
        # レコーダーが /joint_states を購読し始めるまで待つ（起動に数秒かかることがある）
        if not wait_until(lambda: node.count_subscribers("/joint_states") > base, 20, node):
            print("!! 記録が始まりません")
            return
        print(f"===== ep{n} を記録中。操作してください。終わったら Ctrl+C =====", flush=True)
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.try_shutdown()
    summarize(bag_dir, target)


if __name__ == "__main__":
    main()
