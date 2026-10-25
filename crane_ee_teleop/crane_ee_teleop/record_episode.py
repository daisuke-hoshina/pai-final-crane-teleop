"""デモを 1 エピソード記録する。

  ros2 run crane_ee_teleop record_episode --method ee                # つかんで持ち上げるデモ（grasp）
  ros2 run crane_ee_teleop record_episode --method ee --task reach   # 目標点へ運ぶデモ（reach）

1. アームがホームで止まっているか確かめる（止まっていなければ記録しない）
2. ~/dataset/rosbags/<task>_<method>/epN に、変換に使うトピックだけを記録する（N は自動で決める）
3. Ctrl+C で止める
4. 長さ・最後の手先と目標点の距離（grasp では記録中のブロックの最大の浮き上がり）・成功したかを表示する
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
    LIFT_SUCCESS,
    SUCCESS_TOL,
    TASKS,
    arm_q,
    at_home,
    cube_lift,
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


def summarize(bag_dir, target, lift=None):
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
    if lift is None:
        ok = d < SUCCESS_TOL and started_home
        result = f"最後の手先と目標点の距離 {100 * d:.1f}cm"
    else:
        # grasp の目標点（つかむ位置）は持ち上げると離れるので、距離は出さない
        ok = lift >= LIFT_SUCCESS and started_home
        result = f"記録中のブロックの最大の浮き {100 * lift:.1f}cm"
    print(
        f"===== {os.path.basename(bag_dir)}: 長さ {dur:.1f} 秒 / {result}"
        f" / {'成功' if ok else '失敗'}"
        + ("" if started_home else "（ホームから始まっていません）")
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["ee", "joint"], required=True)
    ap.add_argument("--task", choices=TASKS, default="grasp")
    ap.add_argument("--root", default=os.path.expanduser("~/dataset/rosbags"))
    args = ap.parse_args()
    out_dir = os.path.join(args.root, f"{args.task}_{args.method}")
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
    target = target_position(args.task)
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
    max_lift = None
    try:
        # レコーダーが /joint_states を購読し始めるまで待つ（起動に数秒かかることがある）
        if not wait_until(lambda: node.count_subscribers("/joint_states") > base, 20, node):
            print("!! 記録が始まりません")
            return
        print(f"===== ep{n} を記録中。操作してください。終わったら Ctrl+C =====", flush=True)
        while True:
            if args.task == "grasp":
                # Gazebo では持ち上げたブロックが数秒で滑り落ちるので、記録中の最大の浮きで判定する。
                # 1 回の読み取りに 0.3 秒ほどかかり、それより短い浮きは見逃すことがある
                h = cube_lift()
                if h is not None:
                    max_lift = h if max_lift is None else max(max_lift, h)
            else:
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
    lift = None
    if args.task == "grasp":
        lift = max_lift
        if lift is None:
            print("!! Gazebo からブロックの位置を読めず、浮き上がりを判定できませんでした")
            lift = 0.0
    summarize(bag_dir, target, lift)


if __name__ == "__main__":
    main()
