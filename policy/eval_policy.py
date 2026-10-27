"""学習した ACT 方策をデプロイして、つかんで持ち上げられるかを N 回測る。

1 回ごとに: ホームへ戻す（グリッパーは開く）→ ブロックを初期位置へ → 講義の infer.py を起動 →
T 秒のあいだブロックの浮きと横（x, y）のずれを見て、最大の浮き（3cm 以上で成功）とその時刻、
最大の横ずれ・終わったときの横ずれ・最初に 5mm 以上ずれた時刻を出す → infer.py を止める。

  python3 eval_policy.py <model_path> [N] [T] [infer_script]
環境変数 TRAJ_DIR を指定すると、各回の関節角の推移を TRAJ_DIR/trial<i>.csv に書き出す
（t = 方策が動き始めてからの秒、関節は学習データと同じ並び joint1, joint2, joint4, joint3, gripper）。
あわせて、コントローラのトピックに届いた指令を TRAJ_DIR/trial<i>_cmd.csv に書き出す（t, arm 4 つ（指令の並び）, gripper）。
"""
import os, signal, subprocess, sys, time
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
sys.path.insert(0, "/workspace/pai-final-crane-teleop/crane_ee_teleop")
from crane_ee_teleop.gazebo_objects import model_position_in_robot_frame, reset_cube
from crane_ee_teleop.task import LIFT_SUCCESS, arm_q, hand_position
from crane_ee_teleop.teleop_keyboard_ee import (ARM_JOINT_NAMES, GRIPPER_JOINT_NAME, GRIPPER_OPEN, HOME_Q,
                                                to_arm_order)

model, n, t_run = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 5, float(sys.argv[3]) if len(sys.argv) > 3 else 120
script = sys.argv[4] if len(sys.argv) > 4 else "scripts/infer.py"
rclpy.init(); node = rclpy.create_node("eval_policy")
arm = node.create_publisher(JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10)
grip = node.create_publisher(JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10)
js = {}
TRAJ_NAMES = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint4", "crane_plus_joint3", GRIPPER_JOINT_NAME]  # 学習データの並び
traj = {"t0": None, "rows": []}  # t0 が None のあいだは記録しない


def on_joint_state(m):
    js["msg"] = m
    if traj["t0"] is not None:
        pos = dict(zip(m.name, m.position))
        traj["rows"].append([time.time() - traj["t0"]] + [pos.get(n, float("nan")) for n in TRAJ_NAMES])


node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
cmds = {"arm": None, "rows": []}  # 届いた指令。arm は最後に届いた arm の指令


def on_cmd(m, kind):
    if traj["t0"] is None or not m.points:
        return
    pos = list(m.points[0].positions)
    if kind == "arm":
        cmds["arm"] = pos
    elif cmds["arm"] is not None:
        cmds["rows"].append([time.time() - traj["t0"]] + cmds["arm"] + pos)


node.create_subscription(JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", lambda m: on_cmd(m, "arm"), 10)
node.create_subscription(JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", lambda m: on_cmd(m, "grip"), 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

def go_home(seconds=4.0):
    end = time.time() + seconds
    while time.time() < end:
        for pub, names, pos, tfs in ((arm, ARM_JOINT_NAMES, to_arm_order(HOME_Q), 0.1), (grip, [GRIPPER_JOINT_NAME], [GRIPPER_OPEN], 0.01)):
            jt = JointTrajectory(); jt.joint_names = names
            pt = JointTrajectoryPoint(positions=[float(v) for v in pos]); pt.time_from_start.nanosec = int(tfs * 1e9)
            jt.points = [pt]; pub.publish(jt)
        time.sleep(0.05)

results = []
for i in range(n):
    go_home(); reset_cube(); time.sleep(2)
    p0 = model_position_in_robot_frame()  # 方策を動かす前のブロックの位置（横ずれの基準）
    log = f"/tmp/eval_infer_{i}.log"
    # infer.py は 20Hz で action を出力し続けるので、パイプではなくファイルへ書かせる（パイプが詰まると止まる）
    p = subprocess.Popen(
        ["bash", "-c", f"export PATH=$HOME/.pixi/bin:$PATH; cd ~/pai_training && exec pixi run python {script} --model-path {model} > {log} 2>&1"],
        start_new_session=True)
    # 方策の読み込みを待つ（最初の action が出たら開始）
    while not (os.path.exists(log) and "action:" in open(log).read()):
        time.sleep(0.2)
    gmax = -9.0
    t0 = time.time(); best = (0.0, 0.0); low = (1.0, None, 0.0)  # 手先の最低点 (z, 位置, グリッパー)
    traj["rows"] = []; cmds["rows"] = []; cmds["arm"] = None; traj["t0"] = t0
    shift_max, shift_last, t_moved = 0.0, (0.0, 0.0), None  # 横ずれ [m]、最初に 5mm 以上ずれた時刻
    while time.time() - t0 < t_run:
        c = model_position_in_robot_frame()  # ブロックの位置（p は infer.py のプロセスなので別の名前にする）
        if c is not None:
            h = c[2]
            if h > best[0]:
                best = (h, time.time() - t0)
            if p0 is not None:
                shift_last = (c[0] - p0[0], c[1] - p0[1])
                d = (shift_last[0] ** 2 + shift_last[1] ** 2) ** 0.5
                shift_max = max(shift_max, d)
                if t_moved is None and d >= 0.005:
                    t_moved = time.time() - t0
        m = js.get("msg")
        if m is not None:
            hp = hand_position(arm_q(m.name, m.position))
            g = dict(zip(m.name, m.position)).get(GRIPPER_JOINT_NAME, 0.0)
            if hp[2] < low[0]:
                low = (hp[2], [round(100 * v, 1) for v in hp], g)
            gmax = max(gmax, g)
    traj["t0"] = None
    os.killpg(p.pid, signal.SIGINT); p.wait(timeout=20)
    if os.environ.get("TRAJ_DIR"):
        os.makedirs(os.environ["TRAJ_DIR"], exist_ok=True)
        with open(os.path.join(os.environ["TRAJ_DIR"], f"trial{i}.csv"), "w") as f:
            f.write("t,joint1,joint2,joint4,joint3,gripper\n")
            for r in list(traj["rows"]):
                f.write(",".join(f"{v:.4f}" for v in r) + "\n")
        with open(os.path.join(os.environ["TRAJ_DIR"], f"trial{i}_cmd.csv"), "w") as f:
            f.write("t,cmd0,cmd1,cmd2,cmd3,gripper\n")
            for r in list(cmds["rows"]):
                f.write(",".join(f"{v:.4f}" for v in r) + "\n")
    ok = best[0] >= LIFT_SUCCESS
    results.append(ok)
    print(f"trial {i}: 最大の浮き {100 * best[0]:.1f}cm（{best[1]:.0f} 秒後）/ {'成功' if ok else '失敗'}"
          f" / 手先の最低点 {low[1]}（そのときのグリッパー {low[2]:+.2f}）/ グリッパーの最大 {gmax:+.2f}"
          f" / ブロックの横ずれ 最大 {100 * shift_max:.1f}cm・終わり [{100 * shift_last[0]:+.1f}, {100 * shift_last[1]:+.1f}]cm"
          f"（5mm 以上ずれたのは {'なし' if t_moved is None else f'{t_moved:.0f} 秒後'}）", flush=True)
go_home(); reset_cube()
print(f"成功 {sum(results)}/{n}")
