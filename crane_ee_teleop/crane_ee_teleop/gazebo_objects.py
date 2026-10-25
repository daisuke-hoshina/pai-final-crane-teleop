"""Gazebo (Ignition Fortress) のモデルの位置を読む。

ros_gz_bridge で /world/<name>/pose/info を ROS の TFMessage にすると、モデル名が落ちて区別できない。
そのため `ign topic -e -n 1` の出力を直接読む（1 回 0.3 秒ほど）。
"""

import re
import subprocess

WORLD_POSE_TOPIC = "/world/default/pose/info"
# crane_plus_with_table.launch.py がロボットを置く高さ（create の -z）。ロボット基準へ直すのに使う
ROBOT_BASE_Z = 1.015
CUBE_NAME = "wood_cube_5cm"
# Fuel の "Wood cube 5cm" はモデルの原点が底面にある（机の上で原点の高さ＝机の高さ 1.015 になる）。
# 中心はそこから 2.5cm 上
CUBE_HALF_SIZE = 0.025
# ブロックの初期位置（world 座標）。crane_plus_with_table.launch.py の world に置かれている位置
CUBE_HOME_WORLD = (0.0, -0.1, 1.02)

_NUM = r"(-?[\d.]+(?:e[-+]?\d+)?)"


def model_position(name: str = CUBE_NAME, timeout: float = 5.0):
    """world 座標でのモデルの位置 (x, y, z)。読めなければ None。"""
    try:
        out = subprocess.run(
            ["ign", "topic", "-e", "-n", "1", "-t", WORLD_POSE_TOPIC],
            capture_output=True,
            text=True,
            timeout=timeout,
        ).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    parts = out.split(f'name: "{name}"')
    if len(parts) < 2:
        return None
    # 名前のすぐ後の position { ... } を読む。0 の成分は出力から省かれる
    m = re.search(r"position \{([^}]*)\}", parts[1])
    if m is None:
        return None
    v = {k: float(x) for k, x in re.findall(r"(\w): " + _NUM, m.group(1))}
    return v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0)


def model_position_in_robot_frame(name: str = CUBE_NAME):
    """crane_plus_base 基準でのモデルの原点の位置。ロボットは world の (0, 0, ROBOT_BASE_Z) に回転なしで置かれている。"""
    p = model_position(name)
    if p is None:
        return None
    return p[0], p[1], p[2] - ROBOT_BASE_Z


def cube_center_in_robot_frame():
    """crane_plus_base 基準でのブロックの中心。ブロックが傾いていないことを前提にしている。"""
    p = model_position_in_robot_frame(CUBE_NAME)
    if p is None:
        return None
    return p[0], p[1], p[2] + CUBE_HALF_SIZE


def reset_cube(timeout: float = 5.0) -> bool:
    """ブロックを初期位置へ戻す（向きもまっすぐに）。戻せたら True。"""
    x, y, z = CUBE_HOME_WORLD
    req = f'name: "{CUBE_NAME}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
    try:
        out = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                "/world/default/set_pose",
                "--reqtype",
                "ignition.msgs.Pose",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                str(int(timeout * 1000)),
                "--req",
                req,
            ],
            capture_output=True,
            text=True,
            timeout=timeout + 2,
        ).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return "data: true" in out
