"""タスクの定義。2 種類ある。

reach: ホームから手先を目標点（ブロックの中心の真上 TARGET_HEIGHT_ABOVE_BLOCK）まで運ぶ。
  エピソードの終わりに手先が目標点から SUCCESS_TOL 以内にあれば成功。
grasp: ホームからつかむ位置（ブロックの中心の GRASP_HEIGHT_ABOVE_CENTER 上）へ手先を運び、つかんで持ち上げる。
  エピソード中に一度でもブロックの底面が机から LIFT_SUCCESS 以上浮けば成功（Gazebo では持ち上げた
  ブロックが数秒で滑り落ちるため、保持し続けることは求めない）。
  指の摩擦係数を 0.2 → 1.0 に上げた Gazebo で使う（0.2 では持ち上げる間に滑る）。
  手先点は指先より 2〜3cm 上にあり、中心の高さ（机から 2.5cm）までは指先が机に着いて下りられない。
  中心の 1cm 上なら届き、10 試行とも 5.2〜5.9cm 持ち上がった（ブロックの傾き約 19°。2026-10-25 に測定）。
"""

import math

from crane_ee_teleop.gazebo_objects import cube_center_in_robot_frame, model_position_in_robot_frame
from crane_ee_teleop.kinematics import fk
from crane_ee_teleop.teleop_keyboard_ee import HOME_Q

# 指の先（手先点より 2〜3cm 下）がブロックの上面（中心 +2.5cm）に当たらない高さ
TARGET_HEIGHT_ABOVE_BLOCK = 0.055  # [m]
SUCCESS_TOL = 0.01  # [m]
GRASP_HEIGHT_ABOVE_CENTER = 0.01  # [m]
# grasp で成功とみなすブロックの浮き上がり。8cm 上げると、つかむ高さによって 3〜6cm 浮いた
LIFT_SUCCESS = 0.03  # [m]
TASKS = ("reach", "grasp")
HOME_TOL = 0.02  # [rad] 各関節がホームからこれ以内なら、ホームにいるとみなす
STILL_TOL = 0.05  # [rad/s]

ARM_JOINTS = [f"crane_plus_joint{k}" for k in (1, 2, 3, 4)]


def target_position(task="reach"):
    """crane_plus_base 基準の目標点。Gazebo からブロックの位置が読めなければ None。"""
    c = cube_center_in_robot_frame()
    if c is None:
        return None
    if task == "grasp":
        return c[0], c[1], c[2] + GRASP_HEIGHT_ABOVE_CENTER
    return c[0], c[1], c[2] + TARGET_HEIGHT_ABOVE_BLOCK


def cube_lift():
    """ブロックの底面の机からの高さ [m]。机に置かれていれば約 0。読めなければ None。"""
    p = model_position_in_robot_frame()
    return None if p is None else p[2]


def arm_q(names, positions):
    """JointState の name / position から (q1, q2, q3, q4) を取り出す。"""
    d = dict(zip(names, positions, strict=True))
    return tuple(d[n] for n in ARM_JOINTS)


def hand_position(q):
    p = fk(q)
    return p.x, p.y, p.z


def distance_to_target(q, target):
    return math.dist(hand_position(q), target)


def at_home(q, velocities=None):
    still = velocities is None or max(abs(v) for v in velocities) < STILL_TOL
    return still and max(abs(a - b) for a, b in zip(q, HOME_Q, strict=True)) < HOME_TOL
