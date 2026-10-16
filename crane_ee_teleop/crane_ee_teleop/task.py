"""タスクの定義: ホームから手先を目標点まで運ぶ。

目標点はブロックの中心の真上 TARGET_HEIGHT_ABOVE_BLOCK。ブロックはつかまず、目印として使う
（Gazebo ではグリッパーで持ち上げるとブロックが滑り、つかむ動作が安定しないため。2026-10-13 に確認）。
エピソードの終わりに手先が目標点から SUCCESS_TOL 以内にあれば成功とする。
"""

import math

from crane_ee_teleop.gazebo_objects import cube_center_in_robot_frame
from crane_ee_teleop.kinematics import fk
from crane_ee_teleop.teleop_keyboard_ee import HOME_Q

# 指の先（手先点より 2〜3cm 下）がブロックの上面（中心 +2.5cm）に当たらない高さ
TARGET_HEIGHT_ABOVE_BLOCK = 0.055  # [m]
SUCCESS_TOL = 0.01  # [m]
HOME_TOL = 0.02  # [rad] 各関節がホームからこれ以内なら、ホームにいるとみなす
STILL_TOL = 0.05  # [rad/s]

ARM_JOINTS = [f"crane_plus_joint{k}" for k in (1, 2, 3, 4)]


def target_position():
    """crane_plus_base 基準の目標点。Gazebo からブロックの位置が読めなければ None。"""
    c = cube_center_in_robot_frame()
    if c is None:
        return None
    return c[0], c[1], c[2] + TARGET_HEIGHT_ABOVE_BLOCK


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
