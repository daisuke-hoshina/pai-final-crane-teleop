"""CRANE+ V2 の順運動学・逆運動学（ROS に依存しない）。

寸法と回転軸は crane_plus_description の URDF から取った（2026-10-12 に Gazebo の tf と 0.1mm 以内で一致を確認）。

  joint1: base から z +0.0454、軸 z      （根元の旋回）
  joint2: joint1 から z +0.026、軸 +y    （正で前に傾く）
  joint3: joint2 から z +0.083、軸 -y    （正で後ろに折れる）
  joint4: joint3 から z +0.0935、軸 -y   （正で後ろに折れる）

腕の平面内では、各リンクの「鉛直からの傾き（前向きが正）」が
  phi2 = q2,  phi3 = q2 - q3,  phi4 = q2 - q3 - q4
になる。手先の傾き pitch は phi4 で、0 が真上向き、pi/2 が水平前向き、pi が真下向き。

座標はすべて crane_plus_base 基準 [m]。手先点（TCP）は link4 の原点から link4 の z 軸方向に TOOL_LENGTH 先。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

SHOULDER_HEIGHT = 0.0454 + 0.026  # base から joint2 までの高さ
L2 = 0.083  # joint2 → joint3
L3 = 0.0935  # joint3 → joint4
# link4 の原点から指先の間までの長さ。書籍の endtip 版 launch（endtip_offset_xyz '0 0 0.09'）に合わせた
TOOL_LENGTH = 0.09

# URDF の可動範囲 [rad]（joint1〜4）
JOINT_LIMITS = (
    (-2.6179916666666663, 2.6179916666666663),
    (-1.8247401916666663, 1.7837249888888889),
    (-2.5565561288888885, 2.454279921111111),
    (-1.8400990761111107, 1.768366104444444),
)


@dataclass(frozen=True)
class Pose:
    """手先の位置 (x, y, z) [m] と傾き pitch [rad]。"""

    x: float
    y: float
    z: float
    pitch: float


def fk(q: tuple[float, float, float, float], tool_length: float = TOOL_LENGTH) -> Pose:
    """関節角 (q1, q2, q3, q4) から手先の姿勢を求める。"""
    q1, q2, q3, q4 = q
    phi2, phi3, phi4 = q2, q2 - q3, q2 - q3 - q4
    r = L2 * math.sin(phi2) + L3 * math.sin(phi3) + tool_length * math.sin(phi4)
    z = SHOULDER_HEIGHT + L2 * math.cos(phi2) + L3 * math.cos(phi3) + tool_length * math.cos(phi4)
    return Pose(r * math.cos(q1), r * math.sin(q1), z, phi4)


def within_limits(q: tuple[float, float, float, float]) -> bool:
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_LIMITS, strict=True))


def ik_solutions(
    pose: Pose, tool_length: float = TOOL_LENGTH
) -> list[tuple[float, float, float, float]]:
    """手先の姿勢を実現する関節角を、可動範囲内のものだけすべて返す（0〜4 個）。

    根元の向きが 2 通り（手先の方向を向く／反対を向いて後ろへ反る）、それぞれに腕の平面内の
    2 リンクの解が 2 通り（肘が手首より後ろ・上／前・下）ある。
    """
    r = math.hypot(pose.x, pose.y)
    # 真上（r≈0）では旋回角が決まらないので 0 にする
    q1 = math.atan2(pose.y, pose.x) if r > 1e-6 else 0.0
    out = []
    for base, reach in ((q1, r), (_wrap(q1 + math.pi), -r)):
        # 腕の平面内で、手首（joint4）の位置。reach が負なら根元より後ろ
        rw = reach - tool_length * math.sin(pose.pitch)
        zw = pose.z - SHOULDER_HEIGHT - tool_length * math.cos(pose.pitch)
        c = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        if not -1.0 <= c <= 1.0:
            continue  # 腕の長さが足りない、または縮めきれない
        for sign in (1.0, -1.0):
            delta = sign * math.acos(c)  # phi3 - phi2
            phi2 = math.atan2(rw, zw) - math.atan2(L3 * math.sin(delta), L2 + L3 * math.cos(delta))
            phi3 = phi2 + delta
            q = tuple(_wrap(v) for v in (base, phi2, phi2 - phi3, phi3 - pose.pitch))
            if within_limits(q):
                out.append(q)
    return out


def ik(
    pose: Pose,
    q_ref: tuple[float, float, float, float] | None = None,
    tool_length: float = TOOL_LENGTH,
) -> tuple[float, float, float, float] | None:
    """手先の姿勢から関節角を求める。届かない・可動範囲外なら None。

    解が 2 つあるときは q_ref（いまの関節角）にいちばん近いものを選ぶ。テレオペで少しずつ動かすときに、
    途中で別の解へ飛んでアームが大きく動くのを防ぐため。q_ref が無ければ直立 (0, 0, 0, 0) に近いものを選ぶ。
    """
    sols = ik_solutions(pose, tool_length)
    if not sols:
        return None
    ref = q_ref if q_ref is not None else (0.0, 0.0, 0.0, 0.0)
    return min(sols, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, ref, strict=True)))


def _wrap(a: float) -> float:
    """角度を [-pi, pi) に収める。"""
    return (a + math.pi) % (2 * math.pi) - math.pi
