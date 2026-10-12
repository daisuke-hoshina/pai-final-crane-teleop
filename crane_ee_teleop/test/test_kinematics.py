import math
import random

import pytest
from crane_ee_teleop.kinematics import JOINT_LIMITS, Pose, fk, ik, ik_solutions, within_limits

# Gazebo の tf（crane_plus_base → crane_plus_link4）で測った値。tool_length=0 の fk と比べる
TF_SAMPLES = [
    ((-0.0, -0.01, 0.04, -0.02), (-0.0057, 0.0000, 0.2478)),
    ((0.29, 0.39, 0.58, 0.19), (0.0126, 0.0038, 0.2400)),
    ((-0.47, 0.79, -0.36, 0.49), (0.1285, -0.0648, 0.1687)),
    ((-0.0, -0.29, 0.98, -0.59), (-0.1131, 0.0006, 0.1784)),
]


@pytest.mark.parametrize("q, xyz", TF_SAMPLES)
def test_fk_matches_gazebo_tf(q, xyz):
    # tf の値は丸めた関節角で取ったので、許容誤差は数 mm にする
    p = fk(q, tool_length=0.0)
    assert (p.x, p.y, p.z) == pytest.approx(xyz, abs=5e-3)


def test_upright_pose():
    p = fk((0.0, 0.0, 0.0, 0.0))
    assert p.x == pytest.approx(0.0)
    assert p.y == pytest.approx(0.0)
    assert p.pitch == pytest.approx(0.0)
    assert p.z == pytest.approx(0.0714 + 0.083 + 0.0935 + 0.09)


def test_ik_roundtrip_random():
    rng = random.Random(0)
    solved = 0
    for _ in range(2000):
        q = tuple(rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS)
        target = fk(q)
        q_ik = ik(target, q_ref=q)
        assert q_ik is not None  # 元の q が可動範囲内の解なので、必ず解ける
        solved += 1
        assert within_limits(q_ik)
        p = fk(q_ik)
        assert (p.x, p.y, p.z) == pytest.approx((target.x, target.y, target.z), abs=1e-6)
        assert math.cos(p.pitch - target.pitch) == pytest.approx(1.0, abs=1e-9)
        # 元の関節角を参照にすれば、元の解そのものが選ばれる
        assert q_ik == pytest.approx(q, abs=1e-6)
    assert solved == 2000


def test_ik_picks_solution_nearest_to_reference():
    rng = random.Random(1)
    checked = 0
    for _ in range(500):
        target = fk(tuple(rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS))
        sols = ik_solutions(target)
        if len(sols) < 2:
            continue
        checked += 1
        for s in sols:
            assert ik(target, q_ref=s) == pytest.approx(s, abs=1e-9)
    assert checked > 100


def test_ik_small_step_stays_continuous():
    # 手先を 1mm ずつ動かしたとき、関節角が飛ばないこと
    q = ik(Pose(0.12, 0.0, 0.10, 2.0))
    for i in range(1, 60):
        q_next = ik(Pose(0.12, 0.001 * i, 0.10, 2.0), q_ref=q)
        if q_next is None:
            break
        assert max(abs(a - b) for a, b in zip(q_next, q, strict=True)) < 0.05
        q = q_next


def test_ik_book_pick_pose():
    # 書籍の pick_and_place の把持位置付近（真下向き）
    q = ik(Pose(0.0, -0.09, 0.05, math.pi))
    assert q is not None
    p = fk(q)
    assert (p.x, p.y, p.z) == pytest.approx((0.0, -0.09, 0.05), abs=1e-6)


def test_ik_unreachable():
    assert ik(Pose(0.5, 0.0, 0.1, math.pi / 2)) is None
