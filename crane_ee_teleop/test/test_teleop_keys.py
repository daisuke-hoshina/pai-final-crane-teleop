import math

import pytest
from crane_ee_teleop.teleop_keyboard_ee import HOME_POSITION, HOME_Q, solve_pointing_down, step_target


def _cyl(p):
    return math.hypot(p[0], p[1]), math.atan2(p[1], p[0]), p[2]


@pytest.mark.parametrize("key", ["a", "d"])
def test_turn_keeps_distance_and_height_and_moves_only_joint1(key):
    t, q = HOME_POSITION, HOME_Q
    for _ in range(31):  # ホームから約 90° 回る（ブロックの方向）
        t = step_target(t, key)
        q_new = solve_pointing_down(*t, q_ref=q)
        assert q_new is not None
        assert q_new[1:] == pytest.approx(q[1:], abs=1e-6)
        q = q_new
    r, th, z = _cyl(t)
    assert r == pytest.approx(math.hypot(*HOME_POSITION[:2]))
    assert z == pytest.approx(HOME_POSITION[2])
    assert th == pytest.approx(31 * 0.05 * (1 if key == "a" else -1))


def test_d_turns_right():
    t = step_target(HOME_POSITION, "d")
    assert t[1] < 0  # 右 = −y


@pytest.mark.parametrize("key, dr", [("w", 0.005), ("s", -0.005)])
def test_forward_back_keeps_direction_and_height(key, dr):
    start = step_target(step_target(HOME_POSITION, "d"), "d")  # 正面以外の向きでも確かめる
    t = step_target(start, key)
    r0, th0, z0 = _cyl(start)
    r, th, z = _cyl(t)
    assert r - r0 == pytest.approx(dr)
    assert th == pytest.approx(th0)
    assert z == pytest.approx(z0)
    q0 = solve_pointing_down(*start, q_ref=HOME_Q)
    assert solve_pointing_down(*t, q_ref=q0)[0] == pytest.approx(q0[0])  # joint1 は変わらない


@pytest.mark.parametrize("key, dz", [("r", 0.005), ("f", -0.005)])
def test_up_down_changes_only_height(key, dz):
    t = step_target(HOME_POSITION, key)
    assert t[:2] == pytest.approx(HOME_POSITION[:2])
    assert t[2] - HOME_POSITION[2] == pytest.approx(dz)
