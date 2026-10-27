import math

import pytest
from crane_ee_teleop.task_monitor import cylindrical_diff
from crane_ee_teleop.teleop_keyboard_ee import HOME_POSITION, step_target


def test_block_on_the_right_means_turn_right():
    # ホーム（正面 10cm）から見て、根元から同じ 10cm の右（−y）にある目標
    dth, arc, dr, dz = cylindrical_diff((0.0, -0.10, 0.035), HOME_POSITION)
    assert dth == pytest.approx(-math.pi / 2)
    assert arc == pytest.approx(-0.10 * math.pi / 2)
    assert dr == pytest.approx(0.0)
    assert dz == pytest.approx(0.035 - 0.10)


@pytest.mark.parametrize("key", ["a", "d", "w", "s", "r", "f"])
def test_one_key_press_changes_only_its_own_item(key):
    # キーを 1 回押した位置を目標にすると、差はそのキーの項目にだけ出る
    target = step_target(HOME_POSITION, key)
    dth, _, dr, dz = cylindrical_diff(target, HOME_POSITION)
    moved = {"a": dth > 0, "d": dth < 0, "w": dr > 0, "s": dr < 0, "r": dz > 0, "f": dz < 0}
    assert moved[key]
    others = {"ad": abs(dth), "ws": abs(dr), "rf": abs(dz)}
    for k, v in others.items():
        if key not in k:
            assert v == pytest.approx(0.0, abs=1e-9)


def test_angle_wraps_around_behind_the_base():
    # 左後ろと右後ろの差は、360° 近くではなく小さい角度になる
    dth, *_ = cylindrical_diff((-0.10, -0.001, 0.05), (-0.10, 0.001, 0.05))
    assert abs(dth) < 0.1
