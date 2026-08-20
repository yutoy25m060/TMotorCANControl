"""assumed_params.py の仮値が矛盾しないことを保証する回帰テスト。

これらの値は正式決定待ちの仮値だが、将来誰かが値を書き換えて矛盾する組み合わせに
してしまうことを防ぐため、選定根拠（モジュールdocstring参照）を数値として固定する。
"""

import numpy as np
from wire_mechanism import assumed_params as ap
from wire_mechanism import drive_modes as dm


def test_assumed_swing_is_feasible_for_unidirectional_with_tension_min():
    """仮値の組み合わせ（振幅±60°・0.5Hz・T_min=5N）は単方向1本でfeasibleである。"""
    theta, _, ddtheta = ap.ASSUMED_SWING.sample()
    l_arm = np.full_like(theta, 0.05)  # A-2節と同じ代表値（平坦近似）
    demand = dm.wire_torque_demand(theta, ddtheta, ap.ASSUMED_LINK)

    result = dm.unidirectional(demand, l_arm, ap.ASSUMED_TENSION_MIN)

    assert result.feasible


def test_assumed_r_drum_keeps_speed_limit_binding_before_dynamics_limit():
    """r_drum=40mmは、A-2再検討節の「r_drum<42mmでは速度上限が先に効く」閾値未満である。"""
    assert ap.ASSUMED_R_DRUM < 0.042


def test_assumed_r_drum_speed_limit_covers_the_assumed_swing():
    """仮値の揺動(振幅±60°・0.5Hz)に必要な関節角速度が、r_drum=40mmの速度上限に収まる。"""
    required_speed = ap.ASSUMED_SWING.amplitude * 2 * np.pi * ap.ASSUMED_SWING.frequency
    speed_limit = dm.max_joint_speed(
        l_moment_arm=0.05, r_drum=ap.ASSUMED_R_DRUM, motor_speed_max=6.0
    )
    assert required_speed < speed_limit
