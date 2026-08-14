"""drive_modes.py（フェーズA-2再検討）のユニットテスト。"""

import numpy as np
import pytest
from wire_mechanism import drive_modes as dm
from wire_mechanism.wire_statics import gravity_torque, solve_wire_tension

LINK = dm.LinkSpec(mass=1.0, l_com=0.15, inertia=0.03, g=9.8)


def test_demand_reduces_to_gravity_torque_when_static():
    """加速度ゼロ・バネ無しなら、トルク需要は重力トルクそのものになる。"""
    theta = np.linspace(-np.pi / 2, np.pi / 2, 101)
    demand = dm.wire_torque_demand(theta, np.zeros_like(theta), LINK)
    expected = gravity_torque(theta, LINK.mass, LINK.l_com, LINK.g)
    assert demand == pytest.approx(expected, abs=1e-12)


def test_unidirectional_matches_wire_statics_in_static_limit():
    """静的極限では wire_statics.solve_wire_tension と同じ張力・可否になる。"""
    theta = np.linspace(-np.pi / 3, np.pi / 3, 201)
    l_arm = np.full_like(theta, 0.05)
    demand = dm.wire_torque_demand(theta, np.zeros_like(theta), LINK)

    result = dm.unidirectional(demand, l_arm)
    reference = solve_wire_tension(demand, l_arm)

    assert result.feasible == bool(np.all(reference.feasible))
    assert result.tension_max == pytest.approx(reference.tension.max(), rel=1e-12)


def test_motion_spec_sample_is_consistent_sinusoid():
    """sample() の theta/dtheta/ddtheta が微分関係を満たす。"""
    spec = dm.MotionSpec(amplitude=np.deg2rad(60), frequency=1.0)
    theta, dtheta, ddtheta = spec.sample(num_points=20001)
    t = np.linspace(0.0, 1.0, 20001)
    dt = t[1] - t[0]

    assert np.gradient(theta, dt)[5:-5] == pytest.approx(dtheta[5:-5], abs=1e-4)
    assert ddtheta == pytest.approx(
        -((2 * np.pi) ** 2) * (theta - spec.center), abs=1e-9
    )


def test_motion_spec_zero_frequency_is_quasistatic_sweep():
    """周波数0なら可動域の掃引になり、速度・加速度はゼロ。"""
    spec = dm.MotionSpec(amplitude=np.deg2rad(90), frequency=0.0)
    theta, dtheta, ddtheta = spec.sample(num_points=51)
    assert theta.min() == pytest.approx(-np.pi / 2)
    assert theta.max() == pytest.approx(np.pi / 2)
    assert np.all(dtheta == 0.0) and np.all(ddtheta == 0.0)


def test_inertia_can_break_unidirectional_feasibility():
    """静的には成立する動作でも、揺動が速いと単方向1本では T<0 が必要になる。"""
    l_arm = np.full(721, 0.05)

    slow = dm.MotionSpec(amplitude=np.deg2rad(60), frequency=0.1)
    theta_s, _, ddtheta_s = slow.sample()
    assert dm.unidirectional(
        dm.wire_torque_demand(theta_s, ddtheta_s, LINK), l_arm
    ).feasible

    fast = dm.MotionSpec(amplitude=np.deg2rad(60), frequency=2.0)
    theta_f, _, ddtheta_f = fast.sample()
    assert not dm.unidirectional(
        dm.wire_torque_demand(theta_f, ddtheta_f, LINK), l_arm
    ).feasible


def test_spring_preload_restores_feasibility_at_range_ends():
    """可動域端で重力トルクが0になりワイヤーがたるむ問題を、バネ予荷重が解消する。"""
    theta = np.linspace(-np.pi / 2, np.pi / 2, 361)
    l_arm = np.full_like(theta, 0.05)
    tension_min = 5.0  # たるみ防止のため最低5Nを要求する

    bare = dm.unidirectional(
        dm.wire_torque_demand(theta, np.zeros_like(theta), LINK), l_arm, tension_min
    )
    assert not bare.feasible  # theta=±90°で tau_gravity=0 → T=0 < 5N

    with_spring = dm.unidirectional(
        dm.wire_torque_demand(theta, np.zeros_like(theta), LINK, tau_spring=1.0),
        l_arm,
        tension_min,
    )
    assert with_spring.feasible


def test_antagonistic_is_feasible_where_unidirectional_is_not():
    """符号が逆のモーメントアーム2本なら、需要の符号が反転しても成立する。"""
    theta = np.linspace(-np.pi, np.pi, 721)  # tau_gravity の符号が反転する広い範囲
    demand = dm.wire_torque_demand(theta, np.zeros_like(theta), LINK)
    l_a = np.full_like(theta, 0.05)
    l_b = np.full_like(theta, -0.05)

    assert not dm.unidirectional(demand, l_a).feasible

    result, tension_a, tension_b = dm.antagonistic(demand, l_a, l_b, tension_min=1.0)
    assert result.feasible
    assert np.all(tension_a >= 1.0 - 1e-12) and np.all(tension_b >= 1.0 - 1e-12)


def test_antagonistic_reproduces_demand():
    """拮抗の張力配分が、元のトルク需要 D = T_a*l_a + T_b*l_b を再現する。"""
    theta = np.linspace(-np.pi, np.pi, 501)
    demand = dm.wire_torque_demand(theta, np.zeros_like(theta), LINK)
    l_a = np.full_like(theta, 0.04)
    l_b = np.full_like(theta, -0.06)

    _, tension_a, tension_b = dm.antagonistic(demand, l_a, l_b, tension_min=2.0)
    assert tension_a * l_a + tension_b * l_b == pytest.approx(demand, abs=1e-10)


def test_max_joint_speed_tradeoff_with_moment_arm():
    """モーメントアームが大きいほど関節は遅くなる（V_max=6.0 rad/s, r=0.02m）。"""
    assert dm.max_joint_speed(0.05, r_drum=0.02, motor_speed_max=6.0) == pytest.approx(
        2.4
    )
    assert dm.max_joint_speed(0.10, r_drum=0.02, motor_speed_max=6.0) == pytest.approx(
        1.2
    )
    # 符号には依らない（拮抗側の負のモーメントアームでも同じ上限）
    assert dm.max_joint_speed(-0.05, r_drum=0.02, motor_speed_max=6.0) == pytest.approx(
        2.4
    )


def test_motor_torque_is_linear_in_drum_radius():
    assert dm.motor_torque(30.0, r_drum=0.02) == pytest.approx(0.6)
