"""wire_statics.py のフェーズC検証テスト。

`my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部フェーズCの検証内容、および符号規約の較正結果をまとめる。実機・CANバスに非依存。
"""

import numpy as np
import pytest
from wire_mechanism import wire_kinematics as wk
from wire_mechanism import wire_statics as ws


def test_moment_arm_equals_positive_wire_length_derivative():
    """l_moment_arm = +d(l_wire)/d(theta_joint) であることを有限差分で較正する。

    wire_kinematics.moment_arm() の符号規約が、仮想仕事の原理に基づく
    tau_wire = -T・l_moment_arm（wire_statics.py の前提）と整合することの根拠。
    この恒等式は theta_included が theta_joint に対して傾き+1の線形関数である
    ことのみから従うため、wire_kinematics.pulley_polar_from_xy() の
    (x, z)→theta_pulley 変換に既知の疑義があっても影響を受けない
    （pulley側の角度が theta_joint に依存しないため）。
    """
    rng = np.random.default_rng(2)
    h = 1e-6
    worst = 0.0
    for _ in range(500):
        x, z = rng.uniform(-1.5, 1.5, 2)
        l_anchor = rng.uniform(0.05, 1.0)
        alpha = rng.uniform(-np.pi, np.pi)
        theta_joint = rng.uniform(-np.pi, np.pi)

        def l_wire_at(theta):
            geom = wk.solve_wire_geometry(x, z, l_anchor, theta, alpha)
            return geom.l_wire

        l_wire_here = l_wire_at(theta_joint)
        if l_wire_here < 1e-3:
            continue  # 退化点近傍は有限差分の数値誤差が乗るため除外

        d_l_wire = (l_wire_at(theta_joint + h) - l_wire_at(theta_joint - h)) / (2 * h)
        geom_here = wk.solve_wire_geometry(x, z, l_anchor, theta_joint, alpha)
        worst = max(worst, abs(geom_here.l_moment_arm - d_l_wire))

    assert worst < 1e-6


@pytest.mark.parametrize(
    ("theta_deg", "expected_sign"),
    [(-90, 0.0), (90, 0.0), (0, -1.0), (180, 1.0)],
)
def test_gravity_torque_equilibria_and_signs(theta_deg, expected_sign):
    theta = np.deg2rad(theta_deg)
    tau = ws.gravity_torque(theta, mass=1.0, l_com=0.3, g=9.8)
    if expected_sign == 0.0:
        assert tau == pytest.approx(0.0, abs=1e-9)
    else:
        assert np.sign(tau) == expected_sign


def test_gravity_torque_matches_energy_derivative():
    """tau_gravity = -dV/d(theta_joint)（V = mass*g*l_com*sin(theta_joint)）を有限差分で確認する。"""
    mass, l_com, g = 1.0, 0.3, 9.8
    h = 1e-6
    theta = np.linspace(-np.pi, np.pi, 200)

    def potential(th):
        return mass * g * l_com * np.sin(th)

    d_v = (potential(theta + h) - potential(theta - h)) / (2 * h)
    expected = -d_v
    actual = ws.gravity_torque(theta, mass, l_com, g)
    assert np.allclose(actual, expected, atol=1e-6)


def test_downward_hang_is_stable_equilibrium():
    """theta_joint=-90°(鉛直下向き)近傍で、変位方向と逆向きの復元トルクが働く(安定)ことを確認する。"""
    mass, l_com, g = 1.0, 0.3, 9.8
    theta_eq = -np.pi / 2
    delta = np.deg2rad(5)

    tau_plus = ws.gravity_torque(theta_eq + delta, mass, l_com, g)
    tau_minus = ws.gravity_torque(theta_eq - delta, mass, l_com, g)

    # 平衡点からずらすと、ずれを打ち消す向き(theta_eqに戻す向き)のトルクが生じるべき
    assert tau_plus < 0  # +方向にずらすと-方向へ戻す
    assert tau_minus > 0  # -方向にずらすと+方向へ戻す


def test_solve_wire_tension_basic_feasible_case():
    result = ws.solve_wire_tension(tau_external=2.0, l_moment_arm=0.5)
    assert result.tension == pytest.approx(4.0)
    assert result.feasible is True


def test_solve_wire_tension_negative_required_is_infeasible():
    """外力トルクが負(=張力も負でないと釣り合わない)なら feasible=False とする。"""
    result = ws.solve_wire_tension(tau_external=-2.0, l_moment_arm=0.5)
    assert result.tension < 0
    assert result.feasible is False


def test_solve_wire_tension_near_singular_moment_arm_is_infeasible():
    result = ws.solve_wire_tension(
        tau_external=1.0, l_moment_arm=1e-6, l_moment_arm_min=1e-4
    )
    assert result.feasible is False


def test_solve_wire_tension_exact_zero_moment_arm_does_not_raise():
    with np.errstate(divide="ignore", invalid="ignore"):
        result = ws.solve_wire_tension(tau_external=1.0, l_moment_arm=0.0)
    assert result.feasible is False
    assert np.isinf(result.tension) or np.isnan(result.tension)


def test_solve_wire_tension_array_input():
    tau_external = np.array([1.0, -1.0, 1.0])
    l_moment_arm = np.array([0.5, 0.5, 1e-6])

    result = ws.solve_wire_tension(tau_external, l_moment_arm, l_moment_arm_min=1e-4)

    assert result.tension == pytest.approx(np.array([2.0, -2.0, 1e6]))
    assert list(result.feasible) == [True, False, False]


def test_solve_static_tension_gravity_pipeline_matches_manual_composition():
    x, z, l_anchor, alpha = 0.4, 0.3, 0.15, np.deg2rad(15)
    theta_joint = np.deg2rad(-60)
    mass, l_com, g = 0.8, 0.25, 9.8

    geom = wk.solve_wire_geometry(x, z, l_anchor, theta_joint, alpha)
    result = ws.solve_static_tension_gravity(
        theta_joint, geom.l_moment_arm, mass, l_com, g
    )

    tau_gravity = ws.gravity_torque(theta_joint, mass, l_com, g)
    expected = ws.solve_wire_tension(tau_gravity, geom.l_moment_arm)

    assert result.tension == pytest.approx(expected.tension)
    assert result.feasible == expected.feasible
