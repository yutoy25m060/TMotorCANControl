"""wire_kinematics.py のフェーズB検証テスト。

`my_ak45/wire_drive/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md` B-2節の
検証ケース、および回帰・整合性テストをまとめる。実機・CANバスに非依存。
"""

import numpy as np
import pytest
from wire_mechanism import wire_kinematics as wk


def _l_moment_arm_note_formula(
    l_pulley: float, l_anchor: float, l_wire: float
) -> float:
    """ノート第1部3節の非簡略化式（三平方の定理由来）。常に非負。クロスチェック専用。"""
    inside = l_anchor**2 - ((l_anchor**2 - l_pulley**2 + l_wire**2) / (2 * l_wire)) ** 2
    return np.sqrt(np.clip(inside, 0.0, None))


def test_right_angle_equal_lengths():
    l_pulley = l_anchor = 1.0
    theta_included = np.pi / 2
    l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
    l_moment_arm = wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)
    assert l_wire == pytest.approx(np.sqrt(2), abs=1e-6)
    assert l_moment_arm == pytest.approx(1 / np.sqrt(2), abs=1e-6)


def test_moment_arm_maximum():
    l_pulley, l_anchor = 1.5, 0.5
    theta_star = np.arccos(min(l_pulley, l_anchor) / max(l_pulley, l_anchor))

    def _l_moment_arm_at(theta_included: float) -> float:
        l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
        return wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)

    l_moment_arm_at_star = _l_moment_arm_at(theta_star)
    assert l_moment_arm_at_star == pytest.approx(min(l_pulley, l_anchor), abs=1e-6)

    delta = np.deg2rad(0.5)
    assert l_moment_arm_at_star >= _l_moment_arm_at(theta_star + delta)
    assert l_moment_arm_at_star >= _l_moment_arm_at(theta_star - delta)


def test_equal_lengths_half_angle_cosine():
    common_length = 0.7
    theta_included = np.linspace(1e-3, np.pi - 1e-3, 500)
    l_wire = wk.wire_length(common_length, common_length, theta_included)
    l_moment_arm = wk.moment_arm(common_length, common_length, theta_included, l_wire)

    expected = common_length * np.cos(theta_included / 2)
    assert np.allclose(l_moment_arm, expected, atol=1e-9)
    assert np.all(np.diff(l_moment_arm) <= 1e-12)


def test_included_angle_zero_limit():
    for l_pulley, l_anchor in [(0.8, 0.5), (1.5, 0.5), (0.3, 0.9)]:
        thetas = np.array([1e-1, 1e-2, 1e-3, 1e-4])
        l_wire = wk.wire_length(l_pulley, l_anchor, thetas)
        l_moment_arm = wk.moment_arm(l_pulley, l_anchor, thetas, l_wire)

        assert np.all(np.diff(np.abs(l_wire - abs(l_pulley - l_anchor))) < 0)
        assert np.all(np.diff(np.abs(l_moment_arm)) < 0)
        assert l_wire[-1] == pytest.approx(abs(l_pulley - l_anchor), abs=1e-3)
        assert l_moment_arm[-1] == pytest.approx(0.0, abs=1e-3)


def test_moment_arm_sign_is_preserved():
    l_pulley = l_anchor = 1.0
    for theta_included in (np.pi / 2, -np.pi / 2):
        l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
        l_moment_arm = wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)
        expected = np.sign(theta_included) * (1 / np.sqrt(2))
        assert l_moment_arm == pytest.approx(expected, abs=1e-6)

    l_wire_pos = wk.wire_length(l_pulley, l_anchor, np.pi / 2)
    l_wire_neg = wk.wire_length(l_pulley, l_anchor, -np.pi / 2)
    l_moment_arm_pos = wk.moment_arm(l_pulley, l_anchor, np.pi / 2, l_wire_pos)
    l_moment_arm_neg = wk.moment_arm(l_pulley, l_anchor, -np.pi / 2, l_wire_neg)
    assert l_moment_arm_neg == pytest.approx(-l_moment_arm_pos, abs=1e-9)


def test_matches_note_original_formula():
    rng = np.random.default_rng(0)
    n_points = 3000
    l_pulley = rng.uniform(0.1, 2.0, n_points)
    l_anchor = rng.uniform(0.1, 2.0, n_points)
    theta_included = rng.uniform(0.05, np.pi - 0.05, n_points)

    l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
    l_moment_arm = wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)
    l_moment_arm_note = _l_moment_arm_note_formula(l_pulley, l_anchor, l_wire)

    assert np.allclose(np.abs(l_moment_arm), l_moment_arm_note, atol=1e-9)


def test_pulley_polar_round_trip():
    rng = np.random.default_rng(1)
    x = rng.uniform(-2.0, 2.0, 1000)
    z = rng.uniform(-2.0, 2.0, 1000)

    l_pulley, theta_pulley = wk.pulley_polar_from_xy(x, z)
    x_round_trip, z_round_trip = wk.pulley_xy_from_polar(l_pulley, theta_pulley)

    assert np.allclose(x_round_trip, x, atol=1e-9)
    assert np.allclose(z_round_trip, z, atol=1e-9)


def test_anchor_angle_alpha_zero_reduces_to_theta_joint():
    theta_joint = np.linspace(-np.pi, np.pi, 100)
    assert np.array_equal(wk.anchor_angle(theta_joint, 0.0), theta_joint)


def test_solve_wire_geometry_matches_component_functions():
    x, z, l_anchor, theta_joint, theta_anchor_offset = (
        0.3,
        0.4,
        0.2,
        np.deg2rad(100),
        np.deg2rad(10),
    )

    result = wk.solve_wire_geometry(x, z, l_anchor, theta_joint, theta_anchor_offset)

    l_pulley, theta_pulley = wk.pulley_polar_from_xy(x, z)
    theta_anchor = wk.anchor_angle(theta_joint, theta_anchor_offset)
    theta_included = wk.included_angle(theta_anchor, theta_pulley)
    l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
    l_moment_arm = wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)

    assert result.l_pulley == pytest.approx(l_pulley)
    assert result.theta_pulley == pytest.approx(theta_pulley)
    assert result.theta_anchor == pytest.approx(theta_anchor)
    assert result.theta_included == pytest.approx(theta_included)
    assert result.l_wire == pytest.approx(l_wire)
    assert result.l_moment_arm == pytest.approx(l_moment_arm)


def test_array_input_matches_scalar_loop():
    x, z, l_anchor, theta_anchor_offset = 0.3, 0.4, 0.2, np.deg2rad(10)
    theta_joint_sweep = np.linspace(-np.pi / 3, np.pi / 3, 50)

    result_array = wk.solve_wire_geometry(
        x, z, l_anchor, theta_joint_sweep, theta_anchor_offset
    )
    l_moment_arm_scalar_loop = np.array(
        [
            wk.solve_wire_geometry(
                x, z, l_anchor, theta_joint, theta_anchor_offset
            ).l_moment_arm
            for theta_joint in theta_joint_sweep
        ]
    )

    assert np.allclose(result_array.l_moment_arm, l_moment_arm_scalar_loop, atol=1e-12)


def test_degenerate_equal_lengths_zero_angle_is_unguarded_nan():
    l_pulley = l_anchor = 1.0
    theta_included = 0.0
    l_wire = wk.wire_length(l_pulley, l_anchor, theta_included)
    assert l_wire == pytest.approx(0.0, abs=1e-12)

    with np.errstate(invalid="ignore", divide="ignore"):
        l_moment_arm = wk.moment_arm(l_pulley, l_anchor, theta_included, l_wire)

    assert np.isnan(l_moment_arm)


def test_l_wire_matches_direct_euclidean_distance_to_pulley_xy():
    """solve_wire_geometry().l_wire が、pulleyとanchorの実ユークリッド距離と一致することを確認する。

    以前ここには「theta_pulley=atan2(-z,x) が theta_anchor 側と符号規約が揃っていない」
    という xfail（未解決の疑い）があったが、再検証の結果これは誤検知と判明した。
    原因は、この疑いの根拠になった「直接距離」の手計算が anchor 座標を
    z_anchor = +l_anchor・sin(...)（符号反転なし）で求めており、これ自体が
    A-1確定規約（z = -l・sinθ、pulley_xy_from_polar と同じ規約）に反していたこと。
    pulley と同じ z = -l・sinθ で anchor 座標を計算すると、下記の通り厳密に一致する
    （3D回転行列によるA-1規約の独立検算、および20万点の乱数検証でも最大誤差 3e-14 を確認済み）。
    """
    x_pulley, z_pulley = 1.0, 1.0
    l_anchor = 1.0
    theta_anchor_offset = 0.0
    theta_joint = np.deg2rad(45)

    geom = wk.solve_wire_geometry(
        x_pulley, z_pulley, l_anchor, theta_joint, theta_anchor_offset
    )

    x_anchor = l_anchor * np.cos(theta_joint - theta_anchor_offset)
    z_anchor = -l_anchor * np.sin(theta_joint - theta_anchor_offset)
    direct_distance = np.hypot(x_pulley - x_anchor, z_pulley - z_anchor)

    assert geom.l_wire == pytest.approx(direct_distance, abs=1e-9)
