"""pulley_placement_search.py（フェーズD、単方向ワイヤー1本限定）のユニットテスト。"""

import numpy as np
import pytest
from wire_mechanism import drive_modes as dm
from wire_mechanism import pulley_placement_search as pps
from wire_mechanism.wire_kinematics import solve_wire_geometry
from wire_mechanism.wire_statics import gravity_torque

L_ANCHOR = 0.05


def test_grid_shapes_match_input_grids():
    x_grid = np.linspace(0.05, 0.20, 4)
    z_grid = np.linspace(-0.10, 0.10, 3)
    theta = np.linspace(-np.pi / 3, np.pi / 3, 51)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, 0.0, theta, tau
    )

    assert result.max_tension.shape == (3, 4)
    assert result.tension_range.shape == (3, 4)
    assert result.feasible.shape == (3, 4)
    assert result.singular.shape == (3, 4)
    assert result.slack_or_reversed.shape == (3, 4)


def test_origin_cell_is_singular_and_infeasible():
    x_grid = np.array([0.0, 0.10])
    z_grid = np.array([0.0])
    theta = np.linspace(-np.pi / 4, np.pi / 4, 21)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, 0.0, theta, tau
    )

    assert result.singular[0, 0]
    assert not result.feasible[0, 0]
    assert np.isnan(result.max_tension[0, 0])


def test_near_singularity_cell_flagged_singular():
    """theta_joint=0 を含む掃引で x=l_anchor, z=0 の配置は theta_included=0 を通り l5=0(NaN)になる。

    z=0.2 の配置は theta_pulley が 0 からずれるため、同じ掃引でも特異点を踏まない。
    """
    x_grid = np.array([L_ANCHOR])
    z_grid = np.array([0.0, 0.2])
    theta = np.linspace(-np.pi / 6, np.pi / 6, 21)  # 0 を含む
    tau = np.ones_like(theta)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, 0.0, theta, tau, l_moment_arm_min=1e-4
    )

    assert result.singular[0, 0]
    assert not result.feasible[0, 0]
    assert not result.singular[1, 0]


def test_tension_min_marks_range_end_slack_as_infeasible():
    """掃引の両端でトルク需要が0に近づく形状にすると、tension_min>0でたるみによりinfeasibleになる。

    配置(x=0.30, z=0, offset=-90度)は theta_included が [60,120]度に収まりモーメントアームが
    特異点から十分離れる（8-2とは無関係）。tau は掃引の両端で0になる合成波形にしてあるので、
    tension_min=0では成立するが tension_min=5では両端でたるみ、infeasibleになる。
    """
    x_grid = np.array([0.30])
    z_grid = np.array([0.0])
    offset = np.deg2rad(-90)
    theta = np.linspace(-np.pi / 6, np.pi / 6, 121)
    tau = 10.0 * np.cos(theta / theta.max() * (np.pi / 2))

    lax = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, offset, theta, tau, tension_min=0.0
    )
    strict = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, offset, theta, tau, tension_min=5.0
    )

    assert lax.feasible[0, 0]
    assert not strict.feasible[0, 0]
    assert strict.slack_or_reversed[0, 0]
    assert not strict.singular[0, 0]


def test_max_tension_and_range_match_manual_drive_modes_call():
    x_grid = np.array([0.15, 0.25])
    z_grid = np.array([-0.10])
    theta = np.linspace(-np.pi / 3, np.pi / 3, 41)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, 0.0, theta, tau
    )

    for ix, x in enumerate(x_grid):
        geom = solve_wire_geometry(x, z_grid[0], L_ANCHOR, theta, 0.0)
        expected = dm.unidirectional(tau, geom.l_moment_arm, 0.0)
        assert result.max_tension[0, ix] == pytest.approx(expected.tension_max)
        assert result.tension_range[0, ix] == pytest.approx(
            expected.tension_max - expected.tension_min
        )
        assert result.feasible[0, ix] == expected.feasible


def test_best_by_max_tension_picks_the_smallest_feasible_cell():
    """offset=-90度で theta_included を[60,120]度付近に保ち、全セルが8-2の特異点を避ける配置にする。"""
    x_grid = np.linspace(0.15, 0.35, 5)
    z_grid = np.linspace(-0.05, 0.05, 3)
    offset = np.deg2rad(-90)
    theta = np.linspace(-np.pi / 6, np.pi / 6, 61)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, offset, theta, tau
    )

    best = result.best_by_max_tension()
    assert best is not None
    iz, ix = best
    assert result.feasible[iz, ix]
    feasible_values = np.where(result.feasible, result.max_tension, np.inf)
    assert result.max_tension[iz, ix] == pytest.approx(feasible_values.min())


def test_best_by_tension_range_is_independent_metric_from_best_by_max_tension():
    """max_tensionとtension_rangeは別々の指標として、それぞれ自分の指標での最小値を返す。"""
    x_grid = np.linspace(0.15, 0.35, 5)
    z_grid = np.linspace(-0.05, 0.05, 3)
    offset = np.deg2rad(-90)
    theta = np.linspace(-np.pi / 6, np.pi / 6, 61)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, offset, theta, tau
    )

    best_max = result.best_by_max_tension()
    best_range = result.best_by_tension_range()
    assert best_max is not None
    assert best_range is not None
    feasible_max = np.where(result.feasible, result.max_tension, np.inf)
    feasible_range = np.where(result.feasible, result.tension_range, np.inf)
    assert result.max_tension[best_max] == pytest.approx(feasible_max.min())
    assert result.tension_range[best_range] == pytest.approx(feasible_range.min())


def test_best_returns_none_when_nothing_feasible():
    x_grid = np.array([0.0])
    z_grid = np.array([0.0])
    theta = np.linspace(-np.pi / 4, np.pi / 4, 11)
    tau = gravity_torque(theta, mass=1.0, l_com=0.15)

    result = pps.search_unidirectional_placement(
        x_grid, z_grid, L_ANCHOR, 0.0, theta, tau
    )

    assert result.best_by_max_tension() is None
    assert result.best_by_tension_range() is None


def test_call_pattern_reproduces_8_5_moment_arm_upper_bound():
    """D-3の指示: l5_max = min(l2,l3) が本モジュールと同じ呼び出しパターンで再現されるか確認する。

    再現されなければ、solve_wire_geometry の呼び出し方（引数の順序・符号規約）に
    誤りがあることを意味する（8-5節参照）。
    """
    l_pulley = 3 * L_ANCHOR
    # theta_anchor_offset=0, z=0（theta_pulley=0）なら theta_included = theta_joint になる。
    theta_included_sweep = np.linspace(1e-3, np.pi - 1e-3, 4000)

    geom = solve_wire_geometry(l_pulley, 0.0, L_ANCHOR, theta_included_sweep, 0.0)

    expected_l5_max = min(l_pulley, L_ANCHOR)
    assert np.max(np.abs(geom.l_moment_arm)) == pytest.approx(expected_l5_max, abs=1e-3)
