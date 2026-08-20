"""フェーズA-2（駆動方式の決定）の定量比較スクリプト。

`my_ak45/wire_drive/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md` A-2節に
記載した数値を再現する。`python -m wire_mechanism.a2_drive_mode_comparison`
（`my_ak45/` をカレントまたは PYTHONPATH に置いて）で実行する。

前提としている仮置き値の根拠は A-2節の「仮置きした動作仕様」表を参照。
"""

import numpy as np

from wire_mechanism import assumed_params as ap
from wire_mechanism import drive_modes as dm
from wire_mechanism.wire_kinematics import solve_wire_geometry

# --- 仮置きパラメータ（A-2節の表と対応） -------------------------------------
# LINK・TENSION_MIN は assumed_params.py（正式決定待ちの仮値の一元管理先）を参照する。
LINK = ap.ASSUMED_LINK
L_ANCHOR = 0.05  # 関節からワイヤー固定位置までの距離 [m]
V_MAX_MOTOR = 6.0  # AK45-36 出力軸側の速度上限 [rad/s]（MIT_Params、実機で裏取り済み）
TAU_MOTOR_RATED = 8.0  # AK45-36 定格トルク [Nm]（公式基本仕様）
L_ARM_MIN = 0.005  # 特異点回避のためのモーメントアーム下限 [m]
TENSION_MIN = ap.ASSUMED_TENSION_MIN

# 定滑車配置の探索グリッド
GRID_X = np.linspace(-0.30, 0.30, 25)
GRID_Z = np.linspace(-0.30, 0.30, 25)
GRID_ALPHA = np.deg2rad(np.linspace(-180, 180, 37))


def _search(theta, objective, want_positive_arm):
    """可動域全体でモーメントアームの符号が一定な配置のうち、objective を最小化するものを返す。

    objective(arm) は小さいほど良い評価値を返す関数（除外したい配置には np.inf を返す）。
    戻り値は (評価値, x, z, α[deg], arm) または None。
    """
    best = None
    with np.errstate(divide="ignore", invalid="ignore"):
        for x in GRID_X:
            for z in GRID_Z:
                if np.hypot(x, z) < 1e-6:
                    continue
                for alpha in GRID_ALPHA:
                    arm = solve_wire_geometry(x, z, L_ANCHOR, theta, alpha).l_moment_arm
                    if want_positive_arm:
                        if not (arm >= L_ARM_MIN).all():
                            continue
                    elif not (arm <= -L_ARM_MIN).all():
                        continue
                    value = objective(arm)
                    if not np.isfinite(value):
                        continue
                    if best is None or value < best[0]:
                        best = (float(value), x, z, np.rad2deg(alpha), arm)
    return best


def _single_wire_objective(demand, tension_min):
    """単方向構成の目的関数: 全時刻で T>=tension_min を満たす前提で最大張力を最小化。"""

    def objective(arm):
        res = dm.unidirectional(demand, arm, tension_min)
        return res.tension_max if res.feasible else np.inf

    return objective


def _antagonist_side_objective(demand, active_mask):
    """拮抗の片側の目的関数: 自分が主動になる区間での最大張力のみを最小化する。

    従動区間ではプリテンションに固定されるだけなので、T>=T_min の判定はここではしない
    （その扱いは dm.antagonistic() 側が担う）。
    """

    def objective(arm):
        if not active_mask.any():
            # 需要の符号が反転しない条件では従動側は常にプリテンション。
            # 万一の反転に備えてモーメントアームが大きい配置を選んでおく。
            return float(1.0 / np.abs(arm).min())
        tension = demand[active_mask] / arm[active_mask]
        if not np.isfinite(tension).all():
            return np.inf
        return float(tension.max())

    return objective


def section_1_speed_limit():
    print("=" * 78)
    print("1. モーター速度上限から決まる関節速度・揺動周波数の上限")
    print("   (V_max = 6.0 rad/s 出力軸側, モーメントアーム l5 = 0.05 m)")
    print("=" * 78)
    print(
        " r_drum[mm] | 関節角速度上限[rad/s] | ±60°揺動の上限周波数[Hz] | T=30N時のモータtrq[Nm]"
    )
    for r_mm in [10, 20, 30, 40, 60, 80]:
        r = r_mm / 1000.0
        dtheta_max = dm.max_joint_speed(0.05, r, V_MAX_MOTOR)
        f_max = dtheta_max / (np.deg2rad(60) * 2 * np.pi)
        tau_m = dm.motor_torque(30.0, r)
        print(
            f"   {r_mm:5d}    |        {dtheta_max:6.2f}         |"
            f"          {f_max:5.2f}          |       {tau_m:5.2f}"
            f"{'  ← 定格超過' if tau_m > TAU_MOTOR_RATED else ''}"
        )

    # 速度制約と動力学制約（単方向1本の破綻周波数）が入れ替わるドラム半径
    f_dynamic = _breaking_frequency(np.deg2rad(60), np.full(721, 0.05))
    amp = np.deg2rad(60)
    r_cross = f_dynamic * 0.05 * amp * 2 * np.pi / V_MAX_MOTOR
    print()
    print(f"   ±60°揺動における単方向1本の動力学的な破綻周波数 = {f_dynamic:.2f} Hz")
    print(
        f"   → r_drum < {r_cross * 1000:.0f} mm では速度上限が先に効き、動力学的破綻には到達しない"
    )
    print(
        f"     r_drum > {r_cross * 1000:.0f} mm にして初めて『単方向では不可能』な領域に入る"
    )
    print()


def _breaking_frequency(amplitude, arm, f_step=0.01, f_stop=4.0):
    """単方向1本が T>=0 を満たせなくなる下限周波数を返す（見つからなければ inf）。"""
    for f in np.arange(f_step, f_stop, f_step):
        theta, _, ddtheta = dm.MotionSpec(amplitude, f).sample()
        if not dm.unidirectional(
            dm.wire_torque_demand(theta, ddtheta, LINK), arm, 0.0
        ).feasible:
            return float(f)
    return float("inf")


def section_2_feasibility_vs_frequency():
    print("=" * 78)
    print(
        "2. 揺動周波数に対する各方式の成立性（振幅±60°、モーメントアーム0.05m一定で近似）"
    )
    print(f"   最低張力 T_min = {TENSION_MIN} N（たるみ防止）を要求")
    print("=" * 78)
    print(" f[Hz] | 単方向1本      | 単方向+バネ(1.0Nm) | 拮抗2本")
    for f in [0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0]:
        theta, _, ddtheta = dm.MotionSpec(np.deg2rad(60), f).sample()
        arm_a = np.full_like(theta, 0.05)
        arm_b = np.full_like(theta, -0.05)

        uni = dm.unidirectional(
            dm.wire_torque_demand(theta, ddtheta, LINK), arm_a, TENSION_MIN
        )
        spr = dm.unidirectional(
            dm.wire_torque_demand(theta, ddtheta, LINK, tau_spring=1.0),
            arm_a,
            TENSION_MIN,
        )
        ant, _, _ = dm.antagonistic(
            dm.wire_torque_demand(theta, ddtheta, LINK), arm_a, arm_b, TENSION_MIN
        )

        def fmt(r):
            return (
                f"OK({r.tension_max:5.1f}N)"
                if r.feasible
                else f"NG({r.infeasible_fraction * 100:4.1f}%)"
            )

        print(f" {f:5.2f} | {fmt(uni):14} | {fmt(spr):18} | {fmt(ant)}")
    print("   ※ OK(...)内は最大張力、NG(...)内は成立しない時間の割合")
    print()


def section_3_breaking_frequency():
    print("=" * 78)
    print("3. 単方向1本が破綻する周波数（振幅別、T_min=0の緩い条件でも破綻する点）")
    print("=" * 78)
    print(" 振幅[deg] | 破綻下限周波数[Hz] | 同時刻の関節角速度[rad/s]")
    for amp_deg in [30, 45, 60, 80, 90]:
        amp = np.deg2rad(amp_deg)
        f_break = _breaking_frequency(amp, np.full(721, 0.05))
        if not np.isfinite(f_break):
            print(f"   {amp_deg:4d}    |     4.0Hz超まで成立     |")
        elif f_break <= 0.02:
            # 端点で tau_gravity=0 になる振幅では、いくら遅くても慣性項が勝つ
            print(
                f"   {amp_deg:4d}    |   任意の非ゼロ周波数で破綻  |  (端点で重力トルク=0のため)"
            )
        else:
            print(
                f"   {amp_deg:4d}    |       {f_break:5.2f}        |"
                f"        {amp * 2 * np.pi * f_break:5.2f}"
            )
    print()


def section_4_placement_optimized_comparison():
    print("=" * 78)
    print("4. 定滑車配置を各方式ごとに最適化した比較（振幅±60°）")
    print(
        f"   grid: x,z ∈ [-0.3,0.3] 25点, α ∈ [-180°,180°] 37点, l_anchor={L_ANCHOR}m"
    )
    print(f"   T_min = {TENSION_MIN} N, モーメントアーム下限 {L_ARM_MIN} m")
    print("=" * 78)

    for f in [0.0, 1.0]:
        label_f = "準静的 (f=0)" if f == 0.0 else f"揺動 f={f}Hz"
        print(f"--- {label_f} ---")
        theta, _, ddtheta = dm.MotionSpec(np.deg2rad(60), f).sample()
        demand = dm.wire_torque_demand(theta, ddtheta, LINK)
        demand_spring = dm.wire_torque_demand(theta, ddtheta, LINK, tau_spring=1.0)

        def show(label, best):
            if best is None:
                print(f"  {label:22}: 成立する配置なし")
                return None
            t, x, z, a, _arm = best
            print(
                f"  {label:22}: maxT={t:7.2f}N  配置(x={x:+.3f}, z={z:+.3f}, α={a:+4.0f}°)"
            )
            return best

        show(
            "単方向1本",
            _search(theta, _single_wire_objective(demand, TENSION_MIN), True),
        )
        show(
            "単方向+バネ(1.0Nm)",
            _search(theta, _single_wire_objective(demand_spring, TENSION_MIN), True),
        )

        # 拮抗: 各ワイヤーを「自分が主動になる区間」で最適化してから合成評価する
        # （プリテンション由来の相互干渉は2次の効果なので、合成評価で確認する）
        best_a = _search(theta, _antagonist_side_objective(demand, demand >= 0.0), True)
        best_b = _search(theta, _antagonist_side_objective(demand, demand < 0.0), False)
        if best_a is None or best_b is None:
            print(f"  {'拮抗2本':22}: 成立する配置なし")
        else:
            res, _, _ = dm.antagonistic(demand, best_a[4], best_b[4], TENSION_MIN)
            print(
                f"  {'拮抗2本(2本合計)':22}: maxT={res.tension_max:7.2f}N  "
                f"feasible={res.feasible}  "
                f"[A:(x={best_a[1]:+.3f},z={best_a[2]:+.3f},α={best_a[3]:+4.0f}°) "
                f"B:(x={best_b[1]:+.3f},z={best_b[2]:+.3f},α={best_b[3]:+4.0f}°)]"
            )
        print()


def main():
    print()
    print(
        f"リンク: M={LINK.mass}kg, l_com={LINK.l_com}m, I={LINK.inertia:.4f}kg·m² (一様棒)"
    )
    print(f"       重力トルク最大 = {LINK.mass * LINK.g * LINK.l_com:.3f} Nm")
    print()
    section_1_speed_limit()
    section_2_feasibility_vs_frequency()
    section_3_breaking_frequency()
    section_4_placement_optimized_comparison()


if __name__ == "__main__":
    main()
