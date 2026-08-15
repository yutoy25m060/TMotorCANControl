"""ワイヤー駆動関節の Phase B/C 計算結果を静的プロットする（matplotlib）。

このモジュールは matplotlib を必要とします。別途インストール:
    pip install matplotlib

使用例:
    from wire_mechanism.plotting import plot_wire_geometry_phase_bc

    fig, axes = plot_wire_geometry_phase_bc(
        x=0.2, z=-0.15,
        l_anchor=0.2,
        theta_anchor_offset=0.0,
        mass=0.5,
        l_com=0.1,
        g=9.8,
        theta_range=(-np.pi/2, np.pi/2),
        output_file="phase_bc_plot.png"
    )
"""

import numpy as np

from wire_mechanism.pulley_placement_search import PlacementGridResult
from wire_mechanism.wire_kinematics import solve_wire_geometry
from wire_mechanism.wire_statics import gravity_torque, solve_wire_tension


def plot_wire_geometry_phase_bc(
    x: float,
    z: float,
    l_anchor: float,
    theta_anchor_offset: float,
    mass: float,
    l_com: float,
    g: float = 9.8,
    theta_range: tuple[float, float] = (-np.pi / 2, np.pi / 2),
    num_points: int = 200,
    l_moment_arm_min: float = 1e-4,
    output_file: str | None = None,
) -> tuple:
    """Phase B/C の4つの曲線を同時にプロットする。

    Parameters
    ----------
    x, z : float
        定滑車の直交座標 [m]
    l_anchor : float
        関節からワイヤー固定位置までの距離 [m]
    theta_anchor_offset : float
        リンクとワイヤー固定位置のなす角 [rad]
    mass : float
        リンク質量 [kg]
    l_com : float
        関節から重心までの距離 [m]
    g : float
        重力加速度 [m/s²]（デフォルト: 9.8）
    theta_range : tuple[float, float]
        関節角掃引範囲 [rad]（デフォルト: -π/2 to π/2）
    num_points : int
        プロット用のサンプル点数（デフォルト: 200）
    l_moment_arm_min : float
        モーメントアーム判定の閾値 [m]（デフォルト: 1e-4）
    output_file : str | None
        出力ファイルパス（PNG等）。Noneの場合は表示のみ

    Returns
    -------
    tuple
        (fig, axes) — matplotlib の Figure と Axes オブジェクト
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise ImportError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from e

    theta_joint = np.linspace(theta_range[0], theta_range[1], num_points)
    theta_joint_deg = np.degrees(theta_joint)

    # Phase B: 幾何計算
    geom = solve_wire_geometry(
        x=x,
        z=z,
        l_anchor=l_anchor,
        theta_joint=theta_joint,
        theta_anchor_offset=theta_anchor_offset,
    )

    # Phase C: 静力学計算
    tau_gravity_vals = gravity_torque(theta_joint, mass, l_com, g)
    tension_result = solve_wire_tension(
        tau_gravity_vals, geom.l_moment_arm, l_moment_arm_min
    )

    # プロット設定
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"Wire-Driven Joint: Phase B/C Analysis\n"
        f"Pulley at (x={x:.3f}, z={z:.3f}) m, l_anchor={l_anchor:.3f} m, "
        f"M={mass:.2f} kg, l_com={l_com:.3f} m",
        fontsize=14,
        fontweight="bold",
    )

    # --- Plot 1: l_wire vs θ₀
    ax1 = axes[0, 0]
    ax1.plot(theta_joint_deg, geom.l_wire, "b-", linewidth=2, label="l_wire")
    ax1.axhline(y=0, color="k", linestyle="--", linewidth=0.5, alpha=0.3)
    ax1.set_xlabel("θ₀ [deg]", fontsize=11)
    ax1.set_ylabel("l_wire [m]", fontsize=11)
    ax1.set_title("Phase B: Wire Length vs Joint Angle", fontsize=12, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)

    # --- Plot 2: l_moment_arm vs θ₀ (signed)
    ax2 = axes[0, 1]
    ax2.plot(
        theta_joint_deg, geom.l_moment_arm, "r-", linewidth=2, label="l_moment_arm"
    )
    ax2.axhline(y=0, color="k", linestyle="--", linewidth=1, alpha=0.5)
    ax2.axhline(
        y=l_moment_arm_min,
        color="orange",
        linestyle=":",
        linewidth=1,
        label=f"threshold (+{l_moment_arm_min:.4f})",
    )
    ax2.axhline(
        y=-l_moment_arm_min,
        color="orange",
        linestyle=":",
        linewidth=1,
        label=f"threshold (-{l_moment_arm_min:.4f})",
    )
    ax2.set_xlabel("θ₀ [deg]", fontsize=11)
    ax2.set_ylabel("l_moment_arm [m]", fontsize=11)
    ax2.set_title(
        "Phase B: Moment Arm vs Joint Angle (signed)", fontsize=12, fontweight="bold"
    )
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    # --- Plot 3: τ_gravity vs θ₀
    ax3 = axes[1, 0]
    ax3.plot(theta_joint_deg, tau_gravity_vals, "g-", linewidth=2, label="τ_gravity")
    ax3.axhline(y=0, color="k", linestyle="--", linewidth=1, alpha=0.5)
    ax3.set_xlabel("θ₀ [deg]", fontsize=11)
    ax3.set_ylabel("τ_gravity [Nm]", fontsize=11)
    ax3.set_title(
        "Phase C: Gravity Torque vs Joint Angle", fontsize=12, fontweight="bold"
    )
    ax3.fill_between(theta_joint_deg, 0, tau_gravity_vals, alpha=0.2, color="green")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=10)

    # --- Plot 4: T vs θ₀ with feasibility coloring
    ax4 = axes[1, 1]

    # feasible 領域は紫、infeasible は赤
    feasible_mask = np.asarray(tension_result.feasible, dtype=bool)
    if isinstance(feasible_mask, np.ndarray):
        feasible_idx = np.where(feasible_mask)[0]
        infeasible_idx = np.where(~feasible_mask)[0]

        if len(feasible_idx) > 0:
            ax4.plot(
                theta_joint_deg[feasible_idx],
                tension_result.tension[feasible_idx],
                "o-",
                color="purple",
                linewidth=2,
                markersize=4,
                label="feasible",
            )
        if len(infeasible_idx) > 0:
            ax4.plot(
                theta_joint_deg[infeasible_idx],
                tension_result.tension[infeasible_idx],
                "o",
                color="red",
                linewidth=2,
                markersize=6,
                label="infeasible",
            )
    else:
        color = "purple" if feasible_mask else "red"
        ax4.plot(theta_joint_deg, tension_result.tension, "-", color=color, linewidth=2)

    ax4.axhline(y=0, color="k", linestyle="--", linewidth=1, alpha=0.5)
    ax4.set_xlabel("θ₀ [deg]", fontsize=11)
    ax4.set_ylabel("T (wire tension) [N]", fontsize=11)
    ax4.set_title(
        "Phase C: Required Wire Tension vs Joint Angle", fontsize=12, fontweight="bold"
    )
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=10)

    plt.tight_layout()

    # ファイルに保存
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {output_file}")

    return fig, axes


def plot_pulley_placement_heatmap(
    result: PlacementGridResult,
    metric: str = "max_tension",
    output_file: str | None = None,
) -> tuple:
    """フェーズD `pulley_placement_search.search_unidirectional_placement()` の結果を
    ヒートマップ表示する（D-3: 制約違反領域はマスクして色分けする）。

    Parameters
    ----------
    result : PlacementGridResult
        `search_unidirectional_placement()` の戻り値。
    metric : str
        表示する指標。`"max_tension"`（D-1第一候補）または `"tension_range"`（D-1第二候補）。
    output_file : str | None
        出力ファイルパス（PNG等）。Noneの場合は表示のみ。

    Returns
    -------
    tuple
        (fig, ax) — matplotlib の Figure と Axes オブジェクト
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError as e:
        raise ImportError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from e

    if metric == "max_tension":
        values = result.max_tension
        label = "max_θ0 T(θ0) [N]"
    elif metric == "tension_range":
        values = result.tension_range
        label = "T(θ0) range [N]"
    else:
        raise ValueError(
            f"unknown metric: {metric!r} (expected 'max_tension' or 'tension_range')"
        )

    fig, ax = plt.subplots(figsize=(8, 6))

    masked = np.ma.masked_invalid(np.where(result.feasible, values, np.nan))
    im = ax.pcolormesh(
        result.x_grid, result.z_grid, masked, shading="nearest", cmap="viridis"
    )
    fig.colorbar(im, ax=ax, label=label)

    # 制約違反の理由ごとに色分け（8-2: 特異点、8-1/たるみ: T<tension_min）
    violation = np.zeros(result.singular.shape, dtype=float)
    violation[result.singular] = 1.0
    violation[result.slack_or_reversed & ~result.singular] = 2.0
    violation_cmap = ListedColormap(["none", "black", "red"])
    ax.pcolormesh(
        result.x_grid,
        result.z_grid,
        np.ma.masked_equal(violation, 0.0),
        shading="nearest",
        cmap=violation_cmap,
        vmin=0,
        vmax=2,
        alpha=0.6,
    )

    best = (
        result.best_by_max_tension()
        if metric == "max_tension"
        else result.best_by_tension_range()
    )
    if best is not None:
        iz, ix = best
        ax.plot(
            result.x_grid[ix],
            result.z_grid[iz],
            "*",
            color="white",
            markeredgecolor="black",
            markersize=16,
            label="best",
        )
        ax.legend(loc="upper right")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title(
        f"Phase D: Pulley Placement Search ({label})\n"
        "black = singular (8-2), red = slack/T<0 (8-1)"
    )
    ax.set_aspect("equal")

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {output_file}")

    return fig, ax


# ===== Convenience script runner =====

if __name__ == "__main__":
    # 標準的なテスト用パラメータ
    print("Generating Phase B/C plot with standard parameters...")

    fig, axes = plot_wire_geometry_phase_bc(
        x=0.2,
        z=-0.15,
        l_anchor=0.2,
        theta_anchor_offset=0.0,
        mass=0.5,
        l_com=0.1,
        g=9.8,
        theta_range=(-np.pi / 2, np.pi / 2),
        num_points=200,
        output_file="phase_bc_plot.png",
    )

    try:
        import matplotlib.pyplot as plt

        plt.show()
    except Exception as e:
        print(f"Could not display plot: {e}")
