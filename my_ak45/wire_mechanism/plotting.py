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

機構そのものが動く様子をGIFで見たい場合は `animate_wire_mechanism()` /
`animate_antagonistic_mechanism()` を使う（`matplotlib.animation.PillowWriter` を使うため
追加インストールは不要。`pillow` は matplotlib 経由で既に解決済み）。
"""

import numpy as np

from wire_mechanism.pulley_placement_search import (
    AntagonisticPlacementResult,
    PlacementGridResult,
)
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


def plot_antagonistic_placement_heatmap(
    result: AntagonisticPlacementResult,
    output_file: str | None = None,
) -> tuple:
    """フェーズD 拮抗2本の探索結果（`search_antagonistic_placement()`）をプロットする。

    4次元の探索結果はそのまま描けないので、**ワイヤーA側について周辺化した2次元マップ**
    （各セルにAを置き、Bを最良に選んだときの `max_θ0 T`）をヒートマップにする。
    最適解の組は A を白星、その相方 B を白三角で示し、線で結ぶ。

    Parameters
    ----------
    result : AntagonisticPlacementResult
        `search_antagonistic_placement()` の戻り値。
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

    fig, ax = plt.subplots(figsize=(8, 6))

    masked = np.ma.masked_invalid(np.where(result.feasible, result.max_tension, np.nan))
    im = ax.pcolormesh(
        result.x_grid, result.z_grid, masked, shading="nearest", cmap="viridis"
    )
    fig.colorbar(im, ax=ax, label="best achievable max_θ0 T(θ0) [N]")

    # A候補にならなかったセルのうち、B候補（アームが負）である領域を薄く示す。
    b_only = result.candidate_b & ~result.feasible
    ax.pcolormesh(
        result.x_grid,
        result.z_grid,
        np.ma.masked_where(~b_only, np.ones_like(b_only, dtype=float)),
        shading="nearest",
        cmap=ListedColormap(["tab:orange"]),
        alpha=0.35,
    )

    best = result.best_pair()
    if best is not None:
        (iz_a, ix_a), (iz_b, ix_b) = best
        ax.plot(
            [result.x_grid[ix_a], result.x_grid[ix_b]],
            [result.z_grid[iz_a], result.z_grid[iz_b]],
            "-",
            color="white",
            linewidth=1.5,
            alpha=0.8,
        )
        ax.plot(
            result.x_grid[ix_a],
            result.z_grid[iz_a],
            "*",
            color="white",
            markeredgecolor="black",
            markersize=16,
            label="best wire A",
        )
        ax.plot(
            result.x_grid[ix_b],
            result.z_grid[iz_b],
            "^",
            color="white",
            markeredgecolor="black",
            markersize=11,
            label="its partner B",
        )
        ax.legend(loc="upper right")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title(
        "Phase D: Antagonistic Placement Search (marginalized over wire A)\n"
        "orange = wire-B candidates only (negative moment arm)"
    )
    ax.set_aspect("equal")

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {output_file}")

    return fig, ax


def animate_wire_mechanism(
    x: float,
    z: float,
    l_anchor: float,
    theta_anchor_offset: float,
    theta_range: tuple[float, float] = (-np.pi / 2, np.pi / 2),
    num_points: int = 120,
    tension: np.ndarray | None = None,
    fps: int = 24,
    output_file: str | None = None,
) -> tuple:
    """単方向ワイヤー1本の機構が動く様子を2Dアニメーション（GIF）で表示する。

    関節を原点に固定し、リンク（原点→アンカー点）と定滑車→アンカー点のワイヤーを
    `theta_joint` の掃引に合わせて描き直す。往路→復路で1周期分の往復揺動として
    ループするので、途中で瞬間移動するような不自然な切り替わりは起きない。

    定滑車の丸マーカーは模式的なもので、半径そのものに物理的な意味はない
    （`wire_kinematics.py` のモデルは定滑車接点を点として扱う。実際のプーリー半径・
    ドラム半径を考慮した繰り出し量の変換は未実装、E-2節参照）。

    Parameters
    ----------
    x, z : float
        定滑車の直交座標 [m]
    l_anchor : float
        関節からワイヤー固定位置までの距離 [m]
    theta_anchor_offset : float
        リンクとワイヤー固定位置のなす角 [rad]
    theta_range : tuple[float, float]
        関節角掃引範囲 [rad]（デフォルト: -π/2 to π/2）
    num_points : int
        片道あたりのフレーム数（デフォルト: 120。実際のGIFは往復で約2倍のフレーム数になる）
    tension : np.ndarray | None
        `theta_range` を `num_points` 点で `np.linspace` した掃引に対応する張力 [N]
        （例えば `wire_statics.solve_wire_tension()` の結果）。与えるとワイヤーの色を
        張力でマッピングする（`viridis`）。None ならグレー固定色。
    fps : int
        GIFのフレームレート（デフォルト: 24）
    output_file : str | None
        出力ファイルパス（`.gif` 推奨）。指定すると `PillowWriter` で保存する。
        Noneの場合は保存せず、FuncAnimationオブジェクトを返すのみ。

    Returns
    -------
    tuple
        (fig, ax, anim) — matplotlib の Figure, Axes, FuncAnimation オブジェクト
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        from matplotlib.colors import Normalize
    except ImportError as e:
        raise ImportError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from e

    theta_joint = np.linspace(theta_range[0], theta_range[1], num_points)
    theta_frames = np.concatenate([theta_joint, theta_joint[::-1]])
    theta_anchor = theta_frames - theta_anchor_offset
    x_anchor = l_anchor * np.cos(theta_anchor)
    z_anchor = -l_anchor * np.sin(theta_anchor)

    tension_frames = None
    norm = None
    cmap = None
    if tension is not None:
        tension = np.asarray(tension, dtype=float)
        tension_frames = np.concatenate([tension, tension[::-1]])
        norm = Normalize(vmin=np.nanmin(tension_frames), vmax=np.nanmax(tension_frames))
        cmap = plt.colormaps["viridis"]

    fig, ax = plt.subplots(figsize=(6, 6))

    all_x = np.concatenate([x_anchor, [x, 0.0]])
    all_z = np.concatenate([z_anchor, [z, 0.0]])
    margin = 0.1 * max(all_x.max() - all_x.min(), all_z.max() - all_z.min(), 1e-6)
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_z.min() - margin, all_z.max() + margin)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Wire-Driven Joint Mechanism")
    ax.grid(True, alpha=0.3)

    ax.plot(0, 0, "ko", markersize=8, zorder=5, label="joint")
    ax.plot(
        x,
        z,
        "o",
        color="tab:gray",
        markersize=14,
        markeredgecolor="k",
        zorder=4,
        label="pulley",
    )

    (link_line,) = ax.plot(
        [], [], "-", color="tab:blue", linewidth=3, zorder=3, label="link"
    )
    (wire_line,) = ax.plot(
        [], [], "-", color="tab:gray", linewidth=1.5, zorder=2, label="wire"
    )
    (anchor_point,) = ax.plot([], [], "o", color="tab:blue", markersize=6, zorder=4)
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=10)
    ax.legend(loc="lower right", fontsize=9)

    def update(i):
        link_line.set_data([0, x_anchor[i]], [0, z_anchor[i]])
        wire_line.set_data([x, x_anchor[i]], [z, z_anchor[i]])
        anchor_point.set_data([x_anchor[i]], [z_anchor[i]])
        info = f"θ0 = {np.degrees(theta_frames[i]):+.1f}°"
        if tension_frames is not None:
            wire_line.set_color(cmap(norm(tension_frames[i])))
            info += f"\nT = {tension_frames[i]:.1f} N"
        text.set_text(info)
        return link_line, wire_line, anchor_point, text

    anim = FuncAnimation(
        fig, update, frames=len(theta_frames), interval=1000 / fps, blit=False
    )

    if output_file:
        anim.save(output_file, writer=PillowWriter(fps=fps))
        print(f"Animation saved to: {output_file}")

    return fig, ax, anim


def animate_antagonistic_mechanism(
    x_a: float,
    z_a: float,
    x_b: float,
    z_b: float,
    l_anchor: float,
    theta_anchor_offset_a: float,
    theta_anchor_offset_b: float,
    theta_range: tuple[float, float] = (-np.pi / 2, np.pi / 2),
    num_points: int = 120,
    tension_a: np.ndarray | None = None,
    tension_b: np.ndarray | None = None,
    fps: int = 24,
    output_file: str | None = None,
) -> tuple:
    """拮抗2本の機構が動く様子を2Dアニメーション（GIF）で表示する。

    引数構成は `pulley_placement_search.search_antagonistic_placement()` に対応する。
    ワイヤーA・Bそれぞれのアンカー点を、関節（原点）を頂点とする三角形の辺として描く
    ことで、2本が同じ剛体リンク上の異なる固定点に取り付いていることを表す
    （`theta_anchor_offset_a == theta_anchor_offset_b` なら2辺が重なり、実質1点に見える）。

    Parameters
    ----------
    x_a, z_a, x_b, z_b : float
        ワイヤーA・Bそれぞれの定滑車座標 [m]
    l_anchor : float
        関節からワイヤー固定位置までの距離 [m]（A・B共通）
    theta_anchor_offset_a, theta_anchor_offset_b : float
        ワイヤーA・Bそれぞれのアンカーオフセット角 [rad]
    theta_range, num_points, fps, output_file :
        `animate_wire_mechanism()` と同じ意味
    tension_a, tension_b : np.ndarray | None
        各ワイヤーの張力 [N]（例えば `drive_modes.antagonistic()` の戻り値）。
        与えるとワイヤーAは赤系、ワイヤーBは青系のcolormapで色分けする。

    Returns
    -------
    tuple
        (fig, ax, anim) — matplotlib の Figure, Axes, FuncAnimation オブジェクト
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        from matplotlib.colors import Normalize
    except ImportError as e:
        raise ImportError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from e

    theta_joint = np.linspace(theta_range[0], theta_range[1], num_points)
    theta_frames = np.concatenate([theta_joint, theta_joint[::-1]])
    theta_anchor_a = theta_frames - theta_anchor_offset_a
    theta_anchor_b = theta_frames - theta_anchor_offset_b
    xa_anchor = l_anchor * np.cos(theta_anchor_a)
    za_anchor = -l_anchor * np.sin(theta_anchor_a)
    xb_anchor = l_anchor * np.cos(theta_anchor_b)
    zb_anchor = -l_anchor * np.sin(theta_anchor_b)

    cmap_a = plt.colormaps["autumn"]
    cmap_b = plt.colormaps["winter"]
    tension_a_frames = tension_b_frames = None
    norm_a = norm_b = None
    if tension_a is not None:
        tension_a = np.asarray(tension_a, dtype=float)
        tension_a_frames = np.concatenate([tension_a, tension_a[::-1]])
        norm_a = Normalize(
            vmin=np.nanmin(tension_a_frames), vmax=np.nanmax(tension_a_frames)
        )
    if tension_b is not None:
        tension_b = np.asarray(tension_b, dtype=float)
        tension_b_frames = np.concatenate([tension_b, tension_b[::-1]])
        norm_b = Normalize(
            vmin=np.nanmin(tension_b_frames), vmax=np.nanmax(tension_b_frames)
        )

    fig, ax = plt.subplots(figsize=(6, 6))

    all_x = np.concatenate([xa_anchor, xb_anchor, [x_a, x_b, 0.0]])
    all_z = np.concatenate([za_anchor, zb_anchor, [z_a, z_b, 0.0]])
    margin = 0.1 * max(all_x.max() - all_x.min(), all_z.max() - all_z.min(), 1e-6)
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_z.min() - margin, all_z.max() + margin)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Antagonistic Wire-Driven Joint Mechanism")
    ax.grid(True, alpha=0.3)

    ax.plot(0, 0, "ko", markersize=8, zorder=5, label="joint")
    ax.plot(
        x_a,
        z_a,
        "o",
        color="tab:red",
        markersize=12,
        markeredgecolor="k",
        zorder=4,
        label="pulley A",
    )
    ax.plot(
        x_b,
        z_b,
        "o",
        color="tab:blue",
        markersize=12,
        markeredgecolor="k",
        zorder=4,
        label="pulley B",
    )

    (link_shape,) = ax.plot(
        [], [], "-", color="0.3", linewidth=2, zorder=3, label="link"
    )
    (wire_line_a,) = ax.plot(
        [], [], "-", color="tab:red", linewidth=1.5, zorder=2, label="wire A"
    )
    (wire_line_b,) = ax.plot(
        [], [], "-", color="tab:blue", linewidth=1.5, zorder=2, label="wire B"
    )
    (anchor_a,) = ax.plot([], [], "o", color="tab:red", markersize=6, zorder=4)
    (anchor_b,) = ax.plot([], [], "o", color="tab:blue", markersize=6, zorder=4)
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9)
    ax.legend(loc="lower right", fontsize=8)

    def update(i):
        link_shape.set_data(
            [0, xa_anchor[i], xb_anchor[i], 0], [0, za_anchor[i], zb_anchor[i], 0]
        )
        wire_line_a.set_data([x_a, xa_anchor[i]], [z_a, za_anchor[i]])
        wire_line_b.set_data([x_b, xb_anchor[i]], [z_b, zb_anchor[i]])
        anchor_a.set_data([xa_anchor[i]], [za_anchor[i]])
        anchor_b.set_data([xb_anchor[i]], [zb_anchor[i]])
        info = f"θ0 = {np.degrees(theta_frames[i]):+.1f}°"
        if tension_a_frames is not None:
            wire_line_a.set_color(cmap_a(norm_a(tension_a_frames[i])))
            info += f"\nT_a = {tension_a_frames[i]:.1f} N"
        if tension_b_frames is not None:
            wire_line_b.set_color(cmap_b(norm_b(tension_b_frames[i])))
            info += f"\nT_b = {tension_b_frames[i]:.1f} N"
        text.set_text(info)
        return link_shape, wire_line_a, wire_line_b, anchor_a, anchor_b, text

    anim = FuncAnimation(
        fig, update, frames=len(theta_frames), interval=1000 / fps, blit=False
    )

    if output_file:
        anim.save(output_file, writer=PillowWriter(fps=fps))
        print(f"Animation saved to: {output_file}")

    return fig, ax, anim


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
