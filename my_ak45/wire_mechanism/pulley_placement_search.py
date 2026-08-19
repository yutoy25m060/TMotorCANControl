"""定滑車位置 (x, z) のグリッド探索（フェーズD）。

`my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部 フェーズD「定滑車位置の探索」に対応する。A-2 の2つの駆動方式それぞれに
対応する探索関数を持つ:

    - `search_unidirectional_placement()`: 単方向ワイヤー1本。探索次元2 (x, z)。
    - `search_antagonistic_placement()`:   拮抗2本。探索次元4 (x_a, z_a, x_b, z_b)。

A-2（どちらの方式を採るか）は目標揺動周波数の確定待ちで未決着のため
（`.ai/logs/2026-08-13_09_a2-drive-mode-reevaluation_01.md`）、**両方式とも探索できる
状態にしてある**。仮の動作仕様は `assumed_params.py` を参照。

実装した制約 (D-2) と実装しなかった制約（両関数に共通）:
    - `l5 → 0` 近傍の特異点 (8-2): 実装（`l_moment_arm_min` で判定）。
    - `T >= tension_min` (8-1、たるみ防止込み): 実装
      （`drive_modes.unidirectional()` / `drive_modes.antagonistic()` を再利用）。
    - **ワイヤーとリンクの非干渉 (8-3) は未実装**。ドキュメントに具体的なリンク形状
      （太さ・断面）の定義がなく、非干渉判定に必要な幾何情報が本リポジトリにまだ無いため。
      本モジュールが返す `feasible` を「実機で組める」の意味に使わないこと。
      **拮抗2本ではワイヤーが2本になるぶん、この未実装の影響がより大きい**
      （ワイヤー同士の干渉も判定していない）。
    - **物理的な取り付け可能性（フレーム外形との整合）も未実装**。同上の理由。

D-1（評価指標）:
    単方向では2指標とも計算する（ドキュメントの「必ず一方に決める、あるいは両方計算して
    比較する」という指示に従う）:
    - `max_θ0 T(θ0)` の最小化（推奨・第一候補） — `PlacementGridResult.max_tension`
    - `T(θ0)` のレンジ（max-min）の最小化 — `PlacementGridResult.tension_range`

    拮抗2本では **レンジ指標を提供しない**。`drive_modes.antagonistic()` の張力配分は
    従動側を常に `tension_min` に固定するため、2本を合わせた最小張力は恒等的に
    `tension_min` になり、レンジ = `max_tension − tension_min` と
    `max_tension` に順位同値（独立した指標にならない）ことを数値確認済み
    （`tests/test_pulley_placement_search.py::test_antagonistic_range_metric_would_be_rank_equivalent`）。

トルク需要 `tau_external_sweep` は呼び出し側が用意する（`wire_statics.gravity_torque()` や
`drive_modes.wire_torque_demand()` の結果をそのまま渡せる）。本モジュールはどの物理モデルを
使うかに依存しない。
"""

from dataclasses import dataclass

import numpy as np

from wire_mechanism import drive_modes as dm
from wire_mechanism.wire_kinematics import FloatOrArray, solve_wire_geometry


@dataclass(frozen=True, slots=True)
class PlacementGridResult:
    """search_unidirectional_placement() の戻り値（ヒートマップ描画を想定した形状）。

    全ての2次元配列は shape=(len(z_grid), len(x_grid))。`origin_exclusion_radius`
    未満（定滑車が関節直上）のセルは `singular=True`・他は NaN のまま残る。
    """

    x_grid: np.ndarray
    z_grid: np.ndarray
    max_tension: np.ndarray
    tension_range: np.ndarray
    feasible: np.ndarray
    singular: np.ndarray
    slack_or_reversed: np.ndarray

    def best_by_max_tension(self) -> tuple[int, int] | None:
        """D-1 第一候補: 実現可能なセルのうち max_θ0 T(θ0) が最小のセルの (iz, ix) を返す。"""
        return self._best_index(self.max_tension)

    def best_by_tension_range(self) -> tuple[int, int] | None:
        """D-1 第二候補: 実現可能なセルのうち T(θ0) のレンジが最小のセルの (iz, ix) を返す。"""
        return self._best_index(self.tension_range)

    def _best_index(self, metric: np.ndarray) -> tuple[int, int] | None:
        if not self.feasible.any():
            return None
        masked = np.where(self.feasible, metric, np.inf)
        iz, ix = np.unravel_index(np.argmin(masked), masked.shape)
        return int(iz), int(ix)


def search_unidirectional_placement(
    x_grid: np.ndarray,
    z_grid: np.ndarray,
    l_anchor: float,
    theta_anchor_offset: float,
    theta_joint_sweep: FloatOrArray,
    tau_external_sweep: FloatOrArray,
    l_moment_arm_min: float = 1e-4,
    tension_min: float = 0.0,
    origin_exclusion_radius: float = 1e-6,
) -> PlacementGridResult:
    """単方向ワイヤー1本を仮定し、定滑車位置 (x, z) を粗いグリッドで探索する（D-3）。

    `l_anchor`・`theta_anchor_offset` は固定（グリッド探索の対象は x, z の2次元のみ）。
    `theta_joint_sweep` の関節可動域全体で `T = tau_external_sweep / l_moment_arm` を
    評価し、8-2（特異点）・8-1（T>=tension_min）の両制約を満たすセルだけを `feasible=True`
    とする。8-3（非干渉）は評価しない（モジュールdocstring参照）。
    """
    theta_joint_sweep = np.asarray(theta_joint_sweep, dtype=float)
    tau_external_sweep = np.asarray(tau_external_sweep, dtype=float)
    x_grid = np.asarray(x_grid, dtype=float)
    z_grid = np.asarray(z_grid, dtype=float)

    shape = (len(z_grid), len(x_grid))
    max_tension = np.full(shape, np.nan)
    tension_range = np.full(shape, np.nan)
    feasible = np.zeros(shape, dtype=bool)
    singular = np.zeros(shape, dtype=bool)
    slack_or_reversed = np.zeros(shape, dtype=bool)

    for iz, z in enumerate(z_grid):
        for ix, x in enumerate(x_grid):
            if np.hypot(x, z) < origin_exclusion_radius:
                singular[iz, ix] = True
                continue

            with np.errstate(invalid="ignore", divide="ignore"):
                geom = solve_wire_geometry(
                    x, z, l_anchor, theta_joint_sweep, theta_anchor_offset
                )
            abs_arm = np.abs(geom.l_moment_arm)
            # 完全な退化(l_wire=0によるNaN)も特異点として扱う（NaN比較は常にFalseになるため
            # np.min()任せにはできない）。
            is_singular = bool(
                np.any(~np.isfinite(abs_arm)) or np.min(abs_arm) < l_moment_arm_min
            )
            singular[iz, ix] = is_singular

            with np.errstate(invalid="ignore", divide="ignore"):
                result = dm.unidirectional(
                    tau_external_sweep, geom.l_moment_arm, tension_min
                )
            max_tension[iz, ix] = result.tension_max
            tension_range[iz, ix] = result.tension_max - result.tension_min
            slack_or_reversed[iz, ix] = not result.feasible
            feasible[iz, ix] = result.feasible and not is_singular

    return PlacementGridResult(
        x_grid=x_grid,
        z_grid=z_grid,
        max_tension=max_tension,
        tension_range=tension_range,
        feasible=feasible,
        singular=singular,
        slack_or_reversed=slack_or_reversed,
    )


def _moment_arm_grid(
    x_grid: np.ndarray,
    z_grid: np.ndarray,
    l_anchor: float,
    theta_anchor_offset: float,
    theta_joint_sweep: np.ndarray,
    origin_exclusion_radius: float,
) -> np.ndarray:
    """全グリッドセルのモーメントアーム時系列を shape=(nz, nx, n_sweep) で返す。

    原点近傍のセル（定滑車が関節位置と重なる）は NaN で埋める。退化配置由来の NaN も
    そのまま残すので、呼び出し側は比較演算（NaN は常に False）でふるい落とせる。
    """
    arms = np.full(
        (len(z_grid), len(x_grid), len(theta_joint_sweep)), np.nan, dtype=float
    )
    for iz, z in enumerate(z_grid):
        for ix, x in enumerate(x_grid):
            if np.hypot(x, z) < origin_exclusion_radius:
                continue
            with np.errstate(invalid="ignore", divide="ignore"):
                arms[iz, ix] = solve_wire_geometry(
                    x, z, l_anchor, theta_joint_sweep, theta_anchor_offset
                ).l_moment_arm
    return arms


@dataclass(frozen=True, slots=True)
class AntagonisticPlacementResult:
    """search_antagonistic_placement() の戻り値。

    4次元の探索結果をそのまま返すと巨大になるため、**ワイヤーA側の配置について周辺化した
    2次元マップ**を返す。すなわち `max_tension[iz, ix]` は「ワイヤーAをこのセルに置いたとき、
    ワイヤーBを全候補から最良に選べた場合に達成できる最小の最大張力」。その最良の相方は
    `partner_iz`/`partner_ix` で引ける（成立する相方が無いセルは -1）。
    全体最適の組は `best_pair()` で取得する。

    全ての2次元配列は shape=(len(z_grid), len(x_grid))。
    `candidate_a` / `candidate_b` は、そのセルが可動域全体でモーメントアームの符号を
    正 / 負に保てる（かつ特異点に触れない）ワイヤーA / B の候補かどうかを表す。
    """

    x_grid: np.ndarray
    z_grid: np.ndarray
    max_tension: np.ndarray
    partner_iz: np.ndarray
    partner_ix: np.ndarray
    candidate_a: np.ndarray
    candidate_b: np.ndarray
    feasible: np.ndarray

    def best_pair(self) -> tuple[tuple[int, int], tuple[int, int]] | None:
        """max_θ0 T の最小を与える (A側(iz,ix), B側(iz,ix)) を返す。成立する組が無ければ None。"""
        if not self.feasible.any():
            return None
        masked = np.where(self.feasible, self.max_tension, np.inf)
        iz_a, ix_a = np.unravel_index(np.argmin(masked), masked.shape)
        return (int(iz_a), int(ix_a)), (
            int(self.partner_iz[iz_a, ix_a]),
            int(self.partner_ix[iz_a, ix_a]),
        )


def search_antagonistic_placement(
    x_grid: np.ndarray,
    z_grid: np.ndarray,
    l_anchor: float,
    theta_anchor_offset_a: float,
    theta_anchor_offset_b: float,
    theta_joint_sweep: FloatOrArray,
    tau_external_sweep: FloatOrArray,
    l_moment_arm_min: float = 1e-4,
    tension_min: float = 0.0,
    origin_exclusion_radius: float = 1e-6,
) -> AntagonisticPlacementResult:
    """拮抗2本を仮定し、定滑車位置の組 (x_a, z_a, x_b, z_b) を探索する（D-3、探索次元4）。

    ワイヤーAは可動域全体でモーメントアームが `>= +l_moment_arm_min`、ワイヤーBは
    `<= -l_moment_arm_min` である配置のみを候補とする（`drive_modes.antagonistic()` が
    前提とする `l_a > 0 > l_b` の符号条件）。トルク需要の符号が反転しても、符号の異なる
    2本で分担できるのが拮抗方式の要点。

    `theta_anchor_offset_a` / `_b` は2本それぞれのワイヤー固定位置のオフセット角で、
    グリッド探索の対象ではない（探索するのは定滑車の (x, z) ×2 の4次元のみ）。
    2本を同一点に固定する構成なら同じ値を、リンクの反対側に固定する構成なら
    `theta_anchor_offset_b = theta_anchor_offset_a + np.pi` のように指定する。

    張力配分そのものは `drive_modes.antagonistic()` に委ね（両側に `tension_min` の下駄を
    履かせてから残差を主動側に配分する方式）、本関数は候補の組み合わせに対する
    ふるい分けと集約のみを行う。

    計算量は「A候補数 × B候補数」だが、B側は候補をまとめて1回の配列演算で評価するので、
    25×25 グリッド・721点掃引でも1秒未満で完了する。
    """
    theta_joint_sweep = np.asarray(theta_joint_sweep, dtype=float)
    tau_external_sweep = np.asarray(tau_external_sweep, dtype=float)
    x_grid = np.asarray(x_grid, dtype=float)
    z_grid = np.asarray(z_grid, dtype=float)

    arms_a = _moment_arm_grid(
        x_grid,
        z_grid,
        l_anchor,
        theta_anchor_offset_a,
        theta_joint_sweep,
        origin_exclusion_radius,
    )
    arms_b = _moment_arm_grid(
        x_grid,
        z_grid,
        l_anchor,
        theta_anchor_offset_b,
        theta_joint_sweep,
        origin_exclusion_radius,
    )

    # NaN との比較は常に False になるので、退化・原点セルはここで自動的に候補から外れる。
    candidate_a = np.all(arms_a >= l_moment_arm_min, axis=-1)
    candidate_b = np.all(arms_b <= -l_moment_arm_min, axis=-1)

    shape = (len(z_grid), len(x_grid))
    max_tension = np.full(shape, np.nan)
    partner_iz = np.full(shape, -1, dtype=int)
    partner_ix = np.full(shape, -1, dtype=int)
    feasible = np.zeros(shape, dtype=bool)

    b_indices = np.argwhere(candidate_b)
    if len(b_indices) == 0:
        return AntagonisticPlacementResult(
            x_grid=x_grid,
            z_grid=z_grid,
            max_tension=max_tension,
            partner_iz=partner_iz,
            partner_ix=partner_ix,
            candidate_a=candidate_a,
            candidate_b=candidate_b,
            feasible=feasible,
        )
    arms_b_flat = arms_b[candidate_b]  # (n_b, n_sweep)

    for iz_a, ix_a in np.argwhere(candidate_a):
        arm_a = arms_a[iz_a, ix_a][None, :]
        with np.errstate(invalid="ignore", divide="ignore"):
            _, tension_a, tension_b = dm.antagonistic(
                tau_external_sweep, arm_a, arms_b_flat, tension_min
            )

        ok = (
            np.isfinite(tension_a).all(axis=-1)
            & np.isfinite(tension_b).all(axis=-1)
            & (tension_a >= tension_min - 1e-12).all(axis=-1)
            & (tension_b >= tension_min - 1e-12).all(axis=-1)
        )
        # 2本のうち大きい方の最大張力がその組の評価値（D-1第一候補）。
        pair_max = np.maximum(tension_a.max(axis=-1), tension_b.max(axis=-1))
        pair_max = np.where(ok, pair_max, np.inf)

        best_b = int(np.argmin(pair_max))
        if not np.isfinite(pair_max[best_b]):
            continue
        max_tension[iz_a, ix_a] = pair_max[best_b]
        partner_iz[iz_a, ix_a] = int(b_indices[best_b][0])
        partner_ix[iz_a, ix_a] = int(b_indices[best_b][1])
        feasible[iz_a, ix_a] = True

    return AntagonisticPlacementResult(
        x_grid=x_grid,
        z_grid=z_grid,
        max_tension=max_tension,
        partner_iz=partner_iz,
        partner_ix=partner_ix,
        candidate_a=candidate_a,
        candidate_b=candidate_b,
        feasible=feasible,
    )
