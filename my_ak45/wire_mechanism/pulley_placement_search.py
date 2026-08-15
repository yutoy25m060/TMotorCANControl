"""定滑車位置 (x, z) のグリッド探索（フェーズD、単方向ワイヤー1本限定）。

`my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部 フェーズD「定滑車位置の探索」に対応する。

スコープ（重要）:
    `.ai/logs/2026-08-13_09_a2-drive-mode-reevaluation_01.md` の通り、フェーズA-2
    （駆動方式: 単方向1本 or 拮抗2本）は目標揺動周波数・振幅が未確定のため確定していない。
    拮抗2本を採る場合は探索次元が (x1,z1,x2,z2) の4次元に増え、この2次元グリッド探索とは
    別実装が要る。本モジュールは **A-2 の暫定結論である単方向1本（探索次元2）に限定**して
    実装したもの。拮抗2本用の探索は、A-2 が確定してから別途実装する。

実装した制約 (D-2) と実装しなかった制約:
    - `l5 → 0` 近傍の特異点 (8-2): 実装（`l_moment_arm_min` で判定）。
    - `T >= tension_min` (8-1、たるみ防止込み): 実装（`drive_modes.unidirectional()` を再利用）。
    - **ワイヤーとリンクの非干渉 (8-3) は未実装**。ドキュメントに具体的なリンク形状
      （太さ・断面）の定義がなく、非干渉判定に必要な幾何情報が本リポジトリにまだ無いため。
      本モジュールが返す `feasible` を「実機で組める」の意味に使わないこと。
    - **物理的な取り付け可能性（フレーム外形との整合）も未実装**。同上の理由。

D-1（評価指標）は2つを両方計算する（ドキュメントの「必ず一方に決める、あるいは両方計算して
比較する」という指示に従う）:
    - `max_θ0 T(θ0)` の最小化（推奨・第一候補） — `PlacementGridResult.max_tension`
    - `T(θ0)` のレンジ（max-min）の最小化 — `PlacementGridResult.tension_range`

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
