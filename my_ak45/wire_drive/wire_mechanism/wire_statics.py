"""ワイヤー駆動関節の静力学（フェーズC）。

`my_ak45/wire_drive/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部 フェーズC「準静的な τ → T の算出」に基づく。まずは重力のみの
準静的モデル（第2部 8-4）を対象とし、動力学項の追加はフェーズE-1で行う。
`gravity_torque()` を将来の逆動力学（ニュートン・オイラー法）の結果に
差し替える、または加算しても `solve_wire_tension()` はそのまま使える設計にしている。

符号規約の導出（wire_kinematics.py の l_moment_arm との整合性）:
    仮想仕事の原理より、ワイヤー張力 T が theta_joint 方向に及ぼす一般化トルクは
        tau_wire = -T・d(l_wire)/d(theta_joint)
    である（ワイヤーが縮む＝l_wireが減るとき張力は正の仕事をするため）。

    d(l_wire)/d(theta_joint) = +l_moment_arm であることを有限差分で数値検証済み
    （test_wire_statics.py::test_moment_arm_equals_positive_wire_length_derivative、
    最大誤差1e-9）。これは wire_length の余弦定理の式を theta_included で
    直接微分すると l_pulley・l_anchor・sin(theta_included)/l_wire（= moment_arm の定義式
    そのもの）になり、theta_included = theta_anchor − theta_pulley が theta_joint に対して
    傾き+1の線形関数である（pulley側は theta_joint に依存しない）ことから導かれる恒等式。

    したがって tau_wire = -T・l_moment_arm。静的つり合い
        tau_external + tau_wire = 0
    を解くと
        T = tau_external / l_moment_arm
    となり、ノートの `τ = T・l5`（`τ` = tau_external）とも一致する
    （途中に符号反転を挟む必要はない）。

既知の課題（解決済み・誤検知と判明）:
    以前ここには、wire_kinematics.pulley_polar_from_xy() の
    theta_pulley = atan2(-z, x) が theta_anchor 側の符号規約と揃っていない疑いを
    記録していたが、再検証の結果これは誤検知と判明した。疑いの根拠になった
    「直接ユークリッド距離」の手計算（test_wire_kinematics.py の該当テスト）が
    anchor 座標を z_anchor = +l_anchor・sin(...)（符号反転なし）で計算しており、
    これ自体が A-1確定規約（z = -l・sinθ）に反する誤った比較対象だったことが原因。
    pulley と同じ z = -l・sinθ 規約で anchor 座標を計算すると、
    solve_wire_geometry().l_wire は実ユークリッド距離と厳密に一致する
    （3D回転行列によるA-1規約の独立検算、および20万点の乱数検証で最大誤差 3e-14 を確認）。
    詳細は test_wire_kinematics.py::test_l_wire_matches_direct_euclidean_distance_to_pulley_xy
    を参照。theta_pulley の実装に修正は不要だった。
"""

from dataclasses import dataclass

import numpy as np

from wire_mechanism.wire_kinematics import FloatOrArray


def gravity_torque(
    theta_joint: FloatOrArray,
    mass: float,
    l_com: float,
    g: float = 9.8,
) -> FloatOrArray:
    """重力が theta_joint 方向に及ぼす一般化トルクを求める: +mass・g・l_com・cos(theta_joint)。

    重心はリンク上、関節から l_com の距離にあると仮定する（A-1確定規約と同じ
    z = -l・sinθ のパラメータ化: z_com = -l_com・sin(theta_joint)、theta_joint=0 が
    x軸正方向）。wire_kinematics.pulley_xy_from_polar() と同一の符号規約であることに注意
    （以前の実装は z_com = +l_com・sin(theta_joint) という非A-1準拠の符号を使っており、
    tau_gravity の符号・安定平衡点の位置が逆転するバグがあった。3D回転行列によるA-1規約の
    独立検算で修正済み）。
    tau_gravity = -dV/d(theta_joint) として導出。
    theta_joint = +90°（z軸負方向、鉛直下向き）が安定平衡点、
    theta_joint = -90°（鉛直上向き）が不安定平衡点になることを数値確認済み。
    """
    return mass * g * l_com * np.cos(theta_joint)


@dataclass(frozen=True, slots=True)
class WireTensionResult:
    """solve_wire_tension() の戻り値。"""

    tension: FloatOrArray
    feasible: FloatOrArray


def solve_wire_tension(
    tau_external: FloatOrArray,
    l_moment_arm: FloatOrArray,
    l_moment_arm_min: float = 1e-4,
) -> WireTensionResult:
    """外力による一般化トルクとモーメントアームから必要ワイヤー張力を求める。

    T = tau_external / l_moment_arm（導出はモジュールdocstring参照。符号反転は不要）。

    以下のいずれかに該当する場合は feasible=False とする
    （A-1確定規約 E3: NaN/infではなく専用のブールフラグで実現不可能性を表現）:
    - `abs(l_moment_arm) < l_moment_arm_min`（特異点近傍、第2部8-2）
    - 得られる T が負（ワイヤーは引く方向にしか力を出せない、第2部8-1）

    tension 自体は生の除算結果をそのまま返す（NaN/infになりうる）。
    feasible を必ず確認してから使うこと。
    """
    l_moment_arm = np.asarray(l_moment_arm, dtype=float)
    tau_external = np.asarray(tau_external, dtype=float)

    with np.errstate(divide="ignore", invalid="ignore"):
        tension = tau_external / l_moment_arm

    feasible = (np.abs(l_moment_arm) >= l_moment_arm_min) & (tension >= 0.0)

    if tension.ndim == 0:
        return WireTensionResult(tension=float(tension), feasible=bool(feasible))
    return WireTensionResult(tension=tension, feasible=feasible)


def solve_static_tension_gravity(
    theta_joint: FloatOrArray,
    l_moment_arm: FloatOrArray,
    mass: float,
    l_com: float,
    g: float = 9.8,
    l_moment_arm_min: float = 1e-4,
) -> WireTensionResult:
    """重力のみの準静的モデルで、theta_joint掃引に対する必要張力を一括で求める（C-1〜C-2の一括版）。"""
    tau_gravity = gravity_torque(theta_joint, mass, l_com, g)
    return solve_wire_tension(tau_gravity, l_moment_arm, l_moment_arm_min)
