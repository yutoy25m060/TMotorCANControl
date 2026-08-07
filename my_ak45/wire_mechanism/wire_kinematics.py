"""ワイヤー駆動関節の幾何計算（フェーズB）。

`my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部 A-1「確定した規約」に基づく純粋関数群。実機・CANバスに非依存。

記号対応（ノート → 本モジュール）:
    l2 → l_pulley       関節から定滑車接点までの距離
    l3 → l_anchor       関節からワイヤー固定位置までの距離
    l4 → l_wire         ワイヤー長さ（定滑車接点〜ワイヤー固定位置）
    l5 → l_moment_arm   張力のモーメントアーム（符号付き）
    θ0 → theta_joint    関節角度（x軸正方向が0、CCWが正）
    α  → theta_anchor_offset  リンクとl3のなす角
    θ1 → theta_anchor   x軸正方向とl3のなす角（= θ0 − α）
    θ2 → theta_pulley   x軸正方向とl2のなす角
    θ3 → theta_included l2とl3のなす角（= θ1 − θ2）
"""

from dataclasses import dataclass

import numpy as np

FloatOrArray = float | np.ndarray


def pulley_polar_from_xy(
    x: FloatOrArray, z: FloatOrArray
) -> tuple[FloatOrArray, FloatOrArray]:
    """定滑車接点の直交座標(x, z)を極座標(l_pulley, theta_pulley)に変換する。

    z = -l_pulley・sin(theta_pulley) の符号規約（A-1確定規約）に対応するため、
    theta_pulley は atan2(z, x) ではなく atan2(-z, x) で求める。
    """
    l_pulley = np.hypot(x, z)
    theta_pulley = np.arctan2(-z, x)
    return l_pulley, theta_pulley


def pulley_xy_from_polar(
    l_pulley: FloatOrArray, theta_pulley: FloatOrArray
) -> tuple[FloatOrArray, FloatOrArray]:
    """pulley_polar_from_xy の逆変換。"""
    x = l_pulley * np.cos(theta_pulley)
    z = -l_pulley * np.sin(theta_pulley)
    return x, z


def anchor_angle(
    theta_joint: FloatOrArray, theta_anchor_offset: FloatOrArray
) -> FloatOrArray:
    """関節角とオフセット角から、ワイヤー固定位置(l_anchor)方向角を求める: θ1 = θ0 − α。"""
    return theta_joint - theta_anchor_offset


def included_angle(
    theta_anchor: FloatOrArray, theta_pulley: FloatOrArray
) -> FloatOrArray:
    """l_pulleyとl_anchorのなす挟角を求める: θ3 = θ1 − θ2。"""
    return theta_anchor - theta_pulley


def wire_length(
    l_pulley: FloatOrArray, l_anchor: FloatOrArray, theta_included: FloatOrArray
) -> FloatOrArray:
    """余弦定理でワイヤー長さ l_wire を求める。"""
    return np.sqrt(
        l_pulley**2 + l_anchor**2 - 2 * l_pulley * l_anchor * np.cos(theta_included)
    )


def moment_arm(
    l_pulley: FloatOrArray,
    l_anchor: FloatOrArray,
    theta_included: FloatOrArray,
    l_wire: FloatOrArray,
) -> FloatOrArray:
    """符号付きモーメントアーム l_moment_arm = l_pulley・l_anchor・sin(θ3) / l_wire を求める。

    符号は保持する（abs()やsqrt()で潰さない）。後段フェーズでの T >= 0 実現可否判定に必要。
    """
    return l_pulley * l_anchor * np.sin(theta_included) / l_wire


@dataclass(frozen=True, slots=True)
class WireGeometry:
    """ワイヤー幾何量一式（solve_wire_geometry の戻り値）。"""

    l_pulley: FloatOrArray
    theta_pulley: FloatOrArray
    theta_anchor: FloatOrArray
    theta_included: FloatOrArray
    l_wire: FloatOrArray
    l_moment_arm: FloatOrArray


def solve_wire_geometry(
    x: FloatOrArray,
    z: FloatOrArray,
    l_anchor: FloatOrArray,
    theta_joint: FloatOrArray,
    theta_anchor_offset: FloatOrArray,
) -> WireGeometry:
    """定滑車座標・関節角からワイヤー幾何量一式を計算する（フェーズB本体）。

    theta_joint 等はスカラー・ndarrayのいずれも可（フェーズCのθ0掃引で配列を渡す想定）。
    """
    l_pulley, theta_pulley = pulley_polar_from_xy(x, z)
    theta_anchor = anchor_angle(theta_joint, theta_anchor_offset)
    theta_included = included_angle(theta_anchor, theta_pulley)
    l_wire = wire_length(l_pulley, l_anchor, theta_included)
    l_moment_arm = moment_arm(l_pulley, l_anchor, theta_included, l_wire)
    return WireGeometry(
        l_pulley=l_pulley,
        theta_pulley=theta_pulley,
        theta_anchor=theta_anchor,
        theta_included=theta_included,
        l_wire=l_wire,
        l_moment_arm=l_moment_arm,
    )
