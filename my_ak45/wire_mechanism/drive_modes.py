"""駆動方式（単方向1本 / 単方向＋バネ / 拮抗2本）の比較（フェーズA-2の再検討）。

`my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`
第3部 A-2「駆動方式の決定（単方向 or 拮抗）」に対応する。

wire_statics.py が扱う準静的モデル（重力のみ、加速度ゼロ）を動力学まで拡張し、
「ワイヤーは引く方向にしか力を出せない（T >= 0、第2部8-1）」という制約が
実際の揺動動作で破れるかどうかを判定できるようにしたもの。

動力学の導出（wire_statics.py の符号規約をそのまま拡張）:
    wire_statics より、ワイヤー張力 T が theta_joint 方向に及ぼす一般化トルクは
        tau_wire = -T * l_moment_arm
    である。関節まわりの運動方程式は
        I * ddtheta = tau_external + tau_wire = tau_external - T * l_moment_arm
    したがって
        T = (tau_external - I * ddtheta) / l_moment_arm
    となる。分子を「ワイヤーが受け持つべきトルク需要」と呼び、本モジュールでは
    `wire_torque_demand()` が返す。ddtheta = 0 とすれば wire_statics の
    `T = tau_external / l_moment_arm` に一致する（テストで確認済み）。

    `I` は「関節まわりのリンク慣性」であって、モーターのロータ慣性ではない点に注意。
    ワイヤー駆動ではモーターは関節から離れた位置にあり、関節の受動的な運動方程式に
    ロータ慣性は直接現れない（ロータ慣性はワイヤーを介してモーター側に効いてくる）。

速度制約について:
    ワイヤー全長のうち関節側で変化する分は l_wire だけであり、
    d(l_wire)/d(theta_joint) = +l_moment_arm（wire_statics.py で数値検証済み）なので、
    ドラム半径 r_drum のモーターから見た角速度は
        omega_motor = l_moment_arm * dtheta_joint / r_drum
    となる。AK45-36 の出力軸側速度上限 V_max（`MIT_Params["AK45-36"]["V_max"] = 6.0` rad/s、
    実機ログで裏取り済み）から、関節側の到達可能な角速度上限が決まる。
"""

from dataclasses import dataclass

import numpy as np

from wire_mechanism.wire_kinematics import FloatOrArray
from wire_mechanism.wire_statics import gravity_torque


@dataclass(frozen=True, slots=True)
class LinkSpec:
    """駆動対象リンクの物理諸元。

    inertia は「関節軸まわりのリンク慣性モーメント」。一様棒を関節端まわりで
    回す場合は (1/3)*mass*length**2 になる（点質量近似 mass*l_com**2 より大きい）。
    """

    mass: float
    l_com: float
    inertia: float
    g: float = 9.8


@dataclass(frozen=True, slots=True)
class MotionSpec:
    """正弦揺動の動作仕様: theta(t) = center + amplitude * sin(2*pi*frequency*t)。"""

    amplitude: float
    frequency: float
    center: float = 0.0

    def sample(
        self, num_points: int = 721
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """1周期分の (theta, dtheta, ddtheta) を返す。"""
        if self.frequency <= 0.0:
            # 準静的（周波数ゼロ）: 可動域を掃引するだけで加速度・速度はゼロ扱い。
            theta = np.linspace(
                self.center - self.amplitude, self.center + self.amplitude, num_points
            )
            zeros = np.zeros_like(theta)
            return theta, zeros, zeros
        w = 2.0 * np.pi * self.frequency
        t = np.linspace(0.0, 1.0 / self.frequency, num_points)
        phase = np.sin(w * t)
        theta = self.center + self.amplitude * phase
        dtheta = self.amplitude * w * np.cos(w * t)
        ddtheta = -self.amplitude * w**2 * phase
        return theta, dtheta, ddtheta


def wire_torque_demand(
    theta: FloatOrArray,
    ddtheta: FloatOrArray,
    link: LinkSpec,
    tau_spring: FloatOrArray = 0.0,
) -> FloatOrArray:
    """ワイヤーが受け持つべきトルク需要 D = tau_gravity + tau_spring - I*ddtheta を返す。

    tau_spring は「重力と同じ回転方向に働く受動要素のトルク」（>= 0 で重力を助長し、
    ワイヤーの負担を増やす代わりにワイヤーのたるみを防ぐ）。単方向＋バネ方式で使う。
    """
    tau_g = gravity_torque(theta, link.mass, link.l_com, link.g)
    return tau_g + tau_spring - link.inertia * np.asarray(ddtheta, dtype=float)


@dataclass(frozen=True, slots=True)
class DriveModeResult:
    """各駆動方式の評価結果。

    tension_max は「実現可能な場合の各ワイヤー張力の最大値」。実現不可能な場合
    (feasible=False) でも参考値として算出した値を入れる（判断には feasible を使うこと）。
    """

    feasible: bool
    tension_max: float
    tension_min: float
    infeasible_fraction: float


def _summarize(
    tensions: tuple[np.ndarray, ...], tension_min_required: float
) -> DriveModeResult:
    stacked = np.stack(tensions)
    # NaN/inf（特異点）は実現不可能として扱う
    valid = np.isfinite(stacked).all(axis=0)
    ok = valid & (stacked >= tension_min_required - 1e-12).all(axis=0)
    finite = stacked[:, valid]
    return DriveModeResult(
        feasible=bool(ok.all()),
        tension_max=float(finite.max()) if finite.size else float("inf"),
        tension_min=float(finite.min()) if finite.size else float("-inf"),
        infeasible_fraction=float(1.0 - ok.mean()),
    )


def unidirectional(
    demand: np.ndarray, l_moment_arm: np.ndarray, tension_min: float = 0.0
) -> DriveModeResult:
    """単方向ワイヤー1本: T = D / l_moment_arm が全時刻で tension_min 以上か。"""
    with np.errstate(divide="ignore", invalid="ignore"):
        tension = np.asarray(demand) / np.asarray(l_moment_arm)
    return _summarize((tension,), tension_min)


def antagonistic(
    demand: np.ndarray,
    l_moment_arm_a: np.ndarray,
    l_moment_arm_b: np.ndarray,
    tension_min: float = 0.0,
) -> tuple[DriveModeResult, np.ndarray, np.ndarray]:
    """拮抗2本: 符号が逆のモーメントアームを持つ2本で D を分担する。

    D = T_a*l_a + T_b*l_b（`l_a > 0 > l_b` を想定）は T_a, T_b の2自由度に対して
    1本の式なので冗長。両ワイヤーが必ず tension_min 以上になる最小張力解を採る:

        T_a = tension_min + s_a,  T_b = tension_min + s_b   (s_a, s_b >= 0)
        D' = D - tension_min*(l_a + l_b)
        D' >= 0 なら A が主動: s_a = D'/l_a, s_b = 0
        D' <  0 なら B が主動: s_a = 0,      s_b = D'/l_b   （l_b<0 なので s_b>0）

    単に「従動側を tension_min に固定する」だけだと、需要 D が 0 に近い区間で
    主動側の張力が tension_min を下回ってしまう（|l_b| < l_a のとき顕著）ため、
    上記のように両側に tension_min の下駄を履かせてから残差を配分する。

    戻り値は (評価結果, T_a, T_b)。
    """
    demand = np.asarray(demand, dtype=float)
    l_a = np.asarray(l_moment_arm_a, dtype=float)
    l_b = np.asarray(l_moment_arm_b, dtype=float)

    with np.errstate(divide="ignore", invalid="ignore"):
        residual = demand - tension_min * (l_a + l_b)
        a_leads = residual >= 0.0
        tension_a = tension_min + np.where(a_leads, residual / l_a, 0.0)
        tension_b = tension_min + np.where(a_leads, 0.0, residual / l_b)

    return _summarize((tension_a, tension_b), tension_min), tension_a, tension_b


def max_joint_speed(
    l_moment_arm: FloatOrArray,
    r_drum: float,
    motor_speed_max: float,
) -> FloatOrArray:
    """モーター速度上限から決まる関節角速度の上限 [rad/s]。

    omega_motor = l_moment_arm * dtheta_joint / r_drum <= motor_speed_max より
        |dtheta_joint| <= motor_speed_max * r_drum / |l_moment_arm|
    モーメントアームが大きいほど関節は遅くなる（トルクとのトレードオフ）。
    """
    return motor_speed_max * r_drum / np.abs(np.asarray(l_moment_arm, dtype=float))


def motor_torque(tension: FloatOrArray, r_drum: float) -> FloatOrArray:
    """ワイヤー張力からモーター出力軸トルク [Nm] を求める: tau_motor = T * r_drum。"""
    return np.asarray(tension, dtype=float) * r_drum
