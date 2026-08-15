"""README（../README.md）に埋め込む図を実データ・実結果から生成する。

生成する図（`results/figures/` に保存、gitで追跡）:
    1. excitation_waveform.png  励振トルク波形（multi-sine、フェーズ1の解説用）
    2. fit_comparison.png       同定前後のモデル予測 vs 実測（フェーズ3/4の解説用）

いずれも新しい計算・推定は行わず、既存の実データ（`data/raw/`）と既存の同定結果
（`results/stage3_.../params.yaml`）をそのまま可視化するだけ。数値の根拠は
`identify.py`/`validate.py`/`csv_adapter.py` と同じ。

使い方:
    uv run python plot_results.py
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from mujoco import sysid

# 日本語ラベルの文字化け（tofu）を避けるため、CJK対応フォントを明示的に指定する。
# DejaVu Sans（matplotlib既定）は日本語グリフを持たないため、環境にインストール済みの
# IPAGothic を使う（システムにより異なる場合は環境側のCJKフォント名に読み替えること）。
plt.rcParams["font.family"] = "IPAGothic"
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, str(Path(__file__).resolve().parent))

from csv_adapter import build_sequences  # noqa: E402
from identify import DATA_DIR, DEFAULT_RUNS, MODEL_PATH  # noqa: E402

HERE = Path(__file__).resolve().parent
FIG_DIR = HERE / "results" / "figures"

# フェーズ1の励振式（exp_005_sysid_excitation.py / config.yaml の値をそのまま転記。
# lib.config_loader は実機依存のsys.path設定を伴うためここでは使わず、値を直接書く）
BASE_FREQ = 4.0
AMPLITUDE = 0.9
HARMONIC_RATIOS = (1.0, 3.4, 7.4)
HARMONIC_WEIGHTS = (1.0, 0.6, 0.3)

# フェーズ3ステージ3の同定値（identification/results/stage3_.../params.yaml と同一）
IDENTIFIED = {"armature": 0.012749758164055498, "frictionloss": 0.097735278042639, "damping": 0.02701614746069557}
# 同定前のベースライン（models/ak45_36_joint.xml の初期値。validate.py の BASELINE と同一）
BASELINE = {"armature": 0.01, "frictionloss": 0.0, "damping": 0.0}

JOINT = "ak45_joint"


def multi_sine_torque(t):
    return AMPLITUDE * sum(w * np.sin(2 * np.pi * r * BASE_FREQ * t) for r, w in zip(HARMONIC_RATIOS, HARMONIC_WEIGHTS))


def plot_excitation_waveform():
    """励振トルク波形（合成波 + 3つの成分）を2秒分プロットする。"""
    t = np.linspace(0, 2.0, 4000)
    total = multi_sine_torque(t)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax1.plot(t, total, color="#1f77b4", linewidth=1.5)
    ax1.axhline(0, color="k", linewidth=0.5, alpha=0.4)
    ax1.set_ylabel("desired_torque [Nm]")
    ax1.set_title(f"励振トルク波形（合成波、基準振幅 {AMPLITUDE} Nm・基準周波数 {BASE_FREQ} Hz）")
    ax1.grid(True, alpha=0.3)

    colors = ["#ff7f0e", "#2ca02c", "#d62728"]
    for ratio, weight, color in zip(HARMONIC_RATIOS, HARMONIC_WEIGHTS, colors):
        component = AMPLITUDE * weight * np.sin(2 * np.pi * ratio * BASE_FREQ * t)
        ax2.plot(t, component, color=color, linewidth=1.2, label=f"{ratio}×f (={ratio * BASE_FREQ:g} Hz), 重み{weight}")
    ax2.axhline(0, color="k", linewidth=0.5, alpha=0.4)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("component [Nm]")
    ax2.set_title("内訳（3つの正弦波成分）")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    out = FIG_DIR / "excitation_waveform.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"保存: {out}")


def _build_model(values):
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    joint = spec.joint(JOINT)
    for name, value in values.items():
        if name == "damping":
            joint.damping[0] = value
        else:
            setattr(joint, name, value)
    return spec.compile()


def _rollout(model, states, controls):
    controls = [c.resample(target_dt=model.opt.timestep) for c in controls]
    data = mujoco.MjData(model)
    return sysid.sysid_rollout(model, data, controls, np.array(states))


def plot_fit_comparison(run=DEFAULT_RUNS[0], n_segments=4, seg_start=6):
    """同定前(baseline)・同定後(stage3)のモデル予測を実測と重ねてプロットする。

    区間長0.5sはフェーズ3の同定・フェーズ4の検証と同じ切り出し方（速度ゼロ交差起点）。
    各区間は独立に実測の初期状態から再スタートする（縦の破線が区間の境目）。これは
    「10秒通しで1本のシーケンスとして予測すると開ループの初期状態鋭敏性で誤差が
    再現性の限界を超えて蓄積する」ため、フェーズ3で採用した切り出し方をそのまま
    可視化に使っている（README「なぜ軌道を0.5秒に分割するのか」参照）。
    """
    baseline_model = _build_model(BASELINE)
    identified_model = _build_model(IDENTIFIED)

    csv_path = DATA_DIR / run / "log.csv"
    names, states, controls, sensors = build_sequences(csv_path, identified_model, seg_len=0.5, run_label=run)

    sel = slice(seg_start, seg_start + n_segments)
    states, controls, sensors = states[sel], controls[sel], sensors[sel]

    traj_base = _rollout(baseline_model, states, controls)
    traj_id = _rollout(identified_model, states, controls)

    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    seg_len = 0.5
    for i, (meas, tb, ti) in enumerate(zip(sensors, traj_base, traj_id)):
        offset = i * seg_len
        meas_t = np.asarray(meas.times) + offset
        meas_d = np.asarray(meas.data)
        base_t = np.asarray(tb.sensordata.times) + offset
        base_d = np.asarray(tb.sensordata.data)
        id_t = np.asarray(ti.sensordata.times) + offset
        id_d = np.asarray(ti.sensordata.data)

        label_meas = "実測" if i == 0 else None
        label_base = "同定前（baseline）" if i == 0 else None
        label_id = "同定後（stage3）" if i == 0 else None

        ax_pos.plot(meas_t, meas_d[:, 0], color="k", linewidth=2, label=label_meas)
        ax_pos.plot(base_t, base_d[:, 0], color="#d62728", linewidth=1.3, linestyle="--", label=label_base)
        ax_pos.plot(id_t, id_d[:, 0], color="#1f77b4", linewidth=1.3, label=label_id)

        ax_vel.plot(meas_t, meas_d[:, 1], color="k", linewidth=2, label=label_meas)
        ax_vel.plot(base_t, base_d[:, 1], color="#d62728", linewidth=1.3, linestyle="--", label=label_base)
        ax_vel.plot(id_t, id_d[:, 1], color="#1f77b4", linewidth=1.3, label=label_id)

        if i > 0:
            ax_pos.axvline(offset, color="gray", linewidth=0.7, linestyle=":", alpha=0.6)
            ax_vel.axvline(offset, color="gray", linewidth=0.7, linestyle=":", alpha=0.6)

    ax_pos.set_ylabel("output_angle [rad]")
    ax_pos.set_title(f"同定前後のモデル予測 vs 実測（{run.replace('exp005_sysid_excitation_', 'run')}、0.5s区間×{n_segments}）")
    ax_pos.legend(loc="upper right", fontsize=9)
    ax_pos.grid(True, alpha=0.3)

    ax_vel.set_xlabel("time [s]（区間ごとに実測の初期状態から独立に再スタート、破線=区間境界）")
    ax_vel.set_ylabel("output_velocity [rad/s]")
    ax_vel.legend(loc="upper right", fontsize=9)
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    out = FIG_DIR / "fit_comparison.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"保存: {out}")


if __name__ == "__main__":
    plot_excitation_waveform()
    plot_fit_comparison()
