"""同定したパラメータを、同定に使っていないランで検証する（leave-one-run-out 交差検証）。

作業手順書 `docs_syid/AK45-36_sysid_作業手順.md` フェーズ4 の項目17・18 のうち、
**PC単独で完結する部分**に対応する。

## なぜ交差検証なのか

フェーズ3の同定は採用3ランを全て投入して行っており、報告されるコストは学習データ上の
値でしかない。「パラメータが多いほどコストが下がる」のは自明なので、ステージ2
（armature+frictionloss）とステージ3（+damping）のどちらを採るかを学習コストで
決めることはできない。同定に使っていないデータでの誤差を見て初めて、
damping が実機の再現に寄与しているのか、学習データのノイズを拾っているだけなのかが分かる。

3本を「2本で同定 → 残り1本で評価」と回し（3通り）、各ステージの汎化誤差を比べる。

## 評価指標

`sysid.optimize` が最小化するコストは、センサー列ごとにその列自身のRMSで正規化された
無次元量であり、学習データと検証データで正規化の分母が変わるため直接は比較しにくい。
そこでここでは**物理単位のRMS誤差**（位置 [rad]・速度 [rad/s]）を主指標にする。
予測と実測の時刻合わせは `sysid.model_residual` が内部で行うのと同じ手順
（予測の時間範囲へ実測を窓がけ → 予測を実測時刻へ線形補間）を、
プライベートAPIを使わずに再現している。

使い方:
    uv run python validate.py                # ステージ1〜3を3分割で交差検証
    uv run python validate.py --stages 2 3
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from mujoco import sysid

sys.path.insert(0, str(Path(__file__).resolve().parent))

from identify import (  # noqa: E402
    DEFAULT_RUNS,
    DEFAULT_SHIFT,
    JOINT,
    MODEL_PATH,
    PARAM_SPECS,
    RESULTS_DIR,
    STAGES,
    collect_sequences,
    run_identification,
)

# 同定前のベースライン（models/ak45_36_joint.xml の値）。改善幅を物理単位で見るための基準。
BASELINE = {"armature": 0.01, "frictionloss": 0.0, "damping": 0.0}


def build_model(values):
    """関節パラメータを差し替えたモデルをコンパイルする。

    identify._make_modifier と同じく、damping だけは3要素ベクトルなので
    setattr ではなく第0成分への代入で書き込む。
    """
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    joint = spec.joint(JOINT)
    for name, value in values.items():
        if name == "damping":
            joint.damping[0] = value
        else:
            setattr(joint, name, value)
    return spec.compile()


def rollout_errors(model, states, controls, sensors, common_grid=False):
    """全区間をシミュレートし、実測との差（物理単位）を1つの配列にまとめて返す。

    Args:
        common_grid: 全区間を「同じ長さの、タイムステップちょうどの格子」へ揃えるか。
            既定の False では `TimeSeries.resample(target_dt=...)` に任せるが、これは
            区間の実時間長 span を保ったまま ceil(span/dt)+1 点へ等分するので、
            区間ごとに span が違うとステップ数も1つずれる。CSVのサンプル周期が
            モデルのタイムステップと同じ（1kHzのexp_005）ならこのずれは出ないが、
            100Hzで記録した exp_009 では区間長が 490/491 とばらつき、
            sysid_rollout が要求する「全区間で同形状の制御配列」を満たせなくなる。
            True にすると最短区間に合わせた `arange(n)*dt` 上へ全区間を再標本化する。

    Returns:
        (n_samples, 2) の配列。列は [位置誤差 rad, 速度誤差 rad/s]。
    """
    # 制御信号はモデルのタイムステップへ揃える（model_residual と同じ前処理）。
    # 実機の wall_time は 1ms ちょうどではないため、これをしないと区間ごとに
    # ステップ数が変わってしまう。
    dt = model.opt.timestep
    if common_grid:
        n = min(int(np.floor((np.asarray(c.times)[-1] - np.asarray(c.times)[0]) / dt)) + 1 for c in controls)
        controls = [c.resample(new_times=np.asarray(c.times)[0] + np.arange(n) * dt) for c in controls]
    else:
        controls = [c.resample(target_dt=dt) for c in controls]
    data = mujoco.MjData(model)
    trajs = sysid.sysid_rollout(model, data, controls, np.array(states))

    errors = []
    for traj, measured in zip(trajs, sensors):
        pred_t = np.asarray(traj.sensordata.times)
        pred_d = np.asarray(traj.sensordata.data)
        meas_t = np.asarray(measured.times)
        meas_d = np.asarray(measured.data)
        # 予測が張る時間範囲の外にある実測サンプルは外挿になるので落とす
        keep = (meas_t >= pred_t[0]) & (meas_t <= pred_t[-1])
        pred_on_meas = np.column_stack([np.interp(meas_t[keep], pred_t, pred_d[:, k]) for k in range(pred_d.shape[1])])
        errors.append(pred_on_meas - meas_d[keep])
    return np.vstack(errors)


def evaluate(values, run, seg_len, torque_column, shift):
    """1ランに対する予測誤差を計算する。"""
    model = build_model(values)
    _, states, controls, sensors = collect_sequences(model, (run,), seg_len, torque_column, shift, 0)
    err = rollout_errors(model, states, controls, sensors)
    return dict(
        rms_pos=float(np.sqrt(np.mean(err[:, 0] ** 2))),
        rms_vel=float(np.sqrt(np.mean(err[:, 1] ** 2))),
        max_pos=float(np.max(np.abs(err[:, 0]))),
        n_samples=int(err.shape[0]),
    )


def cross_validate(stages, runs, seg_len, torque_column, shift):
    """leave-one-run-out 交差検証を実行し、結果のリストを返す。"""
    rows = []

    # 参考: 同定前のモデルが検証ランでどれだけ外すか
    for held_out in runs:
        m = evaluate(BASELINE, held_out, seg_len, torque_column, shift)
        rows.append(dict(stage=0, held_out=held_out, params=dict(BASELINE), train_cost=None, **m))

    for stage in stages:
        for held_out in runs:
            train_runs = tuple(r for r in runs if r != held_out)
            print(f"\n--- ステージ{stage} / 検証ラン {held_out.replace('exp005_sysid_excitation_', '')} ---")
            fit = run_identification(
                stage=stage,
                torque_column=torque_column,
                seg_len=seg_len,
                shift=shift,
                runs=train_runs,
                make_report=False,
                verbose=False,
            )
            # そのステージで同定していないパラメータはXMLの値（=0）のままにする
            values = dict(BASELINE)
            values.update(fit["params"])
            m = evaluate(values, held_out, seg_len, torque_column, shift)
            print(f"  検証ラン誤差: 位置RMS {m['rms_pos'] * 1000:.2f} mrad / 速度RMS {m['rms_vel']:.4f} rad/s")
            rows.append(
                dict(
                    stage=stage,
                    held_out=held_out,
                    params=fit["params"],
                    train_cost=fit["cost_after"],
                    **m,
                )
            )
    return rows


def summarize(rows, stages):
    """ステージごとに3分割の平均を取って表にする。"""
    lines = []
    lines.append("")
    lines.append("=" * 78)
    lines.append("leave-one-run-out 交差検証の結果（検証ラン＝同定に使っていない1本）")
    lines.append("=" * 78)
    header = f"{'ステージ':<10}{'学習コスト':>12}{'位置RMS[mrad]':>16}{'速度RMS[rad/s]':>16}{'位置最大[mrad]':>16}"
    lines.append(header)
    lines.append("-" * 78)
    for stage in [0, *stages]:
        sel = [r for r in rows if r["stage"] == stage]
        if not sel:
            continue
        label = "同定前" if stage == 0 else f"ステージ{stage}"
        train = [r["train_cost"] for r in sel if r["train_cost"] is not None]
        train_s = f"{np.mean(train):.3f}" if train else "—"
        lines.append(
            f"{label:<10}{train_s:>12}"
            f"{np.mean([r['rms_pos'] for r in sel]) * 1000:>16.2f}"
            f"{np.mean([r['rms_vel'] for r in sel]):>16.4f}"
            f"{np.mean([r['max_pos'] for r in sel]) * 1000:>16.1f}"
        )
    lines.append("-" * 78)
    lines.append("")
    lines.append("同定されたパラメータの分割間ばらつき（項目18: 複数解の曖昧さの確認）:")
    for stage in stages:
        sel = [r for r in rows if r["stage"] == stage]
        for name in STAGES[stage]:
            vals = np.array([r["params"][name] for r in sel])
            lo, hi = PARAM_SPECS[name]["min_value"], PARAM_SPECS[name]["max_value"]
            spread = (vals.max() - vals.min()) / max(abs(vals.mean()), 1e-12) * 100
            at_bound = ""
            if (vals.min() - lo) < 0.01 * (hi - lo) or (hi - vals.max()) < 0.01 * (hi - lo):
                at_bound = "  ← 境界に張り付き"
            lines.append(f"  ステージ{stage} {name:13s} = {vals.mean():.6f} ± {vals.std():.6f}（幅 {spread:.1f}%）{at_bound}")
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description="同定パラメータの leave-one-run-out 交差検証")
    p.add_argument("--stages", type=int, nargs="+", choices=sorted(STAGES), default=sorted(STAGES))
    p.add_argument("--torque", dest="torque_column", choices=["desired_torque", "output_torque"], default="desired_torque")
    p.add_argument("--seg-len", type=float, default=0.5)
    p.add_argument("--shift", type=int, default=DEFAULT_SHIFT)
    p.add_argument("--runs", nargs="+", default=list(DEFAULT_RUNS))
    args = p.parse_args()

    rows = cross_validate(args.stages, tuple(args.runs), args.seg_len, args.torque_column, args.shift)
    report = summarize(rows, args.stages)
    print(report)

    out_dir = RESULTS_DIR / f"validation_{args.torque_column}_seg{args.seg_len:g}s_shift{args.shift}"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.txt").write_text(report + "\n", encoding="utf-8")
    (out_dir / "folds.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n出力: {out_dir}")


if __name__ == "__main__":
    main()
