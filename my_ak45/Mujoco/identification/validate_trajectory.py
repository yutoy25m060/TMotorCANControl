"""同定したパラメータを、同定とは**別種の軌道**（exp_009）で検証する。

作業手順書 `docs_syid/AK45-36_sysid_作業手順.md` フェーズ4 の項目20a に対応する。

## `validate.py` との違い

`validate.py`（項目17・18）の leave-one-run-out 交差検証は、同定に使ったのと
**同じ励振**（multi-sine の開ループトルク指令）の別ランで評価している。これは
「同じ実験を繰り返したときの再現性」は測れるが、「モデルが別の運動でも通用するか」
は測れない。励振信号に含まれる周波数・速度域に特化した解になっていても気づけない。

こちらは同定データとは次の点が全て違うデータで評価する:

| | 同定（exp_005） | 検証（exp_009） |
|---|---|---|
| 制御 | 開ループ（純トルク指令） | 閉ループ（インピーダンス制御による位置追従） |
| 軌道 | multi-sine 励振 | 三角波追従（振幅・周期・K・Bをランダム化した5試行） |
| サンプリング | 1kHz | 100Hz |
| 入力に使うトルク | `desired_torque`（指令値） | `output_torque`（実測値のみ） |

「制御則が違う」こと自体はMuJoCo側には見えない（どちらもトルクを開ループで
再生するだけ）が、結果として励起される軌道の性質（速度域・加減速の向き・
停留時間）が変わるため、汎化性能の検証になる。

## 読み方の注意

1. **入力トルクの列が同定側と揃わない。** exp_009 は閉ループなので明示的な指令
   トルク値が存在せず、`output_torque`（実測）を使うほかない。実測トルクは
   `Kt_actual` の既知の誤り（公式0.11 Nm/A に対し約+10%）を直接受けるため、
   同定を `desired_torque` で行った場合、全ステージに共通のトルクスケール誤差が
   上乗せされる。`--fit-torque output_torque` で同定側も実測トルクに揃えた場合と
   両方を見て、ステージ間の順位が変わらないことを確認するのが安全。
2. **絶対値は `validate.py` の数字と直接比較できない。** 区間の切り出し方・
   サンプリング周波数・軌道が違うため、ここで見るべきは「同定前からどれだけ
   改善したか」と「ステージ間の順位」であって、mrad の絶対値ではない。

使い方:
    uv run python validate_trajectory.py                          # ステージ1〜3
    uv run python validate_trajectory.py --stages 2 3
    uv run python validate_trajectory.py --fit-torque output_torque
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from csv_adapter import build_sequences  # noqa: E402
from identify import (  # noqa: E402
    DATA_DIR,
    DEFAULT_RUNS,
    DEFAULT_SHIFT,
    MODEL_PATH,
    RESULTS_DIR,
    STAGES,
    run_identification,
)
from validate import BASELINE, build_model, rollout_errors  # noqa: E402

# 検証に使う別軌道データ（exp_009、5試行すべて completed）。
DEFAULT_TRAJ_RUN = "exp009_validation_trajectory_randomized_1787187421"

# exp_009 は100Hz（`make_realtime_loop()` の既定 dt=0.01）で記録されている。
# shift はCSVの「行数」なので、1kHzのexp_005で求めた DEFAULT_SHIFT=2 をそのまま
# 使うと20ms分ずらすことになり過補正になる。100Hzでの内訳は
#   記録の帳簿上のずれ 1行 = 10ms
#   ＋ 電流ループのむだ時間 約1.9ms
#   − sysid_rollout の時刻付けによる暗黙の補正 1モデルステップ = 1ms
#   ≒ 10.9ms ≒ 1.09行
# であり、1行が妥当。実際に 0/1/2 を振ると1が最小になることを確認済み。
TRAJ_SHIFT = 1

# 先頭から捨てる助走時間 [秒]。exp_009 の各試行は、ゼロ化した位置（0 rad）から
# 三角波の始点（-amplitude）へ飛びつくところから始まる。この飛びつきは指令が
# ステップ状に入る過渡で、追従誤差が振幅そのものに達し（manifest の
# max_tracking_error はほぼこの瞬間の値）、速度もその試行の定常運動より一桁速い。
# csv_adapter.startup_trim_time() の速度しきい値（3.815 rad/s）では試行によって
# 引っかかったり引っかからなかったりして扱いが揃わないため、明示的に切る。
# なお誤差への影響自体は小さい（skip=0/0.5/1.0 で平均62.2/63.3/63.4 mrad）。
# 切るのは「全試行を同じ扱いにする」ためであって、数字を良く見せるためではない。
TRAJ_SKIP = 0.5


def load_manifest(run_dir):
    """exp_009 の manifest.csv を読み、completed の試行だけを返す。

    K/B の値はCSV本体に入らないため、試行の条件を報告に載せるにはこれが要る。
    """
    path = run_dir / "manifest.csv"
    if not path.exists():
        raise FileNotFoundError(f"manifest が見つかりません: {path}")
    with open(path, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r["status"] == "completed"]
    if not rows:
        raise RuntimeError(f"{path} に completed の試行がありません")
    return [
        dict(
            trial=int(r["trial"]),
            amplitude=float(r["amplitude"]),
            period=float(r["period"]),
            K=float(r["K"]),
            B=float(r["B"]),
        )
        for r in rows
    ]


def collect_trial(model, run_dir, trial, seg_len, shift, skip):
    """1試行分のCSVを sysid 用のシーケンスへ変換する。

    exp_005 は `{run}/log.csv` だが exp_009 は `{run}/trial_NN/log.csv` という
    階層なので、identify.collect_sequences はそのまま使えない。
    """
    csv_path = run_dir / f"trial_{trial:02d}" / "log.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"検証データが見つかりません: {csv_path}")
    return build_sequences(
        csv_path,
        model,
        seg_len=seg_len,
        # 閉ループなので desired_torque 列は存在しない（選択肢がない）
        torque_column="output_torque",
        shift=shift,
        run_label=f"trial{trial:02d}",
        skip_time=skip,
    )


def evaluate_trial(values, run_dir, trial, seg_len, shift, skip):
    """1試行に対する予測誤差（物理単位）を返す。"""
    model = build_model(values)
    names, states, controls, sensors = collect_trial(model, run_dir, trial, seg_len, shift, skip)
    if not names:
        raise RuntimeError(f"試行{trial}で区間が1つも作れませんでした（seg_len が長すぎる可能性があります）")
    # common_grid=True: 100Hz記録なので区間ごとの実時間長がわずかに違い、既定の
    # resample(target_dt=...) だと区間でステップ数がずれて sysid_rollout に渡せない。
    err = rollout_errors(model, states, controls, sensors, common_grid=True)
    # 誤差の大きさを軌道の大きさで割った相対値。試行ごとに振幅が違うため、
    # mrad の絶対値だけでは試行間の良し悪しを比べられない。
    span = float(np.ptp(np.concatenate([np.asarray(s.data)[:, 0] for s in sensors])))
    return dict(
        n_segments=len(names),
        n_samples=int(err.shape[0]),
        rms_pos=float(np.sqrt(np.mean(err[:, 0] ** 2))),
        rms_vel=float(np.sqrt(np.mean(err[:, 1] ** 2))),
        max_pos=float(np.max(np.abs(err[:, 0]))),
        pos_span=span,
        rel_pos=float(np.sqrt(np.mean(err[:, 0] ** 2)) / span),
    )


def fit_stage(stage, fit_runs, fit_torque, fit_seg_len, fit_shift):
    """同定側（exp_005 全ラン）を1ステージ分走らせ、パラメータ値の辞書を返す。

    そのステージで同定対象にしていないパラメータはXMLの値（BASELINE）のまま残す。
    """
    fit = run_identification(
        stage=stage,
        torque_column=fit_torque,
        seg_len=fit_seg_len,
        shift=fit_shift,
        runs=fit_runs,
        make_report=False,
        verbose=False,
    )
    values = dict(BASELINE)
    values.update(fit["params"])
    return values, fit


def summarize(rows, trials, stages, args):
    """ステージ別の平均表と、試行ごとの内訳を文字列にまとめる。"""
    lines = []
    lines.append("=" * 92)
    lines.append("別軌道での検証（項目20a）: 同定=exp_005 multi-sine開ループ / 検証=exp_009 インピーダンス追従")
    lines.append("=" * 92)
    lines.append(f"同定ラン: {len(args.fit_runs)}本すべて使用 / 入力トルク={args.fit_torque} / 区間{args.fit_seg_len:g}s / shift={args.fit_shift}")
    lines.append(f"検証ラン: {args.run}")
    lines.append(f"  {len(trials)}試行 / 入力トルク=output_torque（閉ループのため選択肢なし） / 区間{args.seg_len:g}s / shift={args.shift} / 助走{args.skip:g}s切り捨て")
    lines.append("")
    lines.append("[ステージ別の平均（5試行の平均）]")
    header = (
        f"{'ステージ':<10}{'armature':>10}{'friction':>10}{'damping':>10}"
        f"{'位置RMS[mrad]':>16}{'速度RMS[rad/s]':>16}{'位置最大[mrad]':>16}{'位置RMS/振れ幅':>16}"
    )
    lines.append(header)
    lines.append("-" * 92)
    for stage in [0, *stages]:
        sel = [r for r in rows if r["stage"] == stage]
        if not sel:
            continue
        p = sel[0]["params"]
        label = "同定前" if stage == 0 else f"ステージ{stage}"
        lines.append(
            f"{label:<10}{p['armature']:>10.5f}{p['frictionloss']:>10.5f}{p['damping']:>10.5f}"
            f"{np.mean([r['rms_pos'] for r in sel]) * 1000:>16.2f}"
            f"{np.mean([r['rms_vel'] for r in sel]):>16.4f}"
            f"{np.mean([r['max_pos'] for r in sel]) * 1000:>16.1f}"
            f"{np.mean([r['rel_pos'] for r in sel]) * 100:>15.2f}%"
        )
    lines.append("-" * 92)

    base = [r for r in rows if r["stage"] == 0]
    base_rms = np.mean([r["rms_pos"] for r in base])
    lines.append("")
    lines.append("[同定前からの改善倍率（位置RMS）]")
    for stage in stages:
        sel = [r for r in rows if r["stage"] == stage]
        lines.append(f"  ステージ{stage}: {base_rms / max(np.mean([r['rms_pos'] for r in sel]), 1e-12):.2f}倍")

    lines.append("")
    lines.append("[試行ごとの内訳（位置RMS [mrad]）]")
    cond = {t["trial"]: t for t in trials}
    head = f"{'試行':<6}{'振幅[rad]':>11}{'周期[s]':>9}{'K':>7}{'B':>7}{'区間数':>8}"
    head += "".join(f"{('同定前' if s == 0 else f'St{s}'):>9}" for s in [0, *stages])
    lines.append(head)
    lines.append("-" * 92)
    for t in trials:
        c = cond[t["trial"]]
        line = f"{t['trial']:<6}{c['amplitude']:>11.3f}{c['period']:>9.2f}{c['K']:>7.2f}{c['B']:>7.3f}"
        n_seg = next(r["n_segments"] for r in rows if r["trial"] == t["trial"])
        line += f"{n_seg:>8}"
        for stage in [0, *stages]:
            r = next(r for r in rows if r["stage"] == stage and r["trial"] == t["trial"])
            line += f"{r['rms_pos'] * 1000:>9.1f}"
        lines.append(line)
    lines.append("-" * 92)
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description="同定パラメータを別軌道（exp_009）で検証する")
    p.add_argument("--stages", type=int, nargs="+", choices=sorted(STAGES), default=sorted(STAGES))
    p.add_argument("--run", default=DEFAULT_TRAJ_RUN, help="検証に使う exp_009 実行フォルダ名")
    p.add_argument("--seg-len", type=float, default=0.5, help="検証側の区間長 [秒]")
    p.add_argument("--shift", type=int, default=TRAJ_SHIFT, help=f"検証側の実測列前詰め行数（既定: {TRAJ_SHIFT}、100Hzデータ用）")
    p.add_argument("--skip", type=float, default=TRAJ_SKIP, help=f"各試行の先頭から捨てる助走時間 [秒]（既定: {TRAJ_SKIP}）")
    p.add_argument("--fit-runs", nargs="+", default=list(DEFAULT_RUNS), help="同定に使う exp_005 ラン")
    p.add_argument("--fit-torque", choices=["desired_torque", "output_torque"], default="desired_torque")
    p.add_argument("--fit-seg-len", type=float, default=0.5, help="同定側の区間長 [秒]")
    p.add_argument("--fit-shift", type=int, default=DEFAULT_SHIFT, help=f"同定側の前詰め行数（既定: {DEFAULT_SHIFT}、1kHzデータ用）")
    args = p.parse_args()

    run_dir = DATA_DIR / args.run
    trials = load_manifest(run_dir)
    print(f"検証データ: {run_dir.name} / completed {len(trials)} 試行")

    rows = []

    # 参考: 同定前のモデル（models/ak45_36_joint.xml の値）が別軌道でどれだけ外すか
    print("\n--- 同定前（ベースライン）---")
    for t in trials:
        m = evaluate_trial(BASELINE, run_dir, t["trial"], args.seg_len, args.shift, args.skip)
        print(f"  試行{t['trial']}: 位置RMS {m['rms_pos'] * 1000:.1f} mrad（{m['n_segments']}区間）")
        rows.append(dict(stage=0, trial=t["trial"], params=dict(BASELINE), train_cost=None, **m))

    for stage in args.stages:
        print(f"\n--- ステージ{stage}: exp_005 {len(args.fit_runs)}ランで同定 ---")
        values, fit = fit_stage(stage, tuple(args.fit_runs), args.fit_torque, args.fit_seg_len, args.fit_shift)
        for t in trials:
            m = evaluate_trial(values, run_dir, t["trial"], args.seg_len, args.shift, args.skip)
            print(f"  試行{t['trial']}: 位置RMS {m['rms_pos'] * 1000:.1f} mrad / 速度RMS {m['rms_vel']:.4f} rad/s")
            rows.append(dict(stage=stage, trial=t["trial"], params=dict(values), train_cost=fit["cost_after"], **m))

    report = summarize(rows, trials, args.stages, args)
    print()
    print(report)

    out_dir = RESULTS_DIR / f"validation_trajectory_fit-{args.fit_torque}_seg{args.seg_len:g}s_shift{args.shift}"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.txt").write_text(report + "\n", encoding="utf-8")
    (out_dir / "trials.json").write_text(
        json.dumps(dict(args=vars(args), conditions=trials, rows=rows), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"\n出力: {out_dir}")


if __name__ == "__main__":
    # MODEL_PATH の存在確認だけ先に済ませる（同定に数分かかるため）
    if not Path(MODEL_PATH).exists():
        raise SystemExit(f"モデルが見つかりません: {MODEL_PATH}")
    mujoco.MjSpec.from_file(str(MODEL_PATH)).compile()
    main()
