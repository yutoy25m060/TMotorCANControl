"""AK45-36 の MuJoCo モデルパラメータを実機データから同定する。

作業手順書 `docs_syid/AK45-36_sysid_作業手順.md` フェーズ3 の項目11・12・13・15・16 に対応する。

同定対象は段階的に増やす（項目13/16）。複数パラメータを最初から同時に動かすと識別不能に
なりやすいため、1つずつ足して各段階の残差とパラメータ値を比べられるようにしている:

    ステージ1: armature のみ           （反射ロータ慣性）
    ステージ2: + frictionloss          （クーロン摩擦）
    ステージ3: + damping               （粘性摩擦）

そのステージで対象にしていないパラメータは models/ak45_36_joint.xml の値がそのまま固定値
として効く（sysid.Parameter の frozen=True は使わない。apply_param_modifiers が frozen の
modifier をスキップするため、値を固定する用途には使えない）。

使い方:
    uv run python identify.py                          # ステージ1、指令トルク、区間0.5s
    uv run python identify.py --stage 3
    uv run python identify.py --torque output_torque    # 実測トルクを入力にした比較
    uv run python identify.py --stage 2 --shift 1 --no-report
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from mujoco import sysid

sys.path.insert(0, str(Path(__file__).resolve().parent))

from csv_adapter import build_sequences  # noqa: E402

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parent / "models" / "ak45_36_joint.xml"
DATA_DIR = HERE.parent / "data" / "raw"
RESULTS_DIR = HERE / "results"

JOINT = "ak45_joint"

# 同定に使う実機ラン。コミット 6e7214b「syid正式採用データ」で採用された3本。
# いずれも wall_time 列あり・10251行・sysid_run_check.py で条件付き合格（WARNのみ）。
# 同一励振を独立に複数回実行したものを全て投入するのは、初期状態の誤差が各ランで独立なので
# 系統的な偏りになりにくいため（作業手順書 項目11）。
DEFAULT_RUNS = (
    "exp005_sysid_excitation_1786575616",
    "exp005_sysid_excitation_1786575633",
    "exp005_sysid_excitation_1786575782",
)

# 実測列を何行前に詰めるか。
#
# リポジトリ内には「1行」「3行」「2行」と3つの数字が登場するが、これらは**矛盾ではなく
# 別々の量**である。CSV上の指令と実測のずれは次の2つの成分に分解できる:
#
#   (a) 記録の帳簿上のずれ = 1サンプル（構造的、コードから確定）
#       TMotorManager_mit_can.update() は「状態を読む → _send_command()」の順で処理する
#       （mit_can.py の update() 末尾）。したがってCSVの行kに入る実測値は、行kの指令を
#       送る *前* の状態であり、desired[k-1] までへの応答である。
#       → exp_005_sysid_excitation.py の「実測列を1行前に詰める」はこの成分を指す。
#
#   (b) モーター内蔵電流ループの物理的なむだ時間 = 約1.9サンプル
#       sysid_run_check.py の周波数応答が採用3ランで L=1.82〜1.87ms と報告する値。
#       1kHzサンプリングなので約2サンプルに相当する。
#
#   合計 (a)+(b) ≈ 2.9 サンプル
#       → sysid_run_check.py が相互相関で測る「3行」はこの合計。1ms刻みの粗い測定であり、
#         一次遅れ T≈1.25ms の寄与も混じるため、むだ時間そのものよりやや大きく出る。
#
# MuJoCo の rollout は sensor[i] = ctrl[0..i-1] への応答 という、実機のロギング規約
# (a) と **まったく同じ**規約を持つ。つまり (a) は rollout 側が自動的に合わせてくれるので、
# ここで補正すべきなのは (b) の物理的なむだ時間だけ → 約2サンプル。
#
# 実際に stage2・desired_torque・区間0.5s で shift を振ると最終コストは
#   shift=0: 0.665 / shift=1: 0.408 / shift=2: 0.334 / shift=3: 0.344
# となり2で最小。独立に測定された L=1.82〜1.87ms（≈2サンプル）と直接一致しており、
# この値は引き算の産物ではなく物理的なむだ時間そのものと解釈できる。
#
# なお armature はこの選択にほぼ影響されない（0.0128〜0.0123、約4%）のに対し、
# frictionloss は 0.119〜0.179 と50%動くため、摩擦を語るときはこの値の妥当性に注意すること。
DEFAULT_SHIFT = 2

# パラメータの初期値と探索範囲。
#
# 初期値の根拠: 実機3ラン横断で τ = J·a + c·sign(v) + b·v の最小二乗を取ると
# J≈0.0030 / c≈0.10 / b≈0 が安定して得られる。ただしこの推定は位置の2階差分という
# ノイズの大きい回帰変数を使うため attenuation bias で過小側に出ており、実際に
# rollout ベースで最適化すると armature は 0.013 前後に収束する（反射慣性に換算すると
# ロータ慣性 ≈1.0e-5 kg·m² となり物理的に妥当）。したがって範囲は広めに取っている。
#
# frictionloss の初期値を 0 にしないのは、探索が境界に張り付いた状態から始まると
# 悪条件の問題でその隅に留まりやすいため（sysid.optimize が警告を出す）。
PARAM_SPECS = {
    "armature": dict(nominal=0.01, min_value=1e-4, max_value=0.05),
    "frictionloss": dict(nominal=0.05, min_value=0.0, max_value=0.5),
    "damping": dict(nominal=0.01, min_value=0.0, max_value=0.2),
}
STAGES = {1: ("armature",), 2: ("armature", "frictionloss"), 3: ("armature", "frictionloss", "damping")}


def _make_modifier(param_name):
    """spec 上の関節属性へパラメータ値を書き込む modifier を作る。

    MjsJoint の属性の型が揃っていない点に注意: armature/frictionloss はスカラーだが、
    damping は（ボール/フリー関節と共用のため）3要素ベクトルであり、setattr で
    スカラーを代入すると TypeError になる。ヒンジ関節では第0成分だけが使われる。
    """
    if param_name == "damping":

        def modifier(spec, param):
            spec.joint(JOINT).damping[0] = param.value[0]
    else:

        def modifier(spec, param):
            setattr(spec.joint(JOINT), param_name, param.value[0])

    return modifier


def build_params(stage):
    """指定ステージの ParameterDict を作る。"""
    params = sysid.ParameterDict()
    for name in STAGES[stage]:
        spec = PARAM_SPECS[name]
        params.add(sysid.Parameter(name, modifier=_make_modifier(name), **spec))
    return params


def collect_sequences(model, runs, seg_len, torque_column, shift, crossing_offset):
    """全ランの区間を1つにまとめて返す。"""
    names, states, controls, sensors = [], [], [], []
    for run in runs:
        csv_path = DATA_DIR / run / "log.csv"
        if not csv_path.exists():
            raise FileNotFoundError(f"実機データが見つかりません: {csv_path}")
        n, s, c, se = build_sequences(
            csv_path,
            model,
            seg_len=seg_len,
            torque_column=torque_column,
            shift=shift,
            crossing_offset=crossing_offset,
            run_label=run.replace("exp005_sysid_excitation_", "run"),
        )
        names += n
        states += s
        controls += c
        sensors += se
    return names, states, controls, sensors


def total_cost(residual_fn, params):
    """残差の二乗和（sysid.optimize が最小化している目的関数の2倍）。"""
    residuals, _, _ = residual_fn(params.as_vector(), params)
    return float(sum(np.sum(np.asarray(r) ** 2) for r in residuals))


def run_identification(
    stage=1,
    torque_column="desired_torque",
    seg_len=0.5,
    shift=DEFAULT_SHIFT,
    crossing_offset=0,
    runs=DEFAULT_RUNS,
    make_report=True,
    verbose=True,
):
    """同定を1回実行し、結果を辞書で返す。"""
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    model = spec.compile()

    names, states, controls, sensors = collect_sequences(model, runs, seg_len, torque_column, shift, crossing_offset)
    if not names:
        raise RuntimeError("区間が1つも作れませんでした（seg_len が長すぎる可能性があります）")

    print(f"ステージ{stage}: {', '.join(STAGES[stage])}")
    print(f"  入力トルク: {torque_column} / 区間長: {seg_len}s / shift: {shift} / 交差オフセット: {crossing_offset}")
    print(f"  ラン {len(runs)} 本から計 {len(names)} 区間")

    ms = sysid.ModelSequences("ak45_36", spec, names, states, controls, sensors)
    params = build_params(stage)
    # 境界に張り付いた状態から始めると悪条件時にその隅で停滞しうるため、わずかに内側へ寄せる
    params = params.move_off_bounds()

    residual_fn = sysid.build_residual_fn(models_sequences=[ms])
    cost_before = total_cost(residual_fn, params)

    opt_params, opt_result = sysid.optimize(
        initial_params=params,
        residual_fn=residual_fn,
        optimizer="mujoco",
        verbose=verbose,
    )
    cost_after = total_cost(residual_fn, opt_params)

    print(f"  コスト: {cost_before:.4f} -> {cost_after:.4f}（{cost_before / max(cost_after, 1e-12):.1f}倍改善）")
    for name in STAGES[stage]:
        value = float(opt_params[name].value[0])
        lo, hi = PARAM_SPECS[name]["min_value"], PARAM_SPECS[name]["max_value"]
        # 境界に張り付いた解は「その方向にはもっと良い値がある」ことを意味し信用できない
        at_bound = " ← 境界に張り付き（要確認）" if (value - lo) < 0.01 * (hi - lo) or (hi - value) < 0.01 * (hi - lo) else ""
        print(f"  {name:13s} = {value:.6f}{at_bound}")

    result = dict(
        stage=stage,
        torque_column=torque_column,
        seg_len=seg_len,
        shift=shift,
        crossing_offset=crossing_offset,
        n_segments=len(names),
        cost_before=cost_before,
        cost_after=cost_after,
        params={n: float(opt_params[n].value[0]) for n in STAGES[stage]},
    )

    if make_report:
        out_dir = RESULTS_DIR / f"stage{stage}_{torque_column}_seg{seg_len:g}s_shift{shift}_off{crossing_offset}"
        out_dir.mkdir(parents=True, exist_ok=True)
        # default_report は save_path に対して "/" 演算子を使うため Path であること
        sysid.default_report(
            models_sequences=[ms],
            initial_params=params,
            opt_params=opt_params,
            residual_fn=residual_fn,
            opt_result=opt_result,
            title=f"AK45-36 sysid stage{stage} ({torque_column})",
            save_path=out_dir,
            generate_videos=False,
        )
        opt_params.save_to_disk(out_dir / "params.yaml")
        (out_dir / "summary.txt").write_text(
            "\n".join(f"{k}: {v}" for k, v in result.items()) + "\n",
            encoding="utf-8",
        )
        print(f"  出力: {out_dir}")
        result["out_dir"] = str(out_dir)

    return result


def main():
    p = argparse.ArgumentParser(description="AK45-36 の MuJoCo パラメータ同定")
    p.add_argument("--stage", type=int, choices=sorted(STAGES), default=1, help="同定対象の段階（既定: 1 = armature のみ）")
    p.add_argument("--torque", dest="torque_column", choices=["desired_torque", "output_torque"], default="desired_torque")
    p.add_argument("--seg-len", type=float, default=0.5, help="区間長 [秒]（既定: 0.5）")
    p.add_argument("--shift", type=int, default=DEFAULT_SHIFT, help=f"実測列を何行前に詰めるか（既定: {DEFAULT_SHIFT}）")
    p.add_argument("--crossing-offset", type=int, default=0, help="読み飛ばす速度ゼロ交差の個数（切り出し点の頑健性確認用）")
    p.add_argument("--runs", nargs="+", default=list(DEFAULT_RUNS), help="使用する実機ランのディレクトリ名")
    p.add_argument("--no-report", action="store_true", help="HTMLレポートを出力しない")
    # 注: 反復ごとのログは最適化バックエンド側が出力しており、sysid.optimize() の
    # verbose 引数（最終的なパラメータ比較表の出力可否）とは別系統で、外から抑制できない。
    p.add_argument("--quiet", action="store_true", help="最適化後のパラメータ比較表の出力を抑制する")
    args = p.parse_args()

    run_identification(
        stage=args.stage,
        torque_column=args.torque_column,
        seg_len=args.seg_len,
        shift=args.shift,
        crossing_offset=args.crossing_offset,
        runs=tuple(args.runs),
        make_report=not args.no_report,
        verbose=not args.quiet,
    )


if __name__ == "__main__":
    main()
