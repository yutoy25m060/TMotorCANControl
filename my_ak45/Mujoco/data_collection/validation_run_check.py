"""validation用の別軌道データ（exp_008/exp_009 の出力CSV）の自動検証。

`sysid_run_check.py` と同じ位置づけの事後チェックだが、対象と目的が違う:

| | `sysid_run_check.py` | 本スクリプト |
|---|---|---|
| 対象 | exp_005（開ループ multi-sine 励振、1kHz） | exp_008/exp_009（閉ループ三角波追従、100Hz） |
| 用途 | **同定**に使えるデータか | **検証**に使えるデータか |
| 指令 | `desired_torque`（明示的なトルク指令） | `desired_pos`（位置目標。トルク指令は存在しない） |

「同定に使えるか」と「検証に使えるか」は要求が違う。同定では励振の線形性・周波数内容が
決定的だが、検証では **PC側の `identification/validate_trajectory.py` がこのCSVを
そのまま食えるか**、そして **同定データと十分に違う運動になっているか** が問題になる。
そのため `sysid_run_check.py` の11項目のうち、指令トルクを前提とする4項目
（トルク線形性・符号反転・指令-実測の遅れ・周波数応答）は原理的に移植できず、
代わりに検証データ特有の項目（追従品質・飛びつき過渡・検証区間数・双方向カバレッジ）を持つ。

判定はCSV1本ごとに行い、exp_009 のような複数試行の実行フォルダに対しては全試行を
回したうえで、最後に「試行間の条件の広がり」を run 全体の項目として追加判定する。

使い方:
    # 実験スクリプトから（exp_008/exp_009 が実行後に自動で呼ぶ）
    from validation_run_check import check_validation_run
    check_validation_run(RUN_DIR, max_temp=..., max_torque=..., max_velocity=...)

    # 過去データに対して単体で
    python validation_run_check.py ../data/raw/exp009_validation_trajectory_randomized_1787187421
    python validation_run_check.py ../data/raw/exp008_validation_trajectory_XXXXXXXXXX/log.csv
"""

import csv
import sys
from pathlib import Path

import numpy as np

# 記録の健全性（サンプル数・wall_time ジッタ）の判定は制御則にも励振波形にも依存しないため、
# sysid_run_check.py の実装をそのまま共有する。しきい値と文言を一箇所に保つのが目的。
# 「1kHz用の値を100Hzデータに流用している」わけではない（判定はいずれも公称dtからの
# 相対量で行われるため、サンプリング周波数に依存しない）。
from sysid_run_check import (
    FD_SLOPE_RANGE,
    NO_LOAD_SPEED,
    RATED_CURRENT,
    VEL_PEAK_FAIL,
    VEL_PEAK_WARN,
    RunCheckReport,
    report_sampling,
)

# --- 判定しきい値（本スクリプト固有） -------------------------------------
#
# 数値の出どころは exp_009 の実機5試行
# （data/raw/exp009_validation_trajectory_randomized_1787187421）。
# 「実際に取れた良好なデータの実測範囲」を測ってから、その外側に余裕をもって置いている。
# 良品を落とさないことを優先し、FAIL は「そのデータでは検証が成立しない」水準に限る。

# 飛びつき過渡: 各試行は現在位置（ゼロ化直後なので0 rad）から三角波の始点（-amplitude）へ
# ステップ状に飛びつくところから始まる。実測の整定時間は0.01〜0.16秒で、PC側の
# `validate_trajectory.py` は先頭0.5秒を無条件に捨てている。整定がそれを超えると
# 捨て幅が足りず、過渡が検証区間に混入する。
SETTLE_SKIP_S = 0.5  # validate_trajectory.TRAJ_SKIP と揃えること
SETTLE_WARN_S = 0.35  # 0.5秒の捨て幅に対して余裕が乏しくなる水準
SETTLE_FAIL_S = SETTLE_SKIP_S

# 追従品質: 実測は振幅比 4.3〜13.4%。これを大きく超えるのは、指令にモーターが
# 付いていけていない＝「三角波を辿った」と言えない状態。
TRACK_RMS_WARN_FRAC = 25.0  # [%] 振幅に対する追従誤差RMS
TRACK_RMS_FAIL_FRAC = 50.0

# 実際に動いた範囲 / 指令された範囲。小さいと、設定した振幅より狭い運動しか
# 記録できていない（実測は 0.84〜1.07 倍）。
RANGE_RATIO_WARN = 0.80

# PC側が切り出せる区間数。`validate_trajectory.py` は速度ゼロ交差を区間の先頭に選び、
# 区間長 `seg_len` 以上の間隔を空けて拾う。区間が少ないと、その試行のRMSが
# たまたま拾った数区間に支配されて統計にならない（実測は 5〜15 区間）。
SEG_LEN_DEFAULT = 0.5  # validate_trajectory.py の --seg-len 既定と揃えること
N_SEGMENTS_WARN = 5
N_SEGMENTS_FAIL = 2

# 双方向カバレッジ: 三角波は往路・復路で同じだけ動くはずなので、正転・逆転のサンプル数は
# 本来ほぼ等しい。偏っていると摩擦の方向依存性を検証できない（実測は 0.69〜0.98）。
VEL_MOVING_THRESHOLD = 0.05  # [rad/s] これ未満は「停止中」として双方向判定から除く
DIRECTION_BALANCE_WARN = 0.50  # min(正転割合, 逆転割合) / max(...)

# 飛びつき中の安全余裕。実測では飛びつきのピーク速度が 5.12 rad/s
# （safety.max_velocity 6.0 の85%）まで達した試行があり、あと少しで SafetyMonitor の
# 緊急停止に掛かるところだった。記録全体の速度・トルクのピークはほぼこの区間に出る。
JUMP_MARGIN_WARN_FRAC = 80.0  # [%] 安全上限に対する割合

# 試行間の条件の広がり（exp_009 のような複数試行の実行に対してのみ判定）。
# 別軌道検証では「速い試行と遅い試行で最良ステージが割れる」ことが分かっており
# （.ai/logs/2026-08-20_03_*）、速い側・遅い側の両方が揃っていないと平均が偏る。
N_TRIALS_WARN = 3
PERIOD_SPREAD_WARN = 1.5  # 周期の 最大/最小 比


def _settle_time(t, desired, actual, amplitude, hold_s=0.2):
    """飛びつき過渡が収まった時刻を返す [秒]。

    「追従誤差が振幅の20%未満に落ち、そのまま hold_s 秒間その状態を保つ」最初の時刻。
    閾値を下回った瞬間ではなく保持を要求するのは、飛びつきの行き過ぎ（オーバーシュート）で
    誤差が一瞬ゼロを横切るのを整定と誤判定しないため。
    """
    err = np.abs(desired - actual)
    thr = 0.2 * amplitude
    dt = float(np.median(np.diff(t)))
    n_hold = max(1, int(round(hold_s / dt)))
    below = err < thr
    for k in range(len(below) - n_hold):
        if below[k : k + n_hold].all():
            return float(t[k])
    return float(t[-1])  # 最後まで整定しなかった


def _count_segments(t, vel, seg_len):
    """`identification/csv_adapter.py` の segment_starts() が拾う区間数を数える。

    PC側の実装は `from mujoco import sysid` を含むモジュールにあり、mujoco の入らない
    Pi 環境からは import できないため、切り出し規則をここに移植している
    （`csv_adapter.VEL_PEAK_WARN` が `sysid_run_check.py` から移植されているのと同じ事情）。
    **規則を変える場合は両方を揃えること。**
    """
    sign = np.sign(vel)
    crossings = np.flatnonzero(sign[:-1] * sign[1:] < 0)
    dt = float(np.median(np.diff(t)))
    n_step = int(round(seg_len / dt))
    starts, last_t = [], -np.inf
    for i in crossings:
        if t[i] - last_t >= seg_len:
            starts.append(int(i))
            last_t = t[i]
    # 末尾に区間長分の余裕がないものは PC 側でも捨てられる
    return sum(1 for i in starts if i + n_step <= len(t))


def check_trial(
    csv_path,
    amplitude=None,
    period=None,
    expected_samples=None,
    max_temp=None,
    max_torque=None,
    max_velocity=None,
    seg_len=SEG_LEN_DEFAULT,
    label=None,
    rep=None,
):
    """別軌道CSVを1本検証し、レポートを標準出力に印字する。

    Args:
        csv_path: exp_008/exp_009 が出力した log.csv のパス
        amplitude: 指令した三角波の振幅 [rad]（None なら desired_pos 列から推定）
        period: 指令した三角波の周期 [秒]（表示のみ。None なら省略）
        expected_samples: 期待サンプル数（None なら完全性チェックを省略）
        max_temp: モーターの温度上限 [℃]（None なら温度余裕チェックを省略）
        max_torque: `safety.max_torque` [Nm]（None なら余裕チェックを省略）
        max_velocity: `safety.max_velocity` [rad/s]（None なら余裕チェックを省略）
        seg_len: PC側の検証で使う区間長 [秒]
        label: 見出しに出す名前（None なら csv_path）
        rep: 複数試行をまとめて集計する場合の RunCheckReport（None なら内部で作る）

    Returns:
        bool: この試行で新たな FAIL が出なければ True
    """
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t"]
    desired = d["desired_pos"]
    pos = d["output_angle"]
    vel = d["output_velocity"]
    cur = d["current"]
    tau = d["output_torque"]
    temp = d["mosfet_temperature"]

    own_rep = rep is None
    rep = rep if rep is not None else RunCheckReport()
    n_fail0, n_warn0 = rep.n_fail, rep.n_warn

    if amplitude is None:
        # 三角波は [-amplitude, 0] を辿る（exp_008/exp_009 の generate_triangle_trajectory）。
        # manifest が無い exp_008 でも、指令列そのものから振幅を復元できる。
        amplitude = float(desired.max() - desired.min())
    head = label or str(csv_path)
    cond = f"振幅 {amplitude:.3f} rad" + (f", 周期 {period:.2f}s" if period is not None else "")
    print(f"\n--- {head}（{cond}）---")

    # --- 1. 取得の完全性・実ジッタ（sysid_run_check.py と共通） ---
    report_sampling(rep, d, expected_samples)

    # --- 2. 飛びつき過渡 ---
    settle = _settle_time(t, desired, pos, amplitude)
    status = "FAIL" if settle >= SETTLE_FAIL_S else ("WARN" if settle >= SETTLE_WARN_S else "PASS")
    rep.item(
        "2. 飛びつき過渡",
        status,
        f"整定 {settle:.2f}s（PC側 validate_trajectory.py は先頭 {SETTLE_SKIP_S:.1f}s を捨てる）",
    )
    # 飛びつき中は指令が現在位置から振幅分だけ離れており、K*誤差 のトルクで急加速する。
    # 記録全体の速度ピークはほぼここに出るため、安全余裕はこの区間で評価する。
    jump = t < max(settle, 0.05)
    vjump = float(np.abs(vel[jump]).max()) if jump.any() else 0.0
    if max_velocity is not None:
        frac = 100 * vjump / max_velocity
        rep.item(
            "   飛びつき中の速度",
            "WARN" if frac > JUMP_MARGIN_WARN_FRAC else "PASS",
            f"ピーク {vjump:.2f} rad/s = safety.max_velocity {max_velocity} の {frac:.0f}%"
            "（超えると SafetyMonitor が緊急停止して以降の試行が中止される。K・振幅を下げること）",
        )
    else:
        rep.item("   飛びつき中の速度", "INFO", f"ピーク {vjump:.2f} rad/s")

    # 以降は過渡を除いた「本来の追従区間」で評価する（PC側の切り出しと同じ範囲）
    keep = t >= SETTLE_SKIP_S
    tk, dk, pk, vk, tauk = t[keep], desired[keep], pos[keep], vel[keep], tau[keep]

    # --- 3. 追従品質 ---
    err = dk - pk
    err_rms = float(np.sqrt(np.mean(err**2)))
    rep.judge(
        "3. 追従品質",
        100 * err_rms / amplitude,
        warn_over=TRACK_RMS_WARN_FRAC,
        fail_over=TRACK_RMS_FAIL_FRAC,
        fmt="{:.1f}",
        detail_suffix=f"% = 追従誤差RMS {err_rms:.4f} rad / 振幅 {amplitude:.3f} rad"
        f"（最大 {float(np.abs(err).max()):.3f} rad）",
    )
    actual_range = float(pk.max() - pk.min())
    range_ratio = actual_range / amplitude
    rep.item(
        "   実際の可動範囲",
        "PASS" if range_ratio >= RANGE_RATIO_WARN else "WARN",
        f"{range_ratio:.2f} 倍（実測 {actual_range:.3f} rad / 指令 {amplitude:.3f} rad）。"
        "小さいと指令したより狭い運動しか検証できていない",
    )

    # --- 4. PC側で切り出せる区間数 ---
    n_seg = _count_segments(tk, vk, seg_len)
    status = "FAIL" if n_seg <= N_SEGMENTS_FAIL else ("WARN" if n_seg < N_SEGMENTS_WARN else "PASS")
    rep.item(
        "4. 検証区間数",
        status,
        f"{n_seg} 区間（区間長 {seg_len}s、速度ゼロ交差起点）。"
        "少ないと、その試行のRMSが数区間に支配されて統計にならない",
    )

    # --- 5. 双方向カバレッジ ---
    fwd = float(np.mean(vk > VEL_MOVING_THRESHOLD))
    bwd = float(np.mean(vk < -VEL_MOVING_THRESHOLD))
    balance = min(fwd, bwd) / max(fwd, bwd) if max(fwd, bwd) > 0 else 0.0
    rep.item(
        "5. 双方向カバレッジ",
        "PASS" if balance >= DIRECTION_BALANCE_WARN else "WARN",
        f"正転 {100 * fwd:.0f}% / 逆転 {100 * bwd:.0f}%（均衡 {balance:.2f}）。"
        "偏ると摩擦の方向依存性を検証できない",
    )

    # --- 6. 速度飽和（sysid_run_check.py の項目2と同基準） ---
    vpk = float(np.abs(vk).max())
    rep.judge(
        "6. 速度ピーク",
        vpk,
        warn_over=VEL_PEAK_WARN,
        fail_over=VEL_PEAK_FAIL,
        fmt="{:.2f}",
        detail_suffix=f" rad/s（無負荷速度 {NO_LOAD_SPEED} の {100 * vpk / NO_LOAD_SPEED:.0f}%）。"
        "飽和域だとMuJoCoの最小モデルが持たないトルク-速度特性が効き、検証誤差にモデル誤差が混ざる",
    )

    # --- 7. トルク・電流の余裕 ---
    # 安全余裕は記録全体（飛びつきを含む）で見る。インピーダンス則はトルクをclampせず、
    # ピークはほぼ必ず飛びつき区間に出るため、追従区間だけを見ると一桁小さく出て
    # 「余裕がある」と誤読する（実測: 追従区間0.33Nm に対し飛びつき込み3.10Nm）。
    tpk_all = float(np.abs(tau).max())
    tpk_track = float(np.abs(tauk).max())
    if max_torque is not None:
        frac = 100 * tpk_all / max_torque
        rep.item(
            "7. トルク余裕",
            "WARN" if frac > JUMP_MARGIN_WARN_FRAC else "PASS",
            f"ピーク {tpk_all:.2f} Nm = safety.max_torque {max_torque} の {frac:.0f}%"
            f"（うち追従区間は {tpk_track:.2f} Nm。差は飛びつき時のもの）",
        )
    else:
        rep.item("7. トルクピーク", "INFO", f"{tpk_all:.2f} Nm（追従区間 {tpk_track:.2f} Nm）")
    ipk = float(np.abs(cur).max())
    rep.item(
        "   電流ピーク",
        "WARN" if ipk > 0.9 * RATED_CURRENT else "PASS",
        f"{ipk:.3f} A（定格 {RATED_CURRENT} A の {100 * ipk / RATED_CURRENT:.0f}%）",
    )

    # --- 8. 速度デコード健全性（sysid_run_check.py の項目8と同基準） ---
    # 時刻軸は wall_time があればそちらを使う。100Hz記録では1サンプルのジッタが
    # 公称dtの数%に相当し、公称tで微分すると勾配が歪みうるため。
    time_axis = d["wall_time"][keep] if "wall_time" in d.dtype.names else tk
    fd = np.gradient(pk, time_axis)
    fd_slope = float(np.polyfit(vk, fd, 1)[0])
    fd_corr = float(np.corrcoef(vk, fd)[0, 1])
    rep.item(
        "8. 速度デコード健全性",
        "PASS" if FD_SLOPE_RANGE[0] <= fd_slope <= FD_SLOPE_RANGE[1] else "FAIL",
        f"位置の有限差分に対する回帰勾配 {fd_slope:.4f}"
        f"（許容 {FD_SLOPE_RANGE[0]}〜{FD_SLOPE_RANGE[1]}）, 相関 {fd_corr:.4f}",
    )

    # --- 9. 温度余裕 ---
    tmin, tmax = float(temp.min()), float(temp.max())
    if max_temp is not None:
        margin = max_temp - tmax
        rep.item(
            "9. 温度余裕",
            "PASS" if margin >= 5 else "WARN",
            f"{tmin:.0f} → {tmax:.0f}℃（上限 {max_temp}℃ まで {margin:.0f}℃）",
        )
    else:
        rep.item("9. 温度", "INFO", f"{tmin:.0f} → {tmax:.0f}℃")

    passed = rep.n_fail == n_fail0
    if own_rep:
        _print_verdict(rep)
    else:
        print(f"       → この試行: FAIL {rep.n_fail - n_fail0}件, WARN {rep.n_warn - n_warn0}件")
    return passed


def _print_verdict(rep):
    print("-" * 70)
    if rep.n_fail:
        print(f"  判定: 不合格（FAIL {rep.n_fail}件, WARN {rep.n_warn}件） — このデータは別軌道検証に使うべきではない")
    elif rep.n_warn:
        print(f"  判定: 条件付き合格（WARN {rep.n_warn}件） — 上記の注意点を踏まえて使用のこと")
    else:
        print("  判定: 合格 — validate_trajectory.py の検証データとして使用可能")
    print("=" * 70)


def _load_manifest(run_dir):
    """exp_009 の manifest.csv を読む（無ければ None）。

    振幅・周期・K/B はCSV本体に入らないため、条件を判定に使うにはこれが要る。
    """
    path = Path(run_dir) / "manifest.csv"
    if not path.exists():
        return None
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def check_validation_run(
    run_dir,
    expected_samples=None,
    max_temp=None,
    max_torque=None,
    max_velocity=None,
    seg_len=SEG_LEN_DEFAULT,
):
    """実行フォルダ全体（exp_009 の複数試行 / exp_008 の単発）を検証する。

    `trial_*/` サブフォルダの有無で exp_009 形式か exp_008 形式かを判別するので、
    呼び出し側はどちらか意識しなくてよい。

    Returns:
        bool: FAIL が1件もなければ True
    """
    run_dir = Path(run_dir)
    rep = RunCheckReport()
    print("=" * 70)
    print("別軌道（validation）データ 自動検証")
    print(f"  対象: {run_dir}")
    print("=" * 70)

    trial_dirs = sorted(p for p in run_dir.glob("trial_*") if p.is_dir()) if run_dir.is_dir() else []

    if trial_dirs:
        # exp_009: 試行ごとのサブフォルダ。条件は manifest.csv から引く
        cond = {int(r["trial"]): r for r in (_load_manifest(run_dir) or [])}
        periods = []
        n_ok = 0
        for td in trial_dirs:
            i = int(td.name.split("_")[1])
            c = cond.get(i)
            if c is not None and c["status"] != "completed":
                # 緊急停止・中断で終わった試行は記録が途中で切れており、
                # 追従品質や区間数を判定しても意味がない（判定不能をWARNとして残す）。
                rep.item(f"試行{i}", "WARN", f"status={c['status']} のため検証をスキップ（記録が途中で切れている）")
                continue
            n_ok += 1
            per = float(c["period"]) if c else None
            if per is not None:
                periods.append(per)
            check_trial(
                td / "log.csv",
                amplitude=float(c["amplitude"]) if c else None,
                period=per,
                expected_samples=expected_samples,
                max_temp=max_temp,
                max_torque=max_torque,
                max_velocity=max_velocity,
                seg_len=seg_len,
                label=f"試行{i}",
                rep=rep,
            )

        # --- run 全体: 試行間の条件の広がり ---
        print("\n--- 実行全体 ---")
        rep.item(
            "10. 有効な試行数",
            "PASS" if n_ok >= N_TRIALS_WARN else "WARN",
            f"{n_ok} / {len(trial_dirs)} 試行。少ないと平均が個々の条件に引きずられる",
        )
        if len(periods) >= 2:
            spread = max(periods) / min(periods)
            rep.item(
                "    周期の広がり",
                "PASS" if spread >= PERIOD_SPREAD_WARN else "WARN",
                f"{min(periods):.2f}〜{max(periods):.2f}s（最大/最小 {spread:.2f}倍）。"
                "速い試行と遅い試行で最良ステージが割れることが分かっているため、両方が要る",
            )
        else:
            rep.item("    周期の広がり", "INFO", "manifest が無いため条件の広がりは評価できない")
    else:
        # exp_008: 実行フォルダ直下に log.csv 1本（パスを直接指定されるケースも許す）
        csv_path = run_dir if run_dir.is_file() else run_dir / "log.csv"
        check_trial(
            csv_path,
            expected_samples=expected_samples,
            max_temp=max_temp,
            max_torque=max_torque,
            max_velocity=max_velocity,
            seg_len=seg_len,
            label=csv_path.parent.name,
            rep=rep,
        )

    _print_verdict(rep)
    return rep.n_fail == 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"使い方: python {sys.argv[0]} <実行フォルダ | log.csv>")
        sys.exit(1)

    # 安全しきい値は config.yaml から取る（Pi 上ではこちらが通る）。読めない環境
    # （config.yaml を持ち出していない解析用PC等）では既定値にフォールバックする。
    kwargs = dict(max_temp=75, max_torque=10.0, max_velocity=6.0)
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "control_mit_can"))
        from lib.config_loader import load_config

        cfg = load_config()
        kwargs = dict(
            max_temp=cfg["motor"]["max_temp"],
            max_torque=cfg["safety"]["max_torque"],
            max_velocity=cfg["safety"]["max_velocity"],
        )
    except Exception as e:
        print(f"（config.yaml を読めなかったため既定のしきい値を使います: {type(e).__name__}: {e}）")

    sys.exit(0 if check_validation_run(Path(sys.argv[1]), **kwargs) else 1)
