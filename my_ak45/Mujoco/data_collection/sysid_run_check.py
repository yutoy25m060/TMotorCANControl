"""sysid励振データ（exp_005_sysid_excitation.py の出力CSV）の自動検証。

2026-08-13の実機初回取得で手作業で行った検証を自動化したもの。取得したデータが
MuJoCo sysid の同定に使える品質かどうかを、その場で判定してレポートする。

背景: 初回取得では振幅1.5Nmでモーターがトルク-速度特性の飽和域に突入しており
（ピーク速度が公式無負荷速度の94%）、高速域で指令トルクの半分以下しか出ていなかった。
MuJoCoの最小モデルはトルク-速度特性を持たないため、この状態のデータで同定すると
armature/frictionloss/damping に誤差が吸収され誤った値に収束する。この失敗は
CSVを開いて解析しないと気付けなかったため、実験のたびに自動でチェックする。
詳細は .ai/logs/2026-08-13_01_* を参照。

使い方:
    # 実験スクリプトから（exp_005_sysid_excitation.py が実行後に自動で呼ぶ）
    from sysid_run_check import check_run
    check_run(csv_path, base_freq=BASE_FREQ, harmonic_ratios=HARMONIC_RATIOS)

    # 過去データに対して単体で
    python sysid_run_check.py ../data/raw/exp005_sysid_excitation_1786559877/log.csv
"""

import sys

import numpy as np

# --- 判定しきい値 ---------------------------------------------------------
# AK45-36 公式基本仕様（docs_mit_can/公式基本仕様.png）より。無負荷回転速度52rpm@24V
# （出力軸側）= 約5.45 rad/s。速度に関する判定はすべてこの値に対する比で行う。
NO_LOAD_SPEED = 5.45
# 定格電流2A・ピーク6.5A。電流がこれに張り付いていれば「電流制限」、そうでなく
# 速度だけが頭打ちなら「逆起電力（電圧）制限」と判別できる。
RATED_CURRENT = 2.0

VEL_PEAK_WARN = 0.70 * NO_LOAD_SPEED  # 3.82 rad/s: これを超えるとトルク低下が始まる領域
VEL_PEAK_FAIL = 0.85 * NO_LOAD_SPEED  # 4.63 rad/s: 明確な飽和域
VEL_HIGH_TIME_FRAC_MAX = 1.0  # VEL_PEAK_WARN 超過の時間割合の許容上限 [%]
SIGN_FLIP_FRAC_MAX = 1.0  # 指令と実測の符号反転の許容上限 [%]
SLOPE_SPREAD_MAX = 1.30  # 速度別の実測/指令勾配の 最大/最小 比の許容上限
SLOPE_MIN_BIN_FRAC = 0.05  # 勾配を評価するビンに必要な最小サンプル割合（少数ビンは交絡が強く当てにならない）
XCORR_MIN = 0.990  # 相互相関ピークの許容下限（低い＝非線形が残っている）
FD_SLOPE_RANGE = (0.80, 1.25)  # デコード速度 vs 位置有限差分 の回帰勾配の許容範囲
POS_RANGE_MIN = 0.20  # 位置の可動範囲の下限 [rad]（小さすぎると摩擦が同定できない）
SAMPLE_COUNT_FRAC_MIN = 0.99  # 期待サンプル数に対する許容下限

# wall_time 列（実時刻、2026-08-13追加）による実ジッタ評価のしきい値。
#
# 単発スパイク（1ms超）の原因は調査済みで、**このスクリプト側の処理ではなく
# Linux（非リアルタイムOS）の背景ノイズ**であることが分かっている:
#   - numpy計算・CSV書き込み・CAN通信・GCをすべて除いた最小構成（time.sleepのみ）でも
#     同じ規模のスパイクが出る。同一条件6回の反復で 1ms超は 0〜7個・最大2.38ms とばらつき、
#     実機ランの「4個・最大1.4〜3.0ms」はこのベースラインの範囲内に収まる
#   - GCイベントは0回でもスパイクは発生（GC仮説は棄却）
#   - CSVバッファのフラッシュが原因なら約173回発生するはずで桁が合わない（棄却）
#   - 実行時のCPU温度56.5℃・スロットリングなし・負荷0.16と、システムは暇な状態
# したがって単発スパイクは制御不能であり、これでFAILにすると良質なデータを捨ててしまう。
# 加えて SoftRealtimeLoop は絶対時刻を目標とするため遅れは直後のサンプルで詰めて相殺され
# （実測: 正のずれ合計+40.6ms に対し負-41.5ms、10秒間の正味-0.9ms）、CSVには wall_time が
# 記録されるので sysid 側で正確な時刻対応を復元できる。
# よって単発スパイクは WARN 止まりとし、FAIL は「遅延が蓄積している」場合に限る
# （蓄積はサンプルの取りこぼしを意味し、こちらは実際にデータを損なう）。
# 詳細は .ai/logs/2026-08-13_03_*・2026-08-13_04_* 参照。
JITTER_STD_WARN_MS = 0.15  # 実測ベースライン上限0.056msの約3倍
JITTER_STD_FAIL_MS = 0.5
JITTER_MAX_WARN_MS = 1.0  # 単発スパイクの検出（背景ノイズなのでWARN止まり）
JITTER_DRIFT_WARN_MS_PER_S = 0.05  # wall_time - t の傾き。蓄積の兆候
JITTER_DRIFT_FAIL_MS_PER_S = 0.20  # 10秒で2ms蓄積＝サンプル2個分に相当

# exp_005_sysid_excitation.py の既定値（単体実行時のフォールバック。
# 実験スクリプトから呼ぶ場合は必ず実際の値を引数で渡すこと）
DEFAULT_BASE_FREQ = 4.0
DEFAULT_HARMONIC_RATIOS = (1.0, 3.4, 7.4)


class RunCheckReport:
    """PASS/WARN/FAIL を集計しながら整形出力するヘルパー。"""

    def __init__(self):
        self.n_fail = 0
        self.n_warn = 0

    def item(self, name, status, detail):
        mark = {"PASS": "[ OK ]", "WARN": "[WARN]", "FAIL": "[FAIL]", "INFO": "[info]"}[status]
        if status == "FAIL":
            self.n_fail += 1
        elif status == "WARN":
            self.n_warn += 1
        print(f"  {mark} {name}: {detail}")

    def judge(self, name, value, warn_over=None, fail_over=None, fmt="{:.3f}", detail_suffix=""):
        """value が閾値を超えていれば WARN/FAIL を立てる（超過方向の判定）。"""
        status = "PASS"
        if fail_over is not None and value > fail_over:
            status = "FAIL"
        elif warn_over is not None and value > warn_over:
            status = "WARN"
        self.item(name, status, fmt.format(value) + detail_suffix)
        return status


def _frequency_response(t, cmd, tau, base_freq, ratios):
    """励振の各高調波での振幅比・位相遅れを同期検波で抽出し、K/T/L に分解する。

    モデル: gain(f) = K / sqrt(1+(2*pi*f*T)^2),  phase(f) = 2*pi*f*L + arctan(2*pi*f*T)
      K: 定常ゲイン（周波数によらない一定の不足）
      T: 一次遅れ時定数（高周波ほど振幅が落ちる）
      L: むだ時間（純粋な時間ずれ）
    ゲインは時間ずれの影響を受けないため、まずゲインのみから K/T を決め、
    その後に位相の残差から L を最小二乗で求める（3点しかないため
    グリッド探索で K/T/L を同時フィットすると T と L が分離しきれない）。
    """
    meas = []
    for r in ratios:
        f = base_freq * r
        s, c = np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)
        pc = complex(2 * np.mean(cmd * s), 2 * np.mean(cmd * c))
        pt = complex(2 * np.mean(tau * s), 2 * np.mean(tau * c))
        if abs(pc) < 1e-9:
            continue
        phase = np.degrees(np.angle(pc) - np.angle(pt))
        phase = (phase + 180) % 360 - 180
        meas.append((f, abs(pt) / abs(pc), phase))
    if len(meas) < 2:
        return None

    best = None
    for T in np.arange(0.0, 0.015, 0.00002):
        ks = [g * np.sqrt(1 + (2 * np.pi * f * T) ** 2) for f, g, _ in meas]
        var = float(np.var(ks))
        if best is None or var < best[0]:
            best = (var, T, float(np.mean(ks)))
    _, T, K = best

    num = den = 0.0
    for f, _, p in meas:
        w = 2 * np.pi * f
        num += w * (np.radians(p) - np.arctan(w * T))
        den += w * w
    L = num / den if den > 0 else 0.0
    return {"K": K, "T": T, "L": L, "harmonics": meas}


def report_sampling(rep, d, expected_samples=None):
    """取得の完全性（項目1）と wall_time による実ジッタを判定する。

    exp_005（開ループ励振・1kHz）と exp_009（閉ループ軌道追従・100Hz）で共通の、
    「記録そのものが健全か」を見る部分。制御則にも励振波形にも依存しないため、
    validation_run_check.py からも同じ判定・同じ文言で呼べるよう切り出してある。

    Returns:
        float: 公称サンプル周期 dt の中央値 [秒]（呼び出し側が以降の判定で使う）
    """
    t = d["t"]
    # --- 1. 取得の完全性 ---
    dt_med = float(np.median(np.diff(t)))
    if expected_samples is not None:
        frac = len(t) / expected_samples
        rep.item(
            "1. 取得の完全性",
            "PASS" if frac >= SAMPLE_COUNT_FRAC_MIN else "FAIL",
            f"{len(t)} / {expected_samples} サンプル ({100 * frac:.1f}%), 記録時間 {t[-1]:.2f}s, 公称dt {dt_med * 1000:.2f}ms",
        )
    else:
        rep.item("1. 取得の完全性", "INFO", f"{len(t)} サンプル, 記録時間 {t[-1]:.2f}s, 公称dt {dt_med * 1000:.2f}ms")

    # t列はSoftRealtimeLoopの公称時刻（dtを機械的に足しているだけ）であり、通信の遅延を
    # 反映しない。実ジッタは独立に記録された wall_time 列（無ければ評価できない。
    # 2026-08-13以前に取得したCSVには存在しない）から評価する。連続サンプル間の実時間間隔
    # real_dt と公称dtとの差を見ることで、wall_time と t の基準点のずれ（一定オフセット）に
    # 影響されずに揺らぎ（ジッタ）だけを取り出せる。
    if "wall_time" in d.dtype.names:
        real_dt = np.diff(d["wall_time"]) * 1000  # ms
        dev = real_dt - dt_med * 1000  # 公称dtからの各サンプルのずれ [ms]
        jstd, jmean, jmax = float(dev.std()), float(dev.mean()), float(np.abs(dev).max())
        # 遅延の「蓄積」を測る。単発スパイクと違いこちらは実際にデータを損なう
        # （サンプルの取りこぼし＝指令と実測の時刻対応が回復不能にずれる）ため、
        # FAIL の判定はこの傾きで行う。
        drift = (d["wall_time"] - t) * 1000  # ms
        drift_slope = float(np.polyfit(t, drift, 1)[0])  # ms/秒
        status = "FAIL" if (jstd > JITTER_STD_FAIL_MS or abs(drift_slope) > JITTER_DRIFT_FAIL_MS_PER_S) else (
            "WARN"
            if (jstd > JITTER_STD_WARN_MS or jmax > JITTER_MAX_WARN_MS or abs(drift_slope) > JITTER_DRIFT_WARN_MS_PER_S)
            else "PASS"
        )
        worst_i = int(np.argmax(np.abs(dev)))
        # 最大値だけでは「単発のスパイクか、恒常的な乱れか」が区別できないため個数も出す。
        n_over = int(np.sum(np.abs(dev) > JITTER_MAX_WARN_MS))
        rep.item(
            "   実ジッタ (wall_time)",
            status,
            f"平均{jmean:+.3f}ms, 標準偏差{jstd:.3f}ms, 最大{jmax:.3f}ms (t={t[worst_i + 1]:.2f}s付近)"
            f", {JITTER_MAX_WARN_MS}ms超は{n_over}/{len(dev)}サンプル ({100 * n_over / len(dev):.2f}%)"
            f", 蓄積{drift_slope:+.4f}ms/秒",
        )
        if n_over:
            rep.item(
                "   (参考)",
                "INFO",
                "単発スパイクはLinux（非リアルタイムOS）の背景ノイズで、本スクリプトの処理が原因ではない"
                "（numpy/CSV/CAN/GCを全て除いた最小構成でも同規模で発生することを確認済み）。"
                "SoftRealtimeLoopが直後のサンプルで詰めて相殺するため蓄積せず、実害はない",
            )
            rep.item(
                "   (対処)",
                "INFO",
                "sysid整形時は t（公称時刻）ではなく wall_time を TimeSeries の時刻軸に使うこと。"
                "指令（トルク／目標位置）は公称tで計算され実際には wall_time の時点で印加されるため、"
                "wall_time を使えばこのずれは補正される",
            )
    else:
        rep.item("   (注)", "INFO", "wall_time列がないため実ジッタは評価不可（t列は公称時刻）")

    return dt_med


def check_run(csv_path, base_freq=DEFAULT_BASE_FREQ, harmonic_ratios=DEFAULT_HARMONIC_RATIOS,
              expected_samples=None, max_temp=None, target_usable_duration=None):
    """励振データCSVを検証し、レポートを標準出力に印字する。

    Args:
        csv_path: exp_005_sysid_excitation.py が出力した log.csv のパス
        base_freq: 励振の基準周波数 [Hz]
        harmonic_ratios: 励振の高調波比
        expected_samples: 期待サンプル数（None なら完全性チェックを省略）
        max_temp: モーターの温度上限 [℃]（None なら温度余裕チェックを省略）
        target_usable_duration: 起動過渡を切り捨てた後に確保したい記録時間 [秒]
            （None なら達成判定を省略。config.yaml の duration は起動過渡分の余裕を
            上乗せした値のため、切り捨て後もこの値以上が残っているかを別途確認する）

    Returns:
        bool: FAIL が1件もなければ True
    """
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t"]
    cmd = d["desired_torque"]
    pos = d["output_angle"]
    vel = d["output_velocity"]
    cur = d["current"]
    tau = d["output_torque"]
    temp = d["mosfet_temperature"]

    rep = RunCheckReport()
    print("=" * 70)
    print("sysid励振データ 自動検証")
    print(f"  対象: {csv_path}")
    print("=" * 70)

    dt_med = report_sampling(rep, d, expected_samples)
    # --- 9. 起動過渡（先に判定して、以降の速度評価から除外する区間を決める） ---
    high = np.abs(vel) > VEL_PEAK_WARN
    trim_t = 0.0
    if high.any():
        last_high = t[np.flatnonzero(high)[-1]]
        first_high = t[np.flatnonzero(high)[0]]
        # 高速域が記録の前半1割に収まっていれば起動過渡とみなす
        if last_high < 0.1 * t[-1]:
            trim_t = float(np.ceil(last_high * 20) / 20)  # 50ms単位で切り上げ
            rep.item(
                "9. 起動過渡",
                "WARN",
                f"{VEL_PEAK_WARN:.2f} rad/s 超は t={first_high:.3f}〜{last_high:.3f}s に集中（起動直後の過渡）。"
                f" 以降の速度評価は t>={trim_t:.2f}s で行う。sysid では先頭 {trim_t:.2f}s を捨てるか励振にフェードインを入れること",
            )
        else:
            rep.item("9. 起動過渡", "PASS", f"高速域が記録全体に分散（t={first_high:.3f}〜{last_high:.3f}s）。起動過渡ではない")
    else:
        rep.item("9. 起動過渡", "PASS", f"{VEL_PEAK_WARN:.2f} rad/s 超のサンプルなし")

    # 起動過渡を切り捨てた後に、当初狙った記録時間分のデータが残っているか
    # （config.yaml の duration は起動過渡の分だけ上乗せしてあるはずなので、
    # それが実際に足りているかをここで確認する）
    usable_duration = float(t[-1] - trim_t)
    if target_usable_duration is not None:
        deficit = target_usable_duration - usable_duration
        status = "FAIL" if deficit > 0.5 else ("WARN" if deficit > 0.0 else "PASS")
        rep.item(
            "   使用可能データ量",
            status,
            f"切り捨て後 {usable_duration:.2f}s / 目標 {target_usable_duration:.2f}s"
            + (f"（{deficit:.2f}s 不足）" if deficit > 0.0 else "（目標達成）"),
        )
    else:
        rep.item("   使用可能データ量", "INFO", f"切り捨て後 {usable_duration:.2f}s")

    keep = t >= trim_t
    tk, cmdk, velk, tauk, posk = t[keep], cmd[keep], vel[keep], tau[keep], pos[keep]

    # --- 2. 速度飽和 ---
    vpeak = float(np.abs(velk).max())
    vrms = float(np.sqrt((velk**2).mean()))
    rep.judge(
        "2. 速度ピーク",
        vpeak,
        warn_over=VEL_PEAK_WARN,
        fail_over=VEL_PEAK_FAIL,
        fmt="{:.2f}",
        detail_suffix=f" rad/s (無負荷速度 {NO_LOAD_SPEED} の {100 * vpeak / NO_LOAD_SPEED:.0f}%), rms {vrms:.2f}",
    )
    high_frac = 100 * float(np.mean(np.abs(velk) > VEL_PEAK_WARN))
    rep.judge(
        "   高速域の時間割合",
        high_frac,
        warn_over=VEL_HIGH_TIME_FRAC_MAX,
        fail_over=5.0,
        fmt="{:.2f}",
        detail_suffix=f"% が {VEL_PEAK_WARN:.2f} rad/s 超",
    )

    # --- 3. トルク線形性（速度別の実測/指令勾配が一定か） ---
    min_bin = max(30, int(SLOPE_MIN_BIN_FRAC * len(tk)))
    slopes = []
    detail = []
    for lo in range(0, 6):
        m = (np.abs(velk) >= lo) & (np.abs(velk) < lo + 1) & (np.abs(cmdk) > 0.15)
        if m.sum() < min_bin:
            continue
        s = float(np.polyfit(cmdk[m], tauk[m], 1)[0])
        slopes.append(s)
        detail.append(f"{lo}-{lo + 1}:{s:.3f}")
    if len(slopes) >= 2:
        spread = max(slopes) / min(slopes) if min(slopes) > 0 else float("inf")
        rep.judge(
            "3. トルク線形性",
            spread,
            warn_over=SLOPE_SPREAD_MAX,
            fail_over=1.5,
            fmt="{:.3f}",
            detail_suffix=f" = 速度別勾配の最大/最小 ({', '.join(detail)})",
        )
    else:
        rep.item("3. トルク線形性", "INFO", f"評価可能なビンが不足（各ビン{min_bin}サンプル以上必要）")

    # --- 4. 符号反転 ---
    sig = np.abs(cmdk) > 0.3
    flip = 100 * float((sig & (np.sign(cmdk) != np.sign(tauk))).sum()) / max(int(sig.sum()), 1)
    rep.judge(
        "4. 指令と実測の符号反転",
        flip,
        warn_over=SIGN_FLIP_FRAC_MAX,
        fail_over=3.0,
        fmt="{:.2f}",
        detail_suffix="% (|指令|>0.3Nm のうち)",
    )

    # --- 5. 飽和の原因判別（電流制限か電圧制限か） ---
    imax = float(np.abs(cur).max())
    if imax > 0.9 * RATED_CURRENT:
        rep.item("5. 飽和の原因", "WARN", f"電流ピーク {imax:.3f}A が定格 {RATED_CURRENT}A に接近 → 電流制限の可能性")
    else:
        rep.item(
            "5. 飽和の原因",
            "INFO",
            f"電流ピーク {imax:.3f}A（定格 {RATED_CURRENT}A の {100 * imax / RATED_CURRENT:.0f}%）。"
            " 速度飽和があれば逆起電力（電圧）由来",
        )

    # --- 6. 遅れ（相互相関） ---
    cc = cmdk - cmdk.mean()
    mm = tauk - tauk.mean()
    lags = [(lag, float(np.corrcoef(cc[: len(cc) - lag], mm[lag:])[0, 1])) for lag in range(0, 15)]
    best_lag, best_r = max(lags, key=lambda x: x[1])
    # このラグは「記録の帳簿上のずれ（1サンプル。update() が状態読み取り→送信の順のため）」と
    # 「電流ループの物理的なむだ時間（下記7のL、約1.9サンプル）」の合計であり、CSVを静的に
    # 扱う限りこの値（約3行）が正しい補正量。ただし sysid の整形で明示的に指定する行数は2で
    # あって3ではない。これは sysid_rollout の時刻付け（sensordata の先頭が初期状態なのに
    # 時刻は dt から始まる）によって約1サンプル分が暗黙に補正されるためで、上の「1サンプル」
    # とは別物。詳細は my_ak45/Mujoco/identification/identify.py の DEFAULT_SHIFT のコメント参照。
    rep.item(
        "6. 指令-実測の遅れ",
        "PASS" if best_r >= XCORR_MIN else ("WARN" if best_r >= 0.97 else "FAIL"),
        f"相互相関ピーク {best_lag * dt_med * 1000:.0f}ms (相関 {best_r:.4f}, ラグ0で {lags[0][1]:.4f})。"
        f" この値は帳簿上のずれ1行と物理的なむだ時間(下記7のL)の合計。sysid整形で明示指定するshiftは2（残り1行はMuJoCo側が暗黙に補正）",
    )

    # --- 7. 周波数応答（K/T/L 分解） ---
    fr = _frequency_response(tk, cmdk, tauk, base_freq, harmonic_ratios)
    if fr is not None:
        h = ", ".join(f"{f:.1f}Hz:振幅比{g:.3f}/位相{p:.1f}deg" for f, g, p in fr["harmonics"])
        rep.item(
            "7. 周波数応答",
            "INFO",
            f"定常ゲインK={fr['K']:.3f}, 一次遅れT={fr['T'] * 1000:.2f}ms, むだ時間L={fr['L'] * 1000:.2f}ms",
        )
        rep.item("   高調波内訳", "INFO", h)

    # --- 8. 速度デコード健全性 ---
    fd = np.gradient(posk, tk)
    fd_slope = float(np.polyfit(velk, fd, 1)[0])
    fd_corr = float(np.corrcoef(velk, fd)[0, 1])
    ok = FD_SLOPE_RANGE[0] <= fd_slope <= FD_SLOPE_RANGE[1]
    rep.item(
        "8. 速度デコード健全性",
        "PASS" if ok else "FAIL",
        f"位置の有限差分に対する回帰勾配 {fd_slope:.4f}（許容 {FD_SLOPE_RANGE[0]}〜{FD_SLOPE_RANGE[1]}）, 相関 {fd_corr:.4f}",
    )

    # --- 10. 励振の十分性 ---
    prange = float(posk.max() - posk.min())
    rep.item(
        "10. 励振の十分性",
        "PASS" if prange >= POS_RANGE_MIN else "WARN",
        f"位置の可動範囲 {prange:.3f} rad（下限 {POS_RANGE_MIN} rad。小さすぎると摩擦が同定できない）",
    )

    # --- 11. 温度余裕 ---
    tmin, tmax = float(temp.min()), float(temp.max())
    if max_temp is not None:
        margin = max_temp - tmax
        rep.item(
            "11. 温度余裕",
            "PASS" if margin >= 5 else "WARN",
            f"{tmin:.0f} → {tmax:.0f}℃（上限 {max_temp}℃ まで {margin:.0f}℃）",
        )
    else:
        rep.item("11. 温度", "INFO", f"{tmin:.0f} → {tmax:.0f}℃")

    print("-" * 70)
    if rep.n_fail:
        print(f"  判定: 不合格（FAIL {rep.n_fail}件, WARN {rep.n_warn}件） — このデータはsysidに使うべきではない")
    elif rep.n_warn:
        print(f"  判定: 条件付き合格（WARN {rep.n_warn}件） — 上記の注意点を踏まえて使用のこと")
    else:
        print("  判定: 合格 — sysidデータとして使用可能")
    print("=" * 70)
    return rep.n_fail == 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"使い方: python {sys.argv[0]} <log.csv> [base_freq]")
        sys.exit(1)
    freq = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BASE_FREQ
    sys.exit(0 if check_run(sys.argv[1], base_freq=freq) else 1)
