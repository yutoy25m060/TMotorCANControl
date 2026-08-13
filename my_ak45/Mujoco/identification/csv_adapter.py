"""実機CSV（exp_005_sysid_excitation.py の出力）を mujoco.sysid の入力形式へ変換するアダプタ。

作業手順書 `docs_syid/AK45-36_sysid_作業手順.md` フェーズ3 の項目9・10・14 に対応する。

変換にあたって、実機データ側の3つの事情を吸収する:

1. **時刻軸は `t` ではなく `wall_time` を使う**
   `t` は SoftRealtimeLoop が dt を機械的に足しているだけの「予定時刻」で、実際にその
   タイミングで通信が完了したかとは無関係（差分の標準偏差が常に0.000msになる）。
   指令トルクは公称 `t` で計算され実際には `wall_time` の時点で印加されるため、
   `wall_time` を時刻軸にすればこのずれは補正される。
   詳細は .ai/logs/2026-08-13_03_* 参照。

2. **起動直後の過渡区間を捨てる**
   multi-sine 励振が助走なしにゼロから始まるため、最初の半周期で正味の力積が入り
   速度が飽和域近くまで一気に乗る。この区間は追従品質が悪く、MuJoCoの最小モデルが
   持たないトルク-速度特性の影響を受けるため同定に使わない。
   詳細は .ai/logs/2026-08-13_02_* 参照。

3. **軌道を短い区間に分割する**
   開ループ（フィードバックによる復元力がない）系のため初期状態鋭敏性が強く、同一励振を
   独立に2回実行しても位置の差が時間とともに増大し t=10s で可動範囲の19%に達する。
   実機自身がこれだけ非決定的にばらつく以上、10秒通しを1シーケンスとして同定すると
   フィットが再現性の限界に埋もれる。区間長Lを短くすると原理的に再現できない量は
   L=10sで可動範囲比61% → L=1sで8.8% → L=0.5sで4.5% と急減する。
   詳細は .ai/logs/2026-08-13_02_* 参照。

使い方:
    from csv_adapter import build_sequences
    names, states, ctrls, sensors = build_sequences(csv_path, model, seg_len=0.5)
"""

import numpy as np
from mujoco import sysid

# 起動過渡の判定しきい値。data_collection/sysid_run_check.py の VEL_PEAK_WARN
# （= 0.70 * 公式無負荷速度5.45 rad/s）と同じ値。同スクリプトはこの判定を check_run() の
# 中にインラインで持っており公開関数として取り出せないため、ここに移植している。
# 判定ルールを変える場合は両方を揃えること。
VEL_PEAK_WARN = 0.70 * 5.45

# 実機CSVの列名（exp_005_sysid_excitation.py の ExcitationLogger が書き出すヘッダ）
TORQUE_COLUMNS = ("desired_torque", "output_torque")


def load_run(csv_path):
    """実機CSVを numpy の構造化配列として読み込む。

    sysid_run_check.py と同じ np.genfromtxt(names=True) 方式に揃えてある
    （列名でアクセスでき、列の増減に強いため）。

    Returns:
        (data, time_column_name): data は構造化配列、time_column_name は
        時刻軸として使うべき列名（"wall_time" または "t"）。
    """
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    if "wall_time" in d.dtype.names:
        return d, "wall_time"
    # 2026-08-13以前に取得したCSVには wall_time 列がない。公称時刻 t で代用するが、
    # t にはジッタが載らないため、指令と実測の時刻対応が最大1ms程度ずれる。
    print(f"警告: {csv_path} に wall_time 列がありません。公称時刻 t で代用します（時刻対応の精度が落ちます）")
    return d, "t"


def startup_trim_time(t, vel):
    """起動直後の過渡区間の終わり（この時刻以降を使う）を返す [秒]。

    判定ルールは sysid_run_check.py の「9. 起動過渡」と同一:
    VEL_PEAK_WARN を超える最後のサンプルが記録の前半1割に収まっていれば起動過渡とみなし、
    その時刻を50ms単位で切り上げた値を返す。全体に分散していれば起動過渡ではないので 0.0。
    """
    high = np.abs(vel) > VEL_PEAK_WARN
    if not high.any():
        return 0.0
    last_high = t[np.flatnonzero(high)[-1]]
    if last_high < 0.1 * t[-1]:
        return float(np.ceil(last_high * 20) / 20)  # 50ms単位で切り上げ
    return 0.0


def segment_starts(t, vel, seg_len, crossing_offset=0):
    """速度ゼロ交差のうち、区間開始点として使うもののインデックスを返す。

    切り出し点を速度ゼロ交差に選ぶのは、区間先頭の初期速度をほぼゼロとして扱えるため。
    速度は瞬時値でノイズ（実測0.10〜0.14 rad/s）が乗るが、位置は有限差分と一致しており
    信頼できる。なお局所多項式フィット等での速度の平滑化は、必要な窓幅が励振の最高調波
    （29.6Hz、周期34ms）に匹敵してしまい実信号まで潰すため有効な対策にならない。

    Args:
        crossing_offset: 先頭から読み飛ばすゼロ交差の個数。切り出し点を変えた場合に
            同定結果が一致するか（＝初期状態鋭敏性の影響を受けていないか）を確認するために使う。
    """
    sign = np.sign(vel)
    crossings = np.flatnonzero(sign[:-1] * sign[1:] < 0)
    crossings = crossings[crossing_offset:]
    starts = []
    last_t = -np.inf
    for i in crossings:
        if t[i] - last_t >= seg_len:
            starts.append(int(i))
            last_t = t[i]
    return starts


def build_sequences(
    csv_path,
    model,
    seg_len=0.5,
    torque_column="desired_torque",
    shift=2,
    crossing_offset=0,
    run_label=None,
):
    """実機CSVを sysid.ModelSequences に渡せる4つのリストへ変換する。

    Args:
        csv_path: log.csv のパス
        model: コンパイル済みの mujoco.MjModel（センサー名の解決と状態ベクトル生成に使う）
        seg_len: 区間長 [秒]
        torque_column: MuJoCo への入力に使う列。"desired_torque"（指令値）が既定。
            "output_torque"（実測値）にすると、モーター内蔵電流ループの定常ゲイン
            K=0.817・一次遅れ T=1.24ms の影響を除いた「純粋な機械パラメータ」が得られるが、
            その絶対スケールは Kt_actual の既知の誤り（公式0.11 Nm/A に対し約+10%）を直接受ける。
        shift: 実測列を何行前に詰めるか（既定2）。CSV上の指令と実測のずれは
            「記録の帳簿上のずれ1サンプル」＋「電流ループの物理的なむだ時間 約1.9サンプル
            （L=1.82〜1.87ms）」に分解でき、前者は MuJoCo の rollout が同じ規約
            （sensor[i] = ctrl[0..i-1] への応答）を持つため自動的に合う。したがって
            ここで補正するのは後者だけで約2サンプルになる。
            リポジトリ内の「1行」「3行」という記述との関係を含む詳細は
            identification/identify.py の DEFAULT_SHIFT のコメントを参照。
        crossing_offset: segment_starts() に渡す（切り出し点を変えた頑健性確認用）
        run_label: シーケンス名の接頭辞。省略時はCSVの親ディレクトリ名。

    Returns:
        (names, initial_states, control_ts_list, sensor_ts_list)
        いずれも区間数と同じ長さのリストで、そのまま sysid.ModelSequences に渡せる。
    """
    if torque_column not in TORQUE_COLUMNS:
        raise ValueError(f"torque_column は {TORQUE_COLUMNS} のいずれかを指定してください: {torque_column!r}")

    d, time_col = load_run(csv_path)
    t_nominal = d["t"]
    time = d[time_col]
    torque = d[torque_column]
    pos = d["output_angle"]
    vel = d["output_velocity"]

    # 起動過渡の切り捨て。判定は公称時刻 t で行う（sysid_run_check.py と同じ基準にするため）。
    trim_t = startup_trim_time(t_nominal, vel)
    keep = t_nominal >= trim_t
    t_nominal, time, torque, pos, vel = (a[keep] for a in (t_nominal, time, torque, pos, vel))

    n_step = int(round(seg_len / (model.opt.timestep)))
    starts = segment_starts(t_nominal, vel, seg_len, crossing_offset=crossing_offset)

    if run_label is None:
        from pathlib import Path

        run_label = Path(csv_path).parent.name

    names, states, controls, sensors = [], [], [], []
    for seq, i0 in enumerate(starts):
        i1 = i0 + n_step
        # shift の分だけ実測側を先に読むため、末尾に余裕がない区間は捨てる
        if i1 + shift > len(time):
            break
        # 区間ごとに時刻を0起点へ振り直す。区間の絶対時刻には意味がなく（重力トルクも
        # ばねもないため力学は位置・時刻に依存しない）、MuJoCo側は各シーケンスを
        # t=0 から回すため。
        tt = time[i0:i1] - time[i0]
        if not (np.diff(tt) > 0).all():
            # TimeSeries は時刻が厳密増加であることを要求する。実機の wall_time は
            # 全ランで単調増加を確認済みだが、将来のデータで壊れた場合に備えて弾く。
            raise ValueError(f"{csv_path} の区間 {seq}（t={t_nominal[i0]:.3f}s〜）で時刻が単調増加していません")

        ms = slice(i0 + shift, i1 + shift)
        names.append(f"{run_label}_seg{seq:02d}")
        states.append(sysid.create_initial_state(model, np.array([pos[ms.start]]), np.array([vel[ms.start]])))
        controls.append(sysid.TimeSeries(tt, torque[i0:i1].reshape(-1, 1)))
        sensors.append(sysid.TimeSeries.from_names(tt, np.column_stack([pos[ms], vel[ms]]), model))

    return names, states, controls, sensors
