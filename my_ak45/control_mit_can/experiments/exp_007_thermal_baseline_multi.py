"""実験 007: 3台並列接続時の温度（mosfet_temperature）時系列ベースライン確認

exp_006_thermal_baseline_check.py（単一モーター版）の3台版。config.yaml の motors:
（motor1=ID1, motor2=ID2, motor3=ID3）全台について、電源投入直後から能動的な指令を
一切送らずに温度・位置・速度・電流の推移を共通タイムラインで1つのCSVに記録する。

安全上の注意（exp_006と同じ方針）:
- config.yaml の motors[].max_temp（各50℃）をそのまま使うと、観測したい65〜71℃前後の
  実測値自体が update() の温度チェックで即座に RuntimeError になる。そのため本スクリプト
  に限り、上限を実機ファームウェアの実測しきい値（l_temp_motor_start=85℃）に合わせて
  85℃に引き上げている。config.yaml のmax_tempは変更していない。
- 3台とも制御モードは切り替えない（IDLEのまま）。能動的な電流・位置・速度指令は
  一切送らない。
- 80℃以上でソフト警告、85℃以上、またはいずれかのモーターでエラーコード検出時は
  全台を安全に停止する（ExitStack が全台の power_off を保証する）。

実行方法（config.yaml / logs/ が親ディレクトリにあるため、experiments/ に移動してから実行）:
cd experiments
python exp_007_thermal_baseline_multi.py
"""

import sys
from contextlib import ExitStack
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.config_loader import load_config
from lib.logging_utils import make_log_path, make_realtime_loop
from lib.sync_logger import SyncMultiMotorLogger

from TMotorCANControl.mit_can import TMotorManager_mit_can

config = load_config()
MOTORS = config["motors"]

# このスクリプト専用の温度上限（exp_006と同じ根拠。config.yamlのmax_tempとは独立）
DIAGNOSTIC_MAX_TEMP = 85.0
SOFT_WARN_TEMP = 80.0

DURATION_SECONDS = 1200  # 観測時間（20分。15分で切り上げたい場合は900に変更）
LOG_VARS = ["output_angle", "output_velocity", "current", "mosfet_temperature"]

SYNC_LOG_FILE = make_log_path("exp007_thermal_baseline_multi")

print("=== 実験 007: 温度ベースライン時系列確認（3台並列・能動指令なし） ===")
for m in MOTORS:
    print(f"  - {m['name']}: {m['type']} (ID: {m['id']})")
print(f"観測時間: {DURATION_SECONDS} 秒")
print(f"診断用temp上限: {DIAGNOSTIC_MAX_TEMP}C（config.yamlの各motor.max_tempとは別）")
print(f"ログ保存: {SYNC_LOG_FILE}")
print("=" * 60)

motor_managers = [
    TMotorManager_mit_can(
        motor_type=m["type"],
        motor_ID=m["id"],
        max_mosfett_temp=DIAGNOSTIC_MAX_TEMP,
        CSV_file=None,
    )
    for m in MOTORS
]
motor_names = [m["name"] for m in MOTORS]

with ExitStack() as stack:
    motors = [stack.enter_context(mgr) for mgr in motor_managers]
    sync_logger = stack.enter_context(SyncMultiMotorLogger(SYNC_LOG_FILE, motors, motor_names, LOG_VARS))

    print("IDLE状態のまま3台分の温度推移を記録します（能動指令なし）...")

    first_temps = [None] * len(motors)
    last_print_time = -999
    warned = [False] * len(motors)
    aborted = False

    loop = make_realtime_loop(dt=1.0, report=False)
    for t in loop:
        for i, motor in enumerate(motors):
            try:
                motor.update()  # IDLE状態：全ゼロ指令を送るのみ
            except RuntimeError as e:
                print(f"\n{motor_names[i]}: {e} — 安全のため全台を中断します。")
                aborted = True
                break

            temp = motor.get_temperature_celsius()
            error = motor.get_motor_error_code()

            if first_temps[i] is None:
                first_temps[i] = temp

            if error != 0:
                print(f"\n{motor_names[i]}: エラーコード{error}検出 — 安全のため全台を中断します。")
                aborted = True
                break

            if temp >= SOFT_WARN_TEMP and not warned[i]:
                print(f"\n警告: {motor_names[i]}が{SOFT_WARN_TEMP}Cに到達（現在{temp:.1f}C）。")
                warned[i] = True

        if aborted:
            break

        sync_logger.log(t)

        if t - last_print_time >= 10.0:
            parts = []
            for i, motor in enumerate(motors):
                temp = motor.get_temperature_celsius()
                delta = temp - first_temps[i]
                parts.append(f"{motor_names[i]}={temp:.1f}C({delta:+.1f})")
            print(f"経過時間: {t:7.1f}秒 | " + " | ".join(parts))
            last_print_time = t

        if t >= DURATION_SECONDS:
            break

    print("=" * 60)
    for i, motor in enumerate(motors):
        final_temp = motor.get_temperature_celsius()
        print(f"{motor_names[i]}: 開始 {first_temps[i]:.1f}C → 終了 {final_temp:.1f}C (差分 {final_temp - first_temps[i]:+.1f}C)")

print(f"ログ保存完了: {SYNC_LOG_FILE}")
print("実験 007 完了")
