"""実験 006: 電源投入直後の温度（mosfet_temperature）時系列ベースライン確認

背景（.ai/logs/2026-08-11_*参照）:
電源投入直後・能動的な指令なし（IDLE状態）でも mosfet_temperature が 65〜67℃程度を示す
現象が確認された。これが「この個体の通常のベースライン値」なのか「異常な発熱」なのかを
切り分けるため、電源投入直後から数分間、能動的な指令を一切送らずに温度・位置・速度・電流の
推移だけを記録する。

安全上の注意:
- 本スクリプト作成時点の config.yaml の motor.max_temp は50℃で、そのまま使うと
  このスクリプトの目的である「65℃前後の実測」自体が update() の温度チェックで
  即座に RuntimeError になってしまっていた（現在は本スクリプトの観測結果を受けて
  75℃に引き上げ済み。config.yaml 冒頭のコメント参照）。本スクリプトは config.yaml の
  値に依存せず、上限を実機ファームウェアの実測しきい値
  （45-36.McParams.McParams の l_temp_motor_start=85℃、モーター温度のディレーティング
  開始点）に合わせて 85℃ に引き上げている。FET/モーターの完全カットオフ（100℃）や
  ドライバ基板の最大許容温度（100℃、ak40-2410-1a-a1 マニュアル）よりは十分低い。
  この 85℃ という値は「新しい通常運転の上限」ではなく、あくまでこの診断スクリプト専用の
  暫定上限。config.yaml の max_temp は変更していない。
- 制御モードは一切切り替えない（IDLE のまま）ため、能動的な電流・位置・速度指令は
  送られない。dev.update() は IDLE 状態の全ゼロ指令を送るのみ。
- 80℃以上でソフト警告、85℃以上でループを自主的に中断する（ライブラリ側の
  RuntimeError による強制遮断より先に、穏やかに止める）。

実行方法（config.yaml / logs/ が親ディレクトリにあるため、experiments/ に移動してから実行）:
cd experiments
python exp_006_thermal_baseline_check.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.config_loader import load_config
from lib.logging_utils import (
    console_log,
    make_log_path,
    make_realtime_loop,
    make_run_dir,
)
from lib.motor_setup import get_motor_config

from TMotorCANControl.mit_can import TMotorManager_mit_can

# 設定ファイルの読み込み（モーター型番・IDのみ config.yaml から取得）
config = load_config()
motor_config = get_motor_config(config)

# このスクリプト専用の温度上限（上記docstring参照。config.yamlのmax_tempとは独立）
DIAGNOSTIC_MAX_TEMP = 85.0  # 実機ファームウェアの l_temp_motor_start と一致させた値
SOFT_WARN_TEMP = 80.0

DURATION_SECONDS = 300  # 観測時間（5分）
LOG_VARS = ["output_angle", "output_velocity", "current", "mosfet_temperature"]

# 実行フォルダ（logs/exp006_thermal_baseline_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("exp006_thermal_baseline")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print("=== 実験 006: 温度ベースライン時系列確認（能動指令なし） ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"観測時間: {DURATION_SECONDS} 秒")
    print(f"診断用temp上限: {DIAGNOSTIC_MAX_TEMP}C（config.yamlのmax_temp={motor_config.max_temp}Cとは別）")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 60)

    with TMotorManager_mit_can(
        motor_type=motor_config.type,
        motor_ID=motor_config.id,
        max_mosfett_temp=DIAGNOSTIC_MAX_TEMP,
        CSV_file=LOG_FILE,
        log_vars=LOG_VARS,
    ) as motor:
        # 位置ゼロ化は行わない（本測定に無関係な能動コマンドを増やさないため）
        print("IDLE状態のまま温度推移を記録します（能動指令なし）...")

        first_temp = None
        last_print_time = -999
        warned = False

        loop = make_realtime_loop(dt=1.0, report=False)
        for t in loop:
            motor.update()  # IDLE状態：全ゼロ指令を送るのみ

            temp = motor.get_temperature_celsius()
            error = motor.get_motor_error_code()

            if first_temp is None:
                first_temp = temp

            if error != 0:
                print(f"\nエラーコード検出: {error} — 安全のため中断します。")
                break

            if temp >= SOFT_WARN_TEMP and not warned:
                print(f"\n警告: 温度が{SOFT_WARN_TEMP}Cに到達（現在{temp:.1f}C）。上限{DIAGNOSTIC_MAX_TEMP}Cに近づいています。")
                warned = True

            if temp >= DIAGNOSTIC_MAX_TEMP:
                print(f"\n診断用上限{DIAGNOSTIC_MAX_TEMP}Cに到達。安全のため自主的に中断します。")
                break

            if t - last_print_time >= 5.0:
                delta = temp - first_temp
                print(
                    f"経過時間: {t:6.1f}秒 | 温度: {temp:5.1f}C (開始比 {delta:+.1f}C) | "
                    f"位置: {motor.get_output_angle_radians(): .3f} rad | "
                    f"電流: {motor.get_current_qaxis_amps(): .3f} A"
                )
                last_print_time = t

            if t >= DURATION_SECONDS:
                break

        final_temp = motor.get_temperature_celsius()
        print("=" * 60)
        print(f"開始温度: {first_temp:.1f}C → 終了温度: {final_temp:.1f}C (差分 {final_temp - first_temp:+.1f}C)")

    print(f"ログ保存完了: {RUN_DIR}")
    print("実験 006 完了")
