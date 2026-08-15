"""電流制御テンプレート

このテンプレートは、AK45-36 の電流制御を実装します。
q軸電流を直接指令することで、トルクを精密に制御できます。

使用方法:
1. config.yaml の control.current.limit を調整
2. 電流指令を set_motor_current_qaxis_amps() で設定
3. 制御ループで update() を呼び出し
"""

import numpy as np
from lib.config_loader import load_config
from lib.logging_utils import (
    console_log,
    make_log_path,
    make_realtime_loop,
    make_run_dir,
)
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = config["logging"]["vars"]

# 電流制御パラメータ
CURRENT_LIMIT = config["control"]["current"]["limit"]  # 電流制限 [A]
CURRENT_KP = config["control"]["current"]["Kp"]  # 比例ゲイン
CURRENT_KI = config["control"]["current"]["Ki"]  # 積分ゲイン

# 実験パラメータ
TARGET_CURRENT = 2.0  # 目標電流 [A]
RUNTIME_SECONDS = 10  # 実験時間 [秒]

# 実行フォルダ（logs/current_control_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("current_control")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print("=== AK45-36 電流制御テンプレート ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"電流制限: {CURRENT_LIMIT} A")
    print(f"目標電流: {TARGET_CURRENT} A")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 40)

    # モーター制御
    with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
        # 位置ゼロ化
        zero_position(motor)

        # 電流制御モード設定
        motor.set_current_gains(kp=CURRENT_KP, ki=CURRENT_KI)

        # メイン制御ループ（NeuroLocoMiddleware使用）
        print("電流制御開始...")
        loop = make_realtime_loop()  # 100Hz制御

        for t in loop:
            # 状態更新
            motor.update()

            # 電流指令（安全のため制限内に収める）
            safe_current = np.clip(TARGET_CURRENT, -CURRENT_LIMIT, CURRENT_LIMIT)
            motor.set_motor_current_qaxis_amps(safe_current)

            # 制御情報表示（100msごと）
            if loop.n % 10 == 0:
                current_current = motor.get_current_qaxis_amps()
                current_torque = motor.get_output_torque_newton_meters()
                current_pos = motor.get_output_angle_radians()

                print(
                    f"経過時間: {t:.1f} 秒 | "
                    f"電流: {current_current:.3f} A | "
                    f"トルク: {current_torque:.3f} Nm | "
                    f"位置: {current_pos:.3f} rad"
                )

            # 実験時間チェック
            if t >= RUNTIME_SECONDS:
                break

        total_time = t
        print(f"実行時間: {total_time:.2f} 秒")
    print(f"ログ保存完了: {RUN_DIR}")
    print("電流制御実験終了")
