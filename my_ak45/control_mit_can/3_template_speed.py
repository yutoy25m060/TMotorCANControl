"""速度制御テンプレート

このテンプレートは、AK45-36 のプレーン速度モード（インピーダンス・電流のいずれとも異なる、
位置ゲイン・フィードフォワード電流を常に0で送るモード）を実装します。
制御則は (v_des - v_actual)*kd = iq で、目標速度は set_output_velocity_radians_per_second() で
設定します（mit_can.py の set_speed_gains()/set_output_velocity_radians_per_second() 参照）。

使用方法:
1. config.yaml の control.speed.kd を調整
2. 目標速度を set_output_velocity_radians_per_second() で設定
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

# 速度制御パラメータ
SPEED_KD = config["control"]["speed"]["kd"]  # 速度ゲイン
MAX_VELOCITY = config["safety"]["max_velocity"]  # 出力軸側の速度上限 [rad/s]（安全弁として使用）

# 実験パラメータ
TARGET_VELOCITY = 1.0  # 目標速度 [rad/s]
RUNTIME_SECONDS = 10  # 実験時間 [秒]

# 実行フォルダ（logs/speed_control_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("speed_control")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print("=== AK45-36 速度制御テンプレート ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"速度ゲイン kd: {SPEED_KD}")
    print(f"目標速度: {TARGET_VELOCITY} rad/s")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 40)

    # モーター制御
    with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
        # 位置ゼロ化
        zero_position(motor)

        # 速度制御モード設定
        motor.set_speed_gains(kd=SPEED_KD)

        # メイン制御ループ（NeuroLocoMiddleware使用）
        print("速度制御開始...")
        loop = make_realtime_loop()  # 100Hz制御

        for t in loop:
            # 状態更新（必須）
            motor.update()

            # 速度指令（安全のため config.yaml の safety.max_velocity 内に収める）
            safe_velocity = np.clip(TARGET_VELOCITY, -MAX_VELOCITY, MAX_VELOCITY)
            motor.set_output_velocity_radians_per_second(safe_velocity)

            # 制御情報表示（100msごと）
            if loop.n % 10 == 0:
                current_vel = motor.get_output_velocity_radians_per_second()
                current_pos = motor.get_output_angle_radians()
                current_torque = motor.get_output_torque_newton_meters()

                print(
                    f"経過時間: {t:.1f} 秒 | "
                    f"速度: {current_vel:.3f} rad/s | "
                    f"位置: {current_pos:.3f} rad | "
                    f"トルク: {current_torque:.3f} Nm"
                )

            # 実験時間チェック
            if t >= RUNTIME_SECONDS:
                break

        total_time = t
        print(f"実行時間: {total_time:.2f} 秒")
    print(f"ログ保存完了: {RUN_DIR}")
    print("実験終了")
