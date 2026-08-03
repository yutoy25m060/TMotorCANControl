"""インピーダンス制御テンプレート

このテンプレートは、AK45-36 のインピーダンス制御（位置 + 速度ゲイン）を実装します。
剛性 K [Nm/rad] と減衰 B [Nm/(rad/s)] を設定して、ばね-ダンパー系の制御を行います。

使用方法:
1. config.yaml の control.impedance.K と B を調整
2. 目標位置を set_output_angle_radians() で設定
3. 制御ループで update() を呼び出し
"""

import numpy as np
from lib.config_loader import load_config
from lib.logging_utils import make_log_path, make_realtime_loop
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = config["logging"]["vars"]

# インピーダンス制御パラメータ
K = config["control"]["impedance"]["K"]  # 剛性 [Nm/rad]
B = config["control"]["impedance"]["B"]  # 減衰 [Nm/(rad/s)]

# 実験パラメータ
TARGET_POSITION = np.pi / 2  # 目標位置 [rad] (90度)
RUNTIME_SECONDS = 15  # 実験時間 [秒]

# ログファイル名
LOG_FILE = make_log_path("impedance_control")

print(f"=== AK45-36 インピーダンス制御テンプレート ===")
print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
print(f"剛性 K: {K} Nm/rad")
print(f"減衰 B: {B} Nm/(rad/s)")
print(f"目標位置: {TARGET_POSITION:.3f} rad ({np.degrees(TARGET_POSITION):.1f}°)")
print(f"ログ保存: {LOG_FILE}")
print("=" * 50)

# モーター制御
with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
    # 位置ゼロ化
    zero_position(motor)

    # インピーダンス制御モード設定
    motor.set_impedance_gains_real_unit(K=K, B=B)

    # メイン制御ループ（NeuroLocoMiddleware使用）
    print("インピーダンス制御開始...")
    loop = make_realtime_loop()  # 100Hz制御

    for t in loop:
        # 状態更新
        motor.update()

        # 目標位置を設定（インピーダンス制御）
        motor.set_output_angle_radians(TARGET_POSITION)

        # 制御情報表示（200msごと）
        if loop.count % 20 == 0:
            current_pos = motor.get_output_angle_radians()
            current_vel = motor.get_output_velocity_radians_per_second()
            current_torque = motor.get_output_torque_newton_meters()
            error = TARGET_POSITION - current_pos

            print(
                f"経過時間: {t:.1f} 秒 | "
                f"位置: {current_pos:.3f} rad | "
                f"速度: {current_vel:.3f} rad/s | "
                f"トルク: {current_torque:.3f} Nm | "
                f"誤差: {error:.3f} rad"
            )

        # 実験時間チェック
        if t >= RUNTIME_SECONDS:
            break

    total_time = t
    print(f"実行時間: {total_time:.2f} 秒")
print(f"ログ保存完了: {LOG_FILE}")
print("インピーダンス制御実験終了")
