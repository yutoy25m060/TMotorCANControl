"""インピーダンス制御テンプレート

このテンプレートは、AK45-36 のインピーダンス制御（位置 + 速度ゲイン）を実装します。
剛性 K [Nm/rad] と減衰 B [Nm/(rad/s)] を設定して、ばね-ダンパー系の制御を行います。

使用方法:
1. config.yaml の control.impedance.K と B を調整
2. 目標位置を set_output_angle_radians() で設定
3. 制御ループで update() を呼び出し
"""

import time
import yaml
import numpy as np
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 設定ファイルの読み込み
with open('config.yaml', 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 設定の展開
MOTOR_TYPE = config['motor']['type']
MOTOR_ID = config['motor']['id']
MAX_TEMP = config['motor']['max_temp']
LOG_VARS = config['logging']['vars']

# インピーダンス制御パラメータ
K = config['control']['impedance']['K']  # 剛性 [Nm/rad]
B = config['control']['impedance']['B']  # 減衰 [Nm/(rad/s)]

# 実験パラメータ
TARGET_POSITION = np.pi/2  # 目標位置 [rad] (90度)
RUNTIME_SECONDS = 15      # 実験時間 [秒]

# ログファイル名
timestamp = int(time.time())
LOG_FILE = f'logs/impedance_control_{timestamp}.csv'

print(f"=== AK45-36 インピーダンス制御テンプレート ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"剛性 K: {K} Nm/rad")
print(f"減衰 B: {B} Nm/(rad/s)")
print(f"目標位置: {TARGET_POSITION:.3f} rad ({np.degrees(TARGET_POSITION):.1f}°)")
print(f"ログ保存: {LOG_FILE}")
print("=" * 50)

# モーター制御
with TMotorManager_mit_can(
    motor_type=MOTOR_TYPE,
    motor_ID=MOTOR_ID,
    max_mosfett_temp=MAX_TEMP,
    CSV_file=LOG_FILE,
    log_vars=LOG_VARS
) as motor:

    # 接続確認
    if not motor.check_can_connection():
        print("エラー: CAN 接続に失敗しました。")
        exit(1)

    # 位置ゼロ化
    print("位置ゼロ化を実行中...")
    motor.set_zero_position()
    time.sleep(1.5)
    print("ゼロ化完了")

    # インピーダンス制御モード設定
    motor.set_impedance_gains_real_unit(K=K, B=B)

    # メイン制御ループ
    print("インピーダンス制御開始...")
    start_time = time.time()
    loop_count = 0

    while time.time() - start_time < RUNTIME_SECONDS:
        loop_start = time.time()

        # 状態更新
        motor.update()

        # 目標位置を設定（インピーダンス制御）
        motor.set_output_angle_radians(TARGET_POSITION)

        # 制御情報表示（200msごと）
        loop_count += 1
        if loop_count % 20 == 0:
            elapsed = time.time() - start_time
            current_pos = motor.get_output_angle_radians()
            current_vel = motor.get_output_velocity_radians_per_second()
            current_torque = motor.get_output_torque_newton_meters()
            error = TARGET_POSITION - current_pos

            print(".1f"
                  ".3f"
                  ".3f"
                  ".3f"
                  ".3f")

        # 制御周期 10ms (100Hz)
        elapsed_loop = time.time() - loop_start
        if elapsed_loop < 0.01:
            time.sleep(0.01 - elapsed_loop)

    total_time = time.time() - start_time
    print(".1f"
print(f"ログ保存完了: {LOG_FILE}")
print("インピーダンス制御実験終了")