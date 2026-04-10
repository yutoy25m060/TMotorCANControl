"""実験 003: 複数モーター制御

この実験では、複数の AK45-36 モーターを同時に制御します。
CAN バス上で複数のモーターを管理する方法を学習します。

実験内容:
1. 2つのモーターを同時に制御
2. 同期した動きを実装
3. 各モーターの状態を個別に監視

注意: この実験には複数のモーターが必要です。
モーター ID を config.yaml で適切に設定してください。

実行方法:
python experiments/exp_003_multi_motor.py
"""

import time
import yaml
import numpy as np
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 設定ファイルの読み込み
with open('../config.yaml', 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 複数モーター設定
MOTORS = [
    {
        'type': config['motor']['type'],
        'id': config['motor']['id'],  # モーター 1 の ID
        'name': 'Motor_1'
    },
    {
        'type': config['motor']['type'],
        'id': config['motor']['id'] + 1,  # モーター 2 の ID（+1）
        'name': 'Motor_2'
    }
]

MAX_TEMP = config['motor']['max_temp']
LOG_VARS = config['logging']['vars']

# 制御パラメータ
K = config['control']['impedance']['K']
B = config['control']['impedance']['B']

# 実験パラメータ
AMPLITUDE = np.pi/4  # 振幅 [rad] (45°)
FREQUENCY = 0.5      # 周波数 [Hz]
RUNTIME_SECONDS = 20 # 実験時間 [秒]

# ログファイル名
timestamp = int(time.time())
LOG_FILE_1 = f'../logs/exp003_motor1_{timestamp}.csv'
LOG_FILE_2 = f'../logs/exp003_motor2_{timestamp}.csv'

print(f"=== 実験 003: 複数モーター制御 ===")
print(f"制御モーター数: {len(MOTORS)}")
for motor in MOTORS:
    print(f"  - {motor['name']}: {motor['type']} (ID: {motor['id']})")
print(f"制御パラメータ: K={K}, B={B}")
print(f"軌跡: 振幅 {AMPLITUDE:.3f} rad, 周波数 {FREQUENCY} Hz")
print(f"ログ保存: {LOG_FILE_1}, {LOG_FILE_2}")
print("=" * 50)

# モーター制御（with ブロックのネスト）
with TMotorManager_mit_can(
    motor_type=MOTORS[0]['type'],
    motor_ID=MOTORS[0]['id'],
    max_mosfett_temp=MAX_TEMP,
    CSV_file=LOG_FILE_1,
    log_vars=LOG_VARS
) as motor1:

    with TMotorManager_mit_can(
        motor_type=MOTORS[1]['type'],
        motor_ID=MOTORS[1]['id'],
        max_mosfett_temp=MAX_TEMP,
        CSV_file=LOG_FILE_2,
        log_vars=LOG_VARS
    ) as motor2:

        motors = [motor1, motor2]
        motor_names = [MOTORS[0]['name'], MOTORS[1]['name']]

        # 接続確認
        for i, motor in enumerate(motors):
            if not motor.check_can_connection():
                print(f"エラー: {motor_names[i]} の CAN 接続に失敗しました。")
                exit(1)

        # 位置ゼロ化
        print("全モーターの位置ゼロ化を実行中...")
        for i, motor in enumerate(motors):
            print(f"  {motor_names[i]} ゼロ化中...")
            motor.set_zero_position()
        time.sleep(1.5)
        print("全モーターゼロ化完了")

        # 制御モード設定
        for i, motor in enumerate(motors):
            motor.set_impedance_gains_real_unit(K=K, B=B)
            print(f"  {motor_names[i]} インピーダンス制御設定完了")

        # メイン制御ループ
        print("同期制御開始...")
        start_time = time.time()
        loop_count = 0

        while time.time() - start_time < RUNTIME_SECONDS:
            loop_start = time.time()

            # 時間に基づく目標位置計算（正弦波軌跡）
            t = time.time() - start_time
            target_pos = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * t)

            # 全モーターに同じ目標位置を設定
            for motor in motors:
                motor.update()
                motor.set_output_angle_radians(target_pos)

            # 制御情報表示（200msごと）
            loop_count += 1
            if loop_count % 20 == 0:
                elapsed = time.time() - start_time
                print(".1f"
                      ".3f")
                for i, motor in enumerate(motors):
                    pos = motor.get_output_angle_radians()
                    vel = motor.get_output_velocity_radians_per_second()
                    print(f"    {motor_names[i]}: pos={pos:.3f}, vel={vel:.3f}")

            # 制御周期 10ms (100Hz)
            elapsed_loop = time.time() - loop_start
            if elapsed_loop < 0.01:
                time.sleep(0.01 - elapsed_loop)

        total_time = time.time() - start_time
        print(".1f"
print(f"ログ保存完了: {LOG_FILE_1}, {LOG_FILE_2}")
print("実験 003 完了")