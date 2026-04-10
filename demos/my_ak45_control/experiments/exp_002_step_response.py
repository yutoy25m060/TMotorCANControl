"""実験 002: ステップ応答特性評価

この実験では、AK45-36 のステップ応答を詳細に測定し、
制御性能を評価します。

実験内容:
1. 目標位置を 0 → 45° にステップ変化
2. 立ち上がり時間、整定時間、オーバーシュートを測定
3. 振動特性を分析

実行方法:
python experiments/exp_002_step_response.py
"""

import time
import yaml
import numpy as np
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 設定ファイルの読み込み
with open('../config.yaml', 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 設定の展開
MOTOR_TYPE = config['motor']['type']
MOTOR_ID = config['motor']['id']
MAX_TEMP = config['motor']['max_temp']
LOG_VARS = config['logging']['vars']

# 制御パラメータ（config.yaml から読み込み）
K = config['control']['impedance']['K']
B = config['control']['impedance']['B']

# 実験パラメータ
INITIAL_POSITION = 0.0     # 初期位置 [rad]
TARGET_POSITION = np.pi/4  # 目標位置 [rad] (45°)
STEP_TIME = 10.0           # ステップ持続時間 [秒]
SETTLE_THRESHOLD = 0.01    # 安定判定閾値 [rad]

# ログファイル名
timestamp = int(time.time())
LOG_FILE = f'../logs/exp002_step_response_{timestamp}.csv'

print(f"=== 実験 002: ステップ応答特性評価 ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"制御ゲイン: K={K} Nm/rad, B={B} Nm/(rad/s)")
print(f"ステップ: {INITIAL_POSITION:.3f} → {TARGET_POSITION:.3f} rad")
print(f"安定判定閾値: {SETTLE_THRESHOLD:.3f} rad")
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

    # インピーダンス制御設定
    motor.set_impedance_gains_real_unit(K=K, B=B)

    # 初期位置で安定待ち
    print("初期位置安定待ち (3秒)...")
    start_time = time.time()
    while time.time() - start_time < 3.0:
        motor.update()
        motor.set_output_angle_radians(INITIAL_POSITION)
        time.sleep(0.01)

    # ステップ応答開始
    print("ステップ応答測定開始...")
    step_start_time = time.time()
    loop_count = 0
    settled_time = None
    max_position = INITIAL_POSITION
    min_position = INITIAL_POSITION
    overshoot_detected = False

    while time.time() - step_start_time < STEP_TIME:
        loop_start = time.time()

        motor.update()
        motor.set_output_angle_radians(TARGET_POSITION)

        # 応答データ収集
        current_time = time.time() - step_start_time
        current_pos = motor.get_output_angle_radians()
        current_vel = motor.get_output_velocity_radians_per_second()
        error = TARGET_POSITION - current_pos

        # 最大・最小位置追跡
        max_position = max(max_position, current_pos)
        min_position = min(min_position, current_pos)

        # 安定判定
        if abs(error) < SETTLE_THRESHOLD and settled_time is None:
            settled_time = current_time

        # オーバーシュート検出
        if current_pos > TARGET_POSITION and not overshoot_detected:
            overshoot_detected = True
            overshoot_amount = current_pos - TARGET_POSITION
            print(".3f"
        # 進捗表示
        loop_count += 1
        if loop_count % 50 == 0:  # 500ms ごと
            print(".1f"
                  ".3f"
                  ".3f")

        # 制御周期 10ms
        elapsed_loop = time.time() - loop_start
        if elapsed_loop < 0.01:
            time.sleep(0.01 - elapsed_loop)

    # 実験結果サマリー
    total_time = time.time() - step_start_time
    final_position = motor.get_output_angle_radians()
    steady_state_error = TARGET_POSITION - final_position

    print("
=== ステップ応答結果 ===")
    print(".3f")
    print(".3f")
    print(".3f")
    if overshoot_detected:
        print(".3f")
    if settled_time:
        print(".3f")
    else:
        print("安定せず")

print(f"ログ保存完了: {LOG_FILE}")
print("実験 002 完了")