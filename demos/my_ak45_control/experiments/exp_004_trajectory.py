"""実験 004: 軌跡追従制御

この実験では、AK45-36 に複雑な軌跡を追従させます。
滑らかな動きを実現するための制御手法を評価します。

実験内容:
1. 三角波軌跡の追従
2. 速度・加速度の連続性を考慮
3. 追従誤差の分析

実行方法:
python experiments/exp_004_trajectory.py
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

# 制御パラメータ
K = config['control']['impedance']['K']
B = config['control']['impedance']['B']

# 軌跡パラメータ
AMPLITUDE = np.pi/2  # 振幅 [rad] (90°)
PERIOD = 4.0         # 周期 [秒]
RUNTIME_SECONDS = 20 # 実験時間 [秒]

# ログファイル名
timestamp = int(time.time())
LOG_FILE = f'../logs/exp004_trajectory_{timestamp}.csv'

print(f"=== 実験 004: 軌跡追従制御 ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"制御ゲイン: K={K} Nm/rad, B={B} Nm/(rad/s)")
print(f"軌跡: 三角波, 振幅={AMPLITUDE:.3f} rad, 周期={PERIOD} 秒")
print(f"ログ保存: {LOG_FILE}")
print("=" * 50)

def generate_triangle_trajectory(t, amplitude, period):
    """三角波軌跡を生成"""
    # 周期内の位置を計算 (0 to 1)
    phase = (t % period) / period

    # 三角波: 上昇と下降を繰り返す
    if phase < 0.5:
        # 上昇部分 (0 to amplitude)
        position = 2 * amplitude * phase
    else:
        # 下降部分 (amplitude to 0)
        position = 2 * amplitude * (1 - phase)

    # 中心を 0 にする
    position -= amplitude

    return position

def calculate_trajectory_velocity(t, amplitude, period, dt=0.01):
    """軌跡の速度を数値微分で計算"""
    pos_current = generate_triangle_trajectory(t, amplitude, period)
    pos_next = generate_triangle_trajectory(t + dt, amplitude, period)
    velocity = (pos_next - pos_current) / dt
    return velocity

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

    # メイン制御ループ
    print("軌跡追従開始...")
    start_time = time.time()
    loop_count = 0
    max_tracking_error = 0.0

    while time.time() - start_time < RUNTIME_SECONDS:
        loop_start = time.time()

        # 軌跡生成
        t = time.time() - start_time
        desired_pos = generate_triangle_trajectory(t, AMPLITUDE, PERIOD)
        desired_vel = calculate_trajectory_velocity(t, AMPLITUDE, PERIOD)

        # モーター制御
        motor.update()
        motor.set_output_angle_radians(desired_pos)

        # 追従誤差計算
        current_pos = motor.get_output_angle_radians()
        tracking_error = abs(desired_pos - current_pos)
        max_tracking_error = max(max_tracking_error, tracking_error)

        # 制御情報表示（200msごと）
        loop_count += 1
        if loop_count % 20 == 0:
            elapsed = time.time() - start_time
            current_vel = motor.get_output_velocity_radians_per_second()
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
          ".3f"
print(f"ログ保存完了: {LOG_FILE}")
print("実験 004 完了")