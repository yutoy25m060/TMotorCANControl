"""実験 002: ステップ応答特性評価

この実験では、AK45-36 のステップ応答を詳細に測定し、
制御性能を評価します。

実験内容:
1. 目標位置を 0 → 45° にステップ変化
2. 立ち上がり時間、整定時間、オーバーシュートを測定
3. 振動特性を分析

実行方法（config.yaml / logs/ が親ディレクトリにあるため、experiments/ に移動してから実行）:
cd experiments
python exp_002_step_response.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from lib.config_loader import load_config
from lib.logging_utils import (
    console_log,
    make_log_path,
    make_realtime_loop,
    make_run_dir,
)
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position
from lib.safety_monitor import SafetyMonitor

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = config["logging"]["vars"]

# 制御パラメータ（config.yaml から読み込み）
K = config["control"]["impedance"]["K"]
B = config["control"]["impedance"]["B"]

# 実験パラメータ
INITIAL_POSITION = 0.0  # 初期位置 [rad]
TARGET_POSITION = np.pi / 4  # 目標位置 [rad] (45°)
STEP_TIME = 10.0  # ステップ持続時間 [秒]
SETTLE_THRESHOLD = 0.01  # 安定判定閾値 [rad]

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実行フォルダ（logs/exp002_step_response_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("exp002_step_response")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print("=== 実験 002: ステップ応答特性評価 ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"制御ゲイン: K={K} Nm/rad, B={B} Nm/(rad/s)")
    print(f"ステップ: {INITIAL_POSITION:.3f} → {TARGET_POSITION:.3f} rad")
    print(f"安定判定閾値: {SETTLE_THRESHOLD:.3f} rad")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    # モーター制御
    with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
        # 位置/速度/トルク/温度の上限監視（超過時は全モーター＝このモーター1台を緊急停止）
        safety_monitor = SafetyMonitor(
            [motor],
            [f"{motor_config.type}(ID{motor_config.id})"],
            MAX_POSITION,
            MAX_VELOCITY,
            MAX_TORQUE,
            emergency_stop=EMERGENCY_STOP_ENABLED,
        )

        # 位置ゼロ化
        zero_position(motor)

        # インピーダンス制御設定
        motor.set_impedance_gains_real_unit(K=K, B=B)

        emergency_aborted = False

        # 初期位置で安定待ち
        print("初期位置安定待ち (3秒)...")
        loop = make_realtime_loop(report=False)
        for t in loop:
            if safety_monitor.update_and_check():
                emergency_aborted = True
                break
            motor.set_output_angle_radians(INITIAL_POSITION)
            if t >= 3.0:
                break

        if not emergency_aborted:
            # ステップ応答開始
            print("ステップ応答測定開始...")
            loop = make_realtime_loop(report=False)
            settled_time = None
            max_position = INITIAL_POSITION
            min_position = INITIAL_POSITION
            overshoot_detected = False

            for t in loop:
                if safety_monitor.update_and_check():
                    emergency_aborted = True
                    break
                motor.set_output_angle_radians(TARGET_POSITION)

                # 応答データ収集
                current_time = t  # SoftRealtimeLoop の時間を使用
                current_pos = motor.get_output_angle_radians()
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
                    print(f"オーバーシュート検出: {overshoot_amount:.3f} rad")

                # 進捗表示
                if loop.n % 50 == 0:  # 500ms ごと
                    print(f"経過時間: {current_time:.1f} 秒 | 現在位置: {current_pos:.3f} rad | 誤差: {error:.3f} rad")

                if t >= STEP_TIME:
                    break

        if emergency_aborted:
            print("安全上限超過のため、ステップ応答測定を中止しました。")
        else:
            # 実験結果サマリー（他の実験スクリプトと同様、SoftRealtimeLoop 自身の経過時間 t を使用）
            total_time = t
            final_position = motor.get_output_angle_radians()
            steady_state_error = TARGET_POSITION - final_position

            print("=== ステップ応答結果 ===")
            print(f"実行時間: {total_time:.2f} 秒")
            print(f"最終位置: {final_position:.3f} rad")
            print(f"定常状態誤差: {steady_state_error:.3f} rad")
            if overshoot_detected:
                print(f"オーバーシュート量: {overshoot_amount:.3f} rad")
            if settled_time:
                print(f"整定時間: {settled_time:.3f} 秒")
            else:
                print("安定せず")

    print(f"ログ保存完了: {RUN_DIR}")
    print("実験 002 完了")
