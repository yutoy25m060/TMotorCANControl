"""実験 001: インピーダンスゲイン調整

この実験では、AK45-36 のインピーダンス制御における
剛性 K と減衰 B の最適値を探索します。

実験内容:
1. 異なる K, B の組み合わせでステップ応答を測定
2. 振動の有無、整定時間、定常偏差を評価
3. 最適なゲインを決定

実行方法（config.yaml / logs/ が親ディレクトリにあるため、experiments/ に移動してから実行）:
cd experiments
python exp_001_gain_tuning.py
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

# ゲイン調整パラメータ（柔らかめ→非常に硬いの順。安全上限超過時は以降のゲインセットも中止する）
GAIN_SETS = [
    {"K": 5.0, "B": 0.1, "name": "柔らかめ"},
    {"K": 10.0, "B": 0.2, "name": "標準"},
    {"K": 20.0, "B": 0.5, "name": "硬め"},
    {"K": 30.0, "B": 1.0, "name": "非常に硬い"},
]

# 実験パラメータ
TARGET_POSITION = np.pi / 4  # 45度
STEP_DURATION = 5.0  # 各ゲインでのステップ時間 [秒]
SETTLE_TIME = 2.0  # 安定待ち時間 [秒]

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実行フォルダ（logs/exp001_gain_tuning_{timestamp}/）を作成し、
# 各ゲインセットのCSV・コンソールログをまとめる
RUN_DIR = make_run_dir("exp001_gain_tuning")

with console_log(RUN_DIR):
    print("=== 実験 001: インピーダンスゲイン調整 ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"目標位置: {TARGET_POSITION:.3f} rad ({np.degrees(TARGET_POSITION):.1f}°)")
    print(f"テストするゲインセット: {len(GAIN_SETS)} 種類")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    emergency_aborted = False
    for i, gain_set in enumerate(GAIN_SETS):
        K = gain_set["K"]
        B = gain_set["B"]
        name = gain_set["name"]

        print(f"\n--- ゲインセット {i + 1}: {name} (K={K}, B={B}) ---")

        # ログファイル名
        LOG_FILE = make_log_path(RUN_DIR, f"gain_{i + 1}_{name}.csv")

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
            zero_position(motor, verbose=False)

            # ゲイン設定
            motor.set_impedance_gains_real_unit(K=K, B=B)

            # 安定待ち
            print(f"安定待ち {SETTLE_TIME} 秒...")
            loop = make_realtime_loop(report=False)  # 安定待ちはレポートなし
            for t in loop:
                if safety_monitor.update_and_check():
                    emergency_aborted = True
                    break
                motor.set_output_angle_radians(0.0)  # ゼロ位置維持
                if t >= SETTLE_TIME:
                    break

            if not emergency_aborted:
                # ステップ応答
                print(f"ステップ応答測定開始 ({STEP_DURATION} 秒)...")
                loop = make_realtime_loop(report=False)  # 測定中はレポートなし
                for t in loop:
                    if safety_monitor.update_and_check():
                        emergency_aborted = True
                        break
                    motor.set_output_angle_radians(TARGET_POSITION)

                    # 進捗表示
                    if loop.n % 50 == 0:  # 500ms ごと
                        current_pos = motor.get_output_angle_radians()
                        error = TARGET_POSITION - current_pos
                        print(f"経過時間: {t:.1f} 秒 | 現在位置: {current_pos:.3f} rad | 誤差: {error:.3f} rad")

                    if t >= STEP_DURATION:
                        break

                print(f"ログ保存: {LOG_FILE}")

        if emergency_aborted:
            print("安全上限超過のため、残りのゲインセットの測定を中止しました。")
            break

    print("\n=== 実験 001 完了 ===")
    print("ログファイルを分析して最適なゲインを決定してください。")
    print("推奨: Python/matplotlib で応答曲線をプロット")
