"""実験 001: インピーダンスゲイン調整

この実験では、AK45-36 のインピーダンス制御における
剛性 K と減衰 B の最適値を探索します。

実験内容:
1. 異なる K, B の組み合わせでステップ応答を測定
2. 振動の有無、整定時間、定常偏差を評価
3. 最適なゲインを決定

実行方法:
python experiments/exp_001_gain_tuning.py
"""

import time
import yaml
import numpy as np
from TMotorCANControl.mit_can import TMotorManager_mit_can
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# 設定ファイルの読み込み
with open("../config.yaml", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

# 設定の展開
MOTOR_TYPE = config["motor"]["type"]
MOTOR_ID = config["motor"]["id"]
MAX_TEMP = config["motor"]["max_temp"]
LOG_VARS = config["logging"]["vars"]

# ゲイン調整パラメータ
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

print(f"=== 実験 001: インピーダンスゲイン調整 ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"目標位置: {TARGET_POSITION:.3f} rad ({np.degrees(TARGET_POSITION):.1f}°)")
print(f"テストするゲインセット: {len(GAIN_SETS)} 種類")
print("=" * 50)

for i, gain_set in enumerate(GAIN_SETS):
    K = gain_set["K"]
    B = gain_set["B"]
    name = gain_set["name"]

    print(f"\n--- ゲインセット {i + 1}: {name} (K={K}, B={B}) ---")

    # ログファイル名
    timestamp = int(time.time())
    LOG_FILE = f"../logs/exp001_gain_{i + 1}_{name}_{timestamp}.csv"

    # モーター制御
    with TMotorManager_mit_can(
        motor_type=MOTOR_TYPE, motor_ID=MOTOR_ID, max_mosfett_temp=MAX_TEMP, CSV_file=LOG_FILE, log_vars=LOG_VARS
    ) as motor:
        # 接続確認
        if not motor.check_can_connection():
            print("エラー: CAN 接続に失敗しました。")
            continue

        # 位置ゼロ化
        motor.set_zero_position()
        time.sleep(1.5)

        # ゲイン設定
        motor.set_impedance_gains_real_unit(K=K, B=B)

        # 安定待ち
        print(f"安定待ち {SETTLE_TIME} 秒...")
        loop = SoftRealtimeLoop(dt=0.01, report=False, fade=0)  # 安定待ちはレポートなし
        for t in loop:
            motor.update()
            motor.set_output_angle_radians(0.0)  # ゼロ位置維持
            if t >= SETTLE_TIME:
                break

        # ステップ応答
        print(f"ステップ応答測定開始 ({STEP_DURATION} 秒)...")
        loop = SoftRealtimeLoop(dt=0.01, report=False, fade=0)  # 測定中はレポートなし
        for t in loop:
            motor.update()
            motor.set_output_angle_radians(TARGET_POSITION)

            # 進捗表示
            if loop.count % 50 == 0:  # 500ms ごと
                current_pos = motor.get_output_angle_radians()
                error = TARGET_POSITION - current_pos
                print(f"経過時間: {t:.1f} 秒 | 現在位置: {current_pos:.3f} rad | 誤差: {error:.3f} rad")

            if t >= STEP_DURATION:
                break

        print(f"ログ保存: {LOG_FILE}")

print("\n=== 実験 001 完了 ===")
print("ログファイルを分析して最適なゲインを決定してください。")
print("推奨: Python/matplotlib で応答曲線をプロット")
