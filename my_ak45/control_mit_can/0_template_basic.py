"""基本的な制御テンプレート - あらゆる実験の出発点

このファイルは、AK45-36 モーター制御の基本的な骨組みを提供します。
すべての実験スクリプトのベースとして使用してください。

使用方法:
1. このファイルをコピーして新規実験スクリプトを作成
2. TODO の部分に制御ロジックを実装
3. config.yaml でパラメータを調整
"""

import time
import yaml
from TMotorCANControl.mit_can import TMotorManager_mit_can
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# 設定ファイルの読み込み
with open("config.yaml", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

# 設定の展開
MOTOR_TYPE = config["motor"]["type"]
MOTOR_ID = config["motor"]["id"]
MAX_TEMP = config["motor"]["max_temp"]
LOG_VARS = config["logging"]["vars"]
RUNTIME_SECONDS = 10  # 実験時間（秒）

# ログファイル名（タイムスタンプ付き）
timestamp = int(time.time())
LOG_FILE = f"logs/basic_control_{timestamp}.csv"

print(f"=== AK45-36 基本制御テンプレート ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"ログ保存: {LOG_FILE}")
print(f"実行時間: {RUNTIME_SECONDS} 秒")
print("=" * 40)

# モーター制御
with TMotorManager_mit_can(
    motor_type=MOTOR_TYPE, motor_ID=MOTOR_ID, max_mosfett_temp=MAX_TEMP, CSV_file=LOG_FILE, log_vars=LOG_VARS
) as motor:
    # 位置ゼロ化（約 1.5 秒待機）
    print("位置ゼロ化を実行中...")
    motor.set_zero_position()
    time.sleep(1.5)
    print("ゼロ化完了")

    # 制御モードの設定（ここではアイドルモード）
    motor.set_impedance_gains_real_unit(K=0, B=0)  # ゲインゼロでアイドル

    # メイン制御ループ（NeuroLocoMiddleware使用）
    print("制御開始...")
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)  # 100Hz制御

    for t in loop:
        # 状態更新（必須）
        motor.update()

        # TODO: ここにあなたの制御ロジックを実装
        # 例:
        # motor.set_output_angle_radians(1.57)  # 90度回転
        # motor.set_motor_current_qaxis_amps(2.0)  # 2A 電流指令

        # ループ情報表示（100msごと）
        if loop.count % 10 == 0:  # 約 100ms 間隔
            print(f"経過時間: {t:.1f} 秒")

        # 実験時間チェック
        if t >= RUNTIME_SECONDS:
            break

    total_time = t
    print(f"実行時間: {total_time:.2f} 秒")
print(f"ログ保存完了: {LOG_FILE}")
print("実験終了")
