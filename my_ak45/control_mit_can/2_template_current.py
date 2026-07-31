"""電流制御テンプレート

このテンプレートは、AK45-36 の電流制御を実装します。
q軸電流を直接指令することで、トルクを精密に制御できます。

使用方法:
1. config.yaml の control.current.limit を調整
2. 電流指令を set_motor_current_qaxis_amps() で設定
3. 制御ループで update() を呼び出し
"""

import time
import yaml
import numpy as np
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

# 電流制御パラメータ
CURRENT_LIMIT = config["control"]["current"]["limit"]  # 電流制限 [A]
CURRENT_KP = config["control"]["current"]["Kp"]  # 比例ゲイン
CURRENT_KI = config["control"]["current"]["Ki"]  # 積分ゲイン

# 実験パラメータ
TARGET_CURRENT = 2.0  # 目標電流 [A]
RUNTIME_SECONDS = 10  # 実験時間 [秒]

# ログファイル名
timestamp = int(time.time())
LOG_FILE = f"logs/current_control_{timestamp}.csv"

print(f"=== AK45-36 電流制御テンプレート ===")
print(f"モーター: {MOTOR_TYPE} (ID: {MOTOR_ID})")
print(f"電流制限: {CURRENT_LIMIT} A")
print(f"目標電流: {TARGET_CURRENT} A")
print(f"ログ保存: {LOG_FILE}")
print("=" * 40)

# モーター制御
with TMotorManager_mit_can(
    motor_type=MOTOR_TYPE, motor_ID=MOTOR_ID, max_mosfett_temp=MAX_TEMP, CSV_file=LOG_FILE, log_vars=LOG_VARS
) as motor:
    # 位置ゼロ化
    print("位置ゼロ化を実行中...")
    motor.set_zero_position()
    time.sleep(1.5)
    print("ゼロ化完了")

    # 電流制御モード設定
    motor.set_current_gains(kp=CURRENT_KP, ki=CURRENT_KI)

    # メイン制御ループ（NeuroLocoMiddleware使用）
    print("電流制御開始...")
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)  # 100Hz制御

    for t in loop:
        # 状態更新
        motor.update()

        # 電流指令（安全のため制限内に収める）
        safe_current = np.clip(TARGET_CURRENT, -CURRENT_LIMIT, CURRENT_LIMIT)
        motor.set_motor_current_qaxis_amps(safe_current)

        # 制御情報表示（100msごと）
        if loop.count % 10 == 0:
            current_current = motor.get_current_qaxis_amps()
            current_torque = motor.get_output_torque_newton_meters()
            current_pos = motor.get_output_angle_radians()

            print(
                f"経過時間: {t:.1f} 秒 | "
                f"電流: {current_current:.3f} A | "
                f"トルク: {current_torque:.3f} Nm | "
                f"位置: {current_pos:.3f} rad"
            )

        # 実験時間チェック
        if t >= RUNTIME_SECONDS:
            break

    total_time = t
    print(f"実行時間: {total_time:.2f} 秒")
print(f"ログ保存完了: {LOG_FILE}")
print("電流制御実験終了")
