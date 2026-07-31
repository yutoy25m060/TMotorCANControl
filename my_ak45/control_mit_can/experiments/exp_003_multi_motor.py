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
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# 設定ファイルの読み込み
with open("../config.yaml", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

# 複数モーター設定
MOTORS = config.get(
    "motors",
    [
        {
            "name": "Motor_1",
            "type": config["motor"]["type"],
            "id": config["motor"]["id"],
            "max_temp": config["motor"]["max_temp"],
        },
        {
            "name": "Motor_2",
            "type": config["motor"]["type"],
            "id": config["motor"]["id"] + 1,
            "max_temp": config["motor"]["max_temp"],
        },
    ],
)

LOG_VARS = config["logging"]["vars"]

# 制御パラメータ
K = config["control"]["impedance"]["K"]
B = config["control"]["impedance"]["B"]

# 実験パラメータ
AMPLITUDE = np.pi / 4  # 振幅 [rad] (45°)
FREQUENCY = 0.5  # 周波数 [Hz]
RUNTIME_SECONDS = 20  # 実験時間 [秒]

# ログファイル名（動的生成）
timestamp = int(time.time())
log_files = []
for motor in MOTORS:
    log_file = f"../logs/exp003_{motor['name'].lower()}_{timestamp}.csv"
    log_files.append(log_file)

print(f"=== 実験 003: 複数モーター制御 ===")
print(f"制御モーター数: {len(MOTORS)}")
for motor in MOTORS:
    print(f"  - {motor['name']}: {motor['type']} (ID: {motor['id']})")
print(f"制御パラメータ: K={K}, B={B}")
print(f"軌跡: 振幅 {AMPLITUDE:.3f} rad, 周波数 {FREQUENCY} Hz")
print(f"ログ保存: {', '.join(log_files)}")
print("=" * 50)

# モーター制御（動的生成）
motor_managers = [
    TMotorManager_mit_can(
        motor_type=motor_config["type"],
        motor_ID=motor_config["id"],
        max_mosfett_temp=motor_config.get("max_temp", 50),
        CSV_file=log_files[i],
        log_vars=LOG_VARS,
    )
    for i, motor_config in enumerate(MOTORS)
]

# コンテキストマネージャとして使用（2モーターのみ対応）
if len(motor_managers) != 2:
    print(f"エラー: このスクリプトは2つのモーターのみ対応しています。現在 {len(motor_managers)} 個設定されています。")
    exit(1)

# with ブロックが電源オン/オフとログファイルの後始末を保証するため、
# 手動での __exit__ 呼び出しは不要（二重電源オフになるため行わない）
with motor_managers[0], motor_managers[1]:
    motors = motor_managers
    motor_names = [m["name"] for m in MOTORS]

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
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)  # 100Hz制御

    for t in loop:
        # 時間に基づく目標位置計算（正弦波軌跡）
        target_pos = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * t)

        # 全モーターに同じ目標位置を設定
        for motor in motors:
            motor.update()
            motor.set_output_angle_radians(target_pos)

        # 制御情報表示（200msごと）
        if loop.count % 20 == 0:
            print(f"経過時間: {t:.1f} 秒 | 目標位置: {target_pos:.3f} rad")
            for i, motor in enumerate(motors):
                pos = motor.get_output_angle_radians()
                vel = motor.get_output_velocity_radians_per_second()
                print(f"    {motor_names[i]}: pos={pos:.3f}, vel={vel:.3f}")

        # 実験時間チェック
        if t >= RUNTIME_SECONDS:
            break

    total_time = t
    print(f"実行時間: {total_time:.2f} 秒")

print(f"ログ保存完了: {', '.join(log_files)}")
print("実験 003 完了")
