"""実験 003: 複数モーター制御

この実験では、複数の AK45-36 モーターを同時に制御します。
CAN バス上で複数のモーターを管理する方法を学習します。

実験内容:
1. config.yaml の motors: に設定した台数（何台でも可）のモーターを同時に制御
2. 同期した動きを実装
3. 各モーターの状態を個別に監視
4. config.yaml の safety.* に基づき、位置/速度/トルクの上限監視と
   1台でも異常なら全モーターを停止する緊急停止を実施

注意: この実験には複数のモーターが必要です。
モーター ID を config.yaml で適切に設定してください。

実行方法（config.yaml / logs/ が親ディレクトリにあるため、experiments/ に移動してから実行）:
cd experiments
python exp_003_multi_motor.py
"""

import sys
from contextlib import ExitStack
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from lib.config_loader import load_config
from lib.logging_utils import make_log_path, make_realtime_loop
from lib.motor_setup import build_motor_managers, zero_positions
from lib.safety_monitor import SafetyMonitor
from lib.sync_logger import SyncMultiMotorLogger

# 設定ファイルの読み込み
config = load_config()

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
        {
            "name": "Motor_3",
            "type": config["motor"]["type"],
            "id": config["motor"]["id"] + 2,
            "max_temp": config["motor"]["max_temp"],
        },
    ],
)

LOG_VARS = config["logging"]["vars"]

# 制御パラメータ
K = config["control"]["impedance"]["K"]
B = config["control"]["impedance"]["B"]

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実験パラメータ
AMPLITUDE = np.pi / 4  # 振幅 [rad] (45°)
FREQUENCY = 0.5  # 周波数 [Hz]
RUNTIME_SECONDS = 20  # 実験時間 [秒]

# ログファイル名（全モーターを共通タイムラインで1ファイルに記録する）
SYNC_LOG_FILE = make_log_path("exp003_multi_motor_sync")

print(f"=== 実験 003: 複数モーター制御 ===")
print(f"制御モーター数: {len(MOTORS)}")
for motor in MOTORS:
    print(f"  - {motor['name']}: {motor['type']} (ID: {motor['id']})")
print(f"制御パラメータ: K={K}, B={B}")
print(f"軌跡: 振幅 {AMPLITUDE:.3f} rad, 周波数 {FREQUENCY} Hz")
print(f"ログ保存: {SYNC_LOG_FILE}")
print("=" * 50)

# モーター制御（動的生成、CSVは同期ロガーでまとめて記録するため個別ログは無効化）
motor_managers = build_motor_managers(MOTORS)

# コンテキストマネージャとして使用（config.yaml の motors: に設定した台数分、動的に開閉する）
if not motor_managers:
    print("エラー: config.yaml の motors: にモーターが1台も設定されていません。")
    exit(1)

# ExitStack が全モーターの電源オン/オフとログファイルの後始末を保証するため、
# 手動での __exit__ 呼び出しは不要（二重電源オフになるため行わない）
with ExitStack() as stack:
    motors = [stack.enter_context(m) for m in motor_managers]
    motor_names = [m["name"] for m in MOTORS]
    sync_logger = stack.enter_context(SyncMultiMotorLogger(SYNC_LOG_FILE, motors, motor_names, LOG_VARS))
    safety_monitor = SafetyMonitor(
        motors, motor_names, MAX_POSITION, MAX_VELOCITY, MAX_TORQUE, emergency_stop=EMERGENCY_STOP_ENABLED
    )

    # 位置ゼロ化
    zero_positions(motors, motor_names)

    # ゼロ化直後の実位置を確認する診断出力・検証。
    # ゼロ化が反映されず前回実行時の実位置を引きずったまま制御開始してしまう事例が実機で確認
    # されたため、ゼロ化成功を前提にせず、大きな残留誤差があれば制御開始前に安全停止する。
    ZERO_TOLERANCE = 0.3  # rad（約17°）。正常時のばらつき(0.09〜0.25 rad程度)は許容する
    zero_failures = []
    for name, motor in zip(motor_names, motors):
        pos = motor.get_output_angle_radians()
        print(f"  {name} ゼロ化後の実位置: {pos:.4f} rad")
        if abs(pos) > ZERO_TOLERANCE:
            zero_failures.append(f"{name} (実位置 {pos:.4f} rad)")
    if zero_failures:
        message = "ゼロ化が反映されていない可能性: " + ", ".join(zero_failures)
        safety_monitor.trigger_emergency_stop(message)
        raise RuntimeError(message)

    # 制御モード設定
    for i, motor in enumerate(motors):
        motor.set_impedance_gains_real_unit(K=K, B=B)
        print(f"  {motor_names[i]} インピーダンス制御設定完了")

    # メイン制御ループ
    print("同期制御開始...")
    loop = make_realtime_loop()  # 100Hz制御

    for t in loop:
        # 時間に基づく目標位置計算（正弦波軌跡）
        target_pos = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * t)
        # config.yaml の safety.max_position を超えないようにクランプ（コマンド段階の安全弁）
        target_pos = np.clip(target_pos, -MAX_POSITION, MAX_POSITION)

        # 全モーターに同じ目標位置を設定
        try:
            for motor in motors:
                motor.update()  # 温度上限超過時はここでRuntimeErrorが送出される
                motor.set_output_angle_radians(target_pos)
        except RuntimeError as e:
            # update()自身の温度チェックはモーター単位のため、無条件で全台を緊急停止する
            safety_monitor.trigger_emergency_stop(str(e))
            break

        # 全モーターの状態を共通タイムラインで1行にまとめて記録
        sync_logger.log(t)

        # 安全制限チェック（1台でも超過したら全モーターを緊急停止）
        exceeded, message = safety_monitor.check()
        if exceeded:
            if safety_monitor.emergency_stop_enabled:
                safety_monitor.trigger_emergency_stop(message)
                break
            else:
                print(f"警告（緊急停止は無効）: {message}")

        # 制御情報表示（200msごと）
        if loop.n % 20 == 0:
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

print(f"ログ保存完了: {SYNC_LOG_FILE}")
print("実験 003 完了")
