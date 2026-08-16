"""ダッシュボード配信テンプレート（複数モーター版）

experiments/exp_003_multi_motor.py と同じ構成（config.yaml の motors: に設定した台数の
モーターを同期制御し、SyncMultiMotorLogger で共通タイムラインのCSVに記録、SafetyMonitor で
安全監視）に、DashboardServer によるリアルタイムWebダッシュボード配信を追加したものです。
標準ライブラリのみで実装されており、追加の pip install は不要です
（lib/dashboard_server.py 参照）。

同一LAN上の別端末のブラウザから http://<Piのアドレス>:8000/ で全モーターの状態
（位置・速度・トルク・電流・温度）と、いずれかのモーターが安全上限を超えた場合の
警告バナーをリアルタイムに閲覧できます。

注意: この実験には複数のモーターが必要です。モーター ID を config.yaml で適切に設定してください。

使用方法:
1. このPiとブラウザを見る端末を同一LAN上に置く
2. python dashboard_demo_multi_motor.py を実行
3. コンソールに表示されるURLをブラウザで開く（他端末から見る場合はPiのIPアドレスを使用）
"""

from contextlib import ExitStack

import numpy as np
from lib.config_loader import load_config
from lib.dashboard_server import DashboardServer
from lib.logging_utils import (
    console_log,
    make_log_path,
    make_realtime_loop,
    make_run_dir,
)
from lib.motor_setup import build_motor_managers, zero_positions
from lib.safety_monitor import SafetyMonitor
from lib.sync_logger import SyncMultiMotorLogger

# 設定ファイルの読み込み
config = load_config()

# 複数モーター設定（config.yaml の motors: を使用。exp_003_multi_motor.py と同じフォールバック）
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
RUNTIME_SECONDS = 120  # ブラウザで確認する時間を確保するため長めに設定

# ダッシュボード配信パラメータ（このスクリプト固有の設定。config.yaml には追加しない）
DASHBOARD_HOST = "0.0.0.0"  # 同一LAN上の別端末から見えるよう全インターフェースにバインド
DASHBOARD_PORT = 8000

# 実行フォルダ（logs/dashboard_demo_multi_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("dashboard_demo_multi")
SYNC_LOG_FILE = make_log_path(RUN_DIR, "sync_log.csv")

with console_log(RUN_DIR):
    print("=== ダッシュボード配信テンプレート（複数モーター版） ===")
    print(f"制御モーター数: {len(MOTORS)}")
    for motor in MOTORS:
        print(f"  - {motor['name']}: {motor['type']} (ID: {motor['id']})")
    print(f"制御パラメータ: K={K}, B={B}")
    print(f"軌跡: 振幅 {AMPLITUDE:.3f} rad, 周波数 {FREQUENCY} Hz")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    # モーター制御（動的生成、CSVは同期ロガーでまとめて記録するため個別ログは無効化）
    motor_managers = build_motor_managers(MOTORS)

    if not motor_managers:
        print("エラー: config.yaml の motors: にモーターが1台も設定されていません。")
        exit(1)

    # ExitStack が全モーターの電源オン/オフ・ログファイル・ダッシュボードサーバーの後始末を
    # 保証するため、手動での __exit__ 呼び出しは不要
    with ExitStack() as stack:
        motors = [stack.enter_context(m) for m in motor_managers]
        motor_names = [m["name"] for m in MOTORS]
        sync_logger = stack.enter_context(SyncMultiMotorLogger(SYNC_LOG_FILE, motors, motor_names, LOG_VARS))
        safety_monitor = SafetyMonitor(
            motors, motor_names, MAX_POSITION, MAX_VELOCITY, MAX_TORQUE, emergency_stop=EMERGENCY_STOP_ENABLED
        )

        # 位置ゼロ化
        zero_positions(motors, motor_names)

        # ゼロ化直後の実位置を確認する診断出力・検証（exp_003_multi_motor.py と同じ方針）
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

        dashboard = stack.enter_context(
            DashboardServer(
                motors,
                motor_names,
                LOG_VARS,
                host=DASHBOARD_HOST,
                port=DASHBOARD_PORT,
                safety_monitor=safety_monitor,
            )
        )
        print(f"ダッシュボード: {dashboard.url}")
        print("同一LAN上の別端末のブラウザで上記URLを開いてください（認証はありません）")

        # メイン制御ループ
        print("同期制御開始...")
        loop = make_realtime_loop()  # 100Hz制御

        for t in loop:
            # 状態更新 + 安全上限監視（必須）。上限超過時は緊急停止してループを抜ける
            if safety_monitor.update_and_check():
                break

            # 時間に基づく目標位置計算（正弦波軌跡）
            target_pos = AMPLITUDE * np.sin(2 * np.pi * FREQUENCY * t)
            # config.yaml の safety.max_position を超えないようにクランプ（コマンド段階の安全弁）
            target_pos = np.clip(target_pos, -MAX_POSITION, MAX_POSITION)

            # 全モーターに同じ目標位置を設定
            for motor in motors:
                motor.set_output_angle_radians(target_pos)

            # 全モーターの状態を共通タイムラインで1行にまとめて記録
            sync_logger.log(t)

            # ダッシュボードへ最新状態を公開（ネットワークI/Oはこの呼び出しの中では発生しない）
            dashboard.publish(t)

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

    print(f"ログ保存完了: {RUN_DIR}")
    print("ダッシュボード配信テンプレート（複数モーター版）終了")
