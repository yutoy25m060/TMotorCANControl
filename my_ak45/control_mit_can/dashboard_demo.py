"""ダッシュボード配信テンプレート

制御ループを回しながら、同一LAN上の別端末のブラウザから
http://<Piのアドレス>:8000/ でモーターの状態（位置・速度・トルク・電流・温度）を
リアルタイムに閲覧できるようにします。標準ライブラリのみで実装されており、追加の
pip install は不要です（lib/dashboard_server.py 参照）。

単一モーター構成の最小例ですが、DashboardServer 自体は SafetyMonitor /
SyncMultiMotorLogger と同じく motors のリストを受け取る設計のため、複数モーター構成
（exp_003/007 スタイル）にもそのまま拡張できます。

使用方法:
1. このPiとブラウザを見る端末を同一LAN上に置く
2. python dashboard_demo.py を実行
3. コンソールに表示されるURLをブラウザで開く（他端末から見る場合はPiのIPアドレスを使用）
"""

import numpy as np
from lib.config_loader import load_config
from lib.dashboard_server import DashboardServer
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

# インピーダンス制御パラメータ（1_template_impedance.py と同じ制御則を使用）
K = config["control"]["impedance"]["K"]
B = config["control"]["impedance"]["B"]

# 実験パラメータ
TARGET_POSITION = np.pi / 4  # 目標位置 [rad] (45度)
RUNTIME_SECONDS = 120  # ブラウザで確認する時間を確保するため長めに設定

# ダッシュボード配信パラメータ（このスクリプト固有の設定。config.yaml には追加しない）
DASHBOARD_HOST = "0.0.0.0"  # 同一LAN上の別端末から見えるよう全インターフェースにバインド
DASHBOARD_PORT = 8000

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実行フォルダ（logs/dashboard_demo_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("dashboard_demo")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print("=== ダッシュボード配信テンプレート ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"目標位置: {TARGET_POSITION:.3f} rad ({np.degrees(TARGET_POSITION):.1f}°)")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
        motor_name = f"{motor_config.type}(ID{motor_config.id})"

        # 位置/速度/トルク/温度の上限監視
        safety_monitor = SafetyMonitor(
            [motor],
            [motor_name],
            MAX_POSITION,
            MAX_VELOCITY,
            MAX_TORQUE,
            emergency_stop=EMERGENCY_STOP_ENABLED,
        )

        # 位置ゼロ化
        zero_position(motor)

        # インピーダンス制御モード設定
        motor.set_impedance_gains_real_unit(K=K, B=B)

        with DashboardServer(
            [motor], [motor_name], LOG_VARS, host=DASHBOARD_HOST, port=DASHBOARD_PORT
        ) as dashboard:
            print(f"ダッシュボード: {dashboard.url}")
            print("同一LAN上の別端末のブラウザで上記URLを開いてください（認証はありません）")

            # メイン制御ループ（NeuroLocoMiddleware使用）
            print("制御開始...")
            loop = make_realtime_loop()  # 100Hz制御

            for t in loop:
                # 状態更新 + 安全上限監視。上限超過時は緊急停止してループを抜ける
                if safety_monitor.update_and_check():
                    break

                # 目標位置を設定（インピーダンス制御）
                motor.set_output_angle_radians(TARGET_POSITION)

                # ダッシュボードへ最新状態を公開（ネットワークI/Oはこの呼び出しの中では発生しない）
                dashboard.publish(t)

                # 制御情報表示（200msごと）
                if loop.n % 20 == 0:
                    current_pos = motor.get_output_angle_radians()
                    print(f"経過時間: {t:.1f} 秒 | 位置: {current_pos:.3f} rad")

                # 実験時間チェック
                if t >= RUNTIME_SECONDS:
                    break

            total_time = t
            print(f"実行時間: {total_time:.2f} 秒")
    print(f"ログ保存完了: {RUN_DIR}")
    print("ダッシュボード配信テンプレート終了")
