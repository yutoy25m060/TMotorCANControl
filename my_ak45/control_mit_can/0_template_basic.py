"""基本的な制御テンプレート - あらゆる実験の出発点

このファイルは、AK45-36 モーター制御の基本的な骨組みを提供します。
すべての実験スクリプトのベースとして使用してください。

使用方法:
1. このファイルをコピーして新規実験スクリプトを作成
2. TODO の部分に制御ロジックを実装
3. config.yaml でパラメータを調整
"""

from lib.config_loader import load_config
from lib.logging_utils import (
    console_log,
    make_log_path,
    make_realtime_loop,
    make_run_dir,
)
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = config["logging"]["vars"]
RUNTIME_SECONDS = 10  # 実験時間（秒）

# 実行フォルダ（logs/basic_control_{timestamp}/）を作成し、CSV・コンソールログをまとめる
RUN_DIR = make_run_dir("basic_control")
LOG_FILE = make_log_path(RUN_DIR, "log.csv")

with console_log(RUN_DIR):
    print(f"=== AK45-36 基本制御テンプレート ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"ログ保存先: {RUN_DIR}")
    print(f"実行時間: {RUNTIME_SECONDS} 秒")
    print("=" * 40)

    # モーター制御
    with build_motor_manager(motor_config, csv_file=LOG_FILE, log_vars=LOG_VARS) as motor:
        # 位置ゼロ化（約 1.5 秒待機）
        zero_position(motor)

        # 制御モードの設定（ここではアイドルモード）
        motor.set_impedance_gains_real_unit(K=0, B=0)  # ゲインゼロでアイドル

        # メイン制御ループ（NeuroLocoMiddleware使用）
        print("制御開始...")
        loop = make_realtime_loop()  # 100Hz制御

        for t in loop:
            # 状態更新（必須）
            motor.update()

            # TODO: ここにあなたの制御ロジックを実装
            # 例:
            # motor.set_output_angle_radians(1.57)  # 90度回転
            # motor.set_motor_current_qaxis_amps(2.0)  # 2A 電流指令

            # ループ情報表示（100msごと）
            if loop.n % 10 == 0:  # 約 100ms 間隔
                print(f"経過時間: {t:.1f} 秒")

            # 実験時間チェック
            if t >= RUNTIME_SECONDS:
                break

        total_time = t
        print(f"実行時間: {total_time:.2f} 秒")
    print(f"ログ保存完了: {RUN_DIR}")
    print("実験終了")
