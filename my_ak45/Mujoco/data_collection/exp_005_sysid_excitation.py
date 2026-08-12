"""実験 005: システム同定用 multi-sine 励振信号

MuJoCo sysid toolbox 用の実機データを取得するため、AK45-36 に純トルク指令（kp=0, kd=0）で
multi-sine 励振信号を送り、応答（位置・速度・電流・トルク・温度）を CSV に記録します。

励振式（RobStride RS02 での実例を踏襲）:
    torque(t) = amp * (sin(2*pi*f*t) + 0.6*sin(2*pi*3.4*f*t) + 0.3*sin(2*pi*7.4*f*t))

詳しい手法の解説は下記を参照してください:
../docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md
../docs_syid/AK45-36_sysid_作業手順.md

注意: このスクリプトは本ワークスペース初の「開ループ」実験です。位置・速度フィードバックによる
復元力を一切持たないため、target_pos に基づく通常のインピーダンス/位置制御実験と異なり、共振や
想定外の入力によってモーターが速度を乗せて暴走するリスクがあります。安全のため
(1) コマンド段階での config.yaml safety.max_torque によるクランプ、
(2) 実測値ベースの SafetyMonitor による位置/速度/トルク超過時の緊急停止、
の2層で保護しています。それでも初回実行時は目視監視のもとで行ってください。

モーター制御自体は my_ak45/control_mit_can/ の共通基盤（lib/・config.yaml）を再利用するため、
実機（Raspberry Pi + CAN）上でのみ実行できます。一方、MuJoCo sysid のモデル最適化は別PC
（Windows、GPU利用）で行う想定のため、出力データはこのスクリプトと同じ my_ak45/Mujoco/ 配下の
data/raw/ に保存します（my_ak45/control_mit_can/logs/ とは異なり git 追跡対象）。

実行方法:
python exp_005_sysid_excitation.py
"""

import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "control_mit_can"))

import numpy as np
from lib.config_loader import load_config
from lib.logging_utils import console_log, make_realtime_loop
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position
from lib.safety_monitor import SafetyMonitor

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = ["output_angle", "output_velocity", "current", "output_torque", "mosfet_temperature"]

# 励振パラメータ（他の exp_00N と異なり、ここでは実際に config.yaml を読み込む。
# 振幅・周波数はハードウェアリスクに直結するため、コードを触らずチューニングできるようにするため）
SYSID_CONFIG = config["experiment"]["sysid_excitation"]
BASE_FREQ = SYSID_CONFIG["base_freq"]
AMPLITUDE = SYSID_CONFIG["amplitude"]
DURATION = SYSID_CONFIG["duration"]
DT = SYSID_CONFIG["dt"]
REPORT = SYSID_CONFIG["report"]

# multi-sine の harmonic 比率・重み（sysid 手法固有の固定パラメータ。
# 誤って変更されないよう config.yaml ではなくここに定数として保持する）
HARMONIC_RATIOS = (1.0, 3.4, 7.4)
HARMONIC_WEIGHTS = (1.0, 0.6, 0.3)
PEAK_TORQUE = AMPLITUDE * sum(HARMONIC_WEIGHTS)

# 安全制限パラメータ（config.yaml の safety.* を再利用、新規フィールドは追加しない）
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実行フォルダ（my_ak45/Mujoco/data/raw/exp005_sysid_excitation_{timestamp}/）を作成し、
# CSV・コンソールログをまとめる。MuJoCo sysid の最適化処理は別PC（Windows、GPU利用）で行うため、
# git 追跡対象外の my_ak45/control_mit_can/logs/ ではなく、git 追跡対象の my_ak45/Mujoco/data/raw/
# に直接保存する（lib.logging_utils.make_run_dir() は control_mit_can/logs/ 固定のためここでは使わない）。
DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "raw"
RUN_DIR = DATA_DIR / f"exp005_sysid_excitation_{int(time.time())}"
RUN_DIR.mkdir(parents=True)
LOG_FILE = str(RUN_DIR / "log.csv")


def multi_sine_torque(t, amplitude, base_freq):
    """multi-sine 励振式でトルク指令 [Nm] を計算する。

    torque(t) = amp * (sin(2*pi*f*t) + 0.6*sin(2*pi*3.4*f*t) + 0.3*sin(2*pi*7.4*f*t))
    瞬時最大値は amplitude * sum(HARMONIC_WEIGHTS)（全成分が同位相になる場合）。
    """
    return amplitude * sum(
        weight * np.sin(2 * np.pi * ratio * base_freq * t) for ratio, weight in zip(HARMONIC_RATIOS, HARMONIC_WEIGHTS)
    )


class ExcitationLogger:
    """単一モーターの励振実験ログを、指令トルクを含めて1行ずつCSVに記録するロガー。

    TMotorManager_mit_can 標準の CSV_file 機構は測定値（LOG_FUNCTIONS）のみを記録し、
    このスクリプトが計算した「指令トルク」自体は記録できない。sysid では実機に送った
    指令値そのものが MuJoCo 側で再生する「入力」になるため、明示的に記録する。
    """

    def __init__(self, csv_file, motor, log_vars):
        self.motor = motor
        self.log_vars = log_vars
        header = ["t", "desired_torque"] + list(log_vars)
        self._file = open(csv_file, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)

    def log(self, t, desired_torque):
        row = [t, desired_torque] + [self.motor.LOG_FUNCTIONS[var]() for var in self.log_vars]
        self._writer.writerow(row)

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, etype, value, tb):
        self.close()


with console_log(RUN_DIR):
    print(f"=== 実験 005: システム同定用 multi-sine 励振信号 ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"基準周波数: {BASE_FREQ} Hz, 基準振幅: {AMPLITUDE} Nm (瞬時最大: {PEAK_TORQUE:.3f} Nm)")
    print(f"記録時間: {DURATION} 秒, サンプリング周期: {DT} 秒 ({1 / DT:.0f} Hz)")
    print(f"安全上限: 位置={MAX_POSITION} rad, 速度={MAX_VELOCITY} rad/s, トルク={MAX_TORQUE} Nm")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    # モーター制御（個別CSVロギングは無効化し、ExcitationLoggerでまとめて記録する）
    with build_motor_manager(motor_config, csv_file=None, log_vars=LOG_VARS) as motor:
        # 位置ゼロ化
        zero_position(motor)

        # 電流制御モード設定
        # kp/ki/ff/spoof はダミー引数（mit_can.py set_current_gains() docstring 参照）。
        # ここでは意図的に config.yaml の control.current.Kp/Ki を渡さない：
        # 渡すと「PD整形された電流ループ」であるかのように読めてしまうが、実際は位置・速度・
        # Kp・Kdが常に0でエンコードされる純トルク指令であり、それがこの実験の要件そのものである。
        motor.set_current_gains()

        motor_name = f"{motor_config.type}(ID={motor_config.id})"
        safety_monitor = SafetyMonitor(
            [motor], [motor_name], MAX_POSITION, MAX_VELOCITY, MAX_TORQUE, emergency_stop=EMERGENCY_STOP_ENABLED
        )

        with ExcitationLogger(LOG_FILE, motor, LOG_VARS) as logger:
            print("励振開始...")
            loop = make_realtime_loop(dt=DT, report=REPORT)

            for t in loop:
                # 励振トルクを計算し、コマンド段階の安全弁としてクランプする。
                #
                # set_*() を update() より「前」に置くことが重要。update() は内部で
                # 「状態を読む → コマンドを送信」の順に処理する（mit_can.py の update() 末尾で
                # _send_command() を呼ぶ）ため、この順序なら commanded_torque は同じイテレーション
                # 内で送信される。逆順（update() が先）にすると送信が次のイテレーションまで
                # 持ち越され、CSV 上で指令列が実測列より1サンプル余計に先行してしまう。
                # 実機データ（2026-08-13）の周波数応答解析でも、この余分な先行が
                # 約1.05ms のむだ時間として現れ、1サンプルずらすと 0.25ms まで消えることを確認した。
                # sysid では入力と出力の時間対応がそのまま同定誤差になるため、ここで削っておく。
                # 詳細は .ai/logs/2026-08-13_01_* 参照。
                raw_torque = multi_sine_torque(t, AMPLITUDE, BASE_FREQ)
                commanded_torque = np.clip(raw_torque, -MAX_TORQUE, MAX_TORQUE)
                motor.set_output_torque_newton_meters(commanded_torque)

                try:
                    # 状態読み取り + commanded_torque の送信。温度上限超過時はRuntimeErrorが送出される
                    motor.update()
                except RuntimeError as e:
                    safety_monitor.trigger_emergency_stop(str(e))
                    break

                # ログ記録（指令トルク + 実測値）
                #
                # 注意: ここで読める実測値は「1つ前のイテレーションで送信したコマンド」への応答。
                # update() は状態を読んでからコマンドを送るため、指令を出す前にその応答を測ることは
                # 原理的にできず、この1サンプル分のずれはスクリプト側では解消できない。
                # MuJoCo sysid 用にデータを整形する際に、実測列を1行前に詰めて補正すること。
                logger.log(t, commanded_torque)

                # 安全制限チェック（実測値ベースの独立した安全層）
                exceeded, message = safety_monitor.check()
                if exceeded:
                    if safety_monitor.emergency_stop_enabled:
                        safety_monitor.trigger_emergency_stop(message)
                        break
                    else:
                        print(f"警告（緊急停止は無効）: {message}")

                # 進捗表示（約100msごと）
                if loop.n % max(1, int(0.1 / DT)) == 0:
                    current_pos = motor.get_output_angle_radians()
                    current_vel = motor.get_output_velocity_radians_per_second()
                    current_torque = motor.get_output_torque_newton_meters()
                    print(
                        f"経過時間: {t:.2f} 秒 | "
                        f"指令トルク: {commanded_torque:.3f} Nm | "
                        f"実測トルク: {current_torque:.3f} Nm | "
                        f"位置: {current_pos:.3f} rad | "
                        f"速度: {current_vel:.3f} rad/s"
                    )

                # 実験時間チェック
                if t >= DURATION:
                    break

            total_time = t
            expected_samples = int(DURATION / DT)
            actual_samples = loop.n
            print(f"実行時間: {total_time:.2f} 秒")
            print(f"サンプル数: 実測 {actual_samples} / 期待値 {expected_samples}")

    print(f"ログ保存完了: {RUN_DIR}")
    print("実験 005 完了")
