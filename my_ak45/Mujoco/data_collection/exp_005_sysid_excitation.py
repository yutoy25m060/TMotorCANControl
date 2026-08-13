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

# 同一ディレクトリのモジュール（control_mit_can/lib ではなくこちら側の資産）
from sysid_run_check import check_run

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

# 参照技術（RS02実験）と同じ使用可能データ量 [秒]。config.yaml の DURATION は起動直後の
# 過渡（励振が助走なしに始まるため速度が一気に乗る区間、sysid_run_check.py が検出して
# 切り捨てを促す）分の余裕を上乗せした値になっているため、切り捨てた後にこの値を
# 満たしているかを sysid_run_check.py 側で確認する。詳細は
# .ai/logs/2026-08-13_02_startup-transient-and-auto-check_01.md 参照。
TARGET_USABLE_DURATION = 10.0

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

    wall_time 列について: t は SoftRealtimeLoop が生成する「予定時刻」（dt を機械的に
    足しているだけの値で、実際にそのタイミングで通信が完了したかとは無関係。詳細は
    .ai/logs/2026-08-13_03_* 参照）であるのに対し、wall_time は各行を記録した瞬間の
    time.time()（ループ開始からの経過秒）で、真の実時刻を独立に記録したもの。
    差分 wall_time - t が、サンプルごとの実ジッタ（予定からの遅れ）に相当する。
    SoftRealtimeLoop 自体の report=True によるタイミングレポート（avg/stddev error）は
    全区間の集計値のみで、どの時刻に遅れが集中したかは分からないため、sysid のデータ
    整形時に時刻対応を正確に扱いたい場合や、特定区間（起動直後等）のジッタを個別に
    確認したい場合はこちらを使う。
    """

    def __init__(self, csv_file, motor, log_vars):
        self.motor = motor
        self.log_vars = log_vars
        header = ["t", "wall_time", "desired_torque"] + list(log_vars)
        self._file = open(csv_file, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)

    def log(self, t, wall_time, desired_torque):
        row = [t, wall_time, desired_torque] + [self.motor.LOG_FUNCTIONS[var]() for var in self.log_vars]
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
            wall_t0 = time.time()  # wall_time 列（真の実時刻）の基準点

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
                # update() 完了直後（＝このイテレーションのCAN送受信が終わった瞬間）の
                # 実時刻を記録する。t はあくまで予定時刻であり通信の遅延を反映しないため、
                # 真のジッタはこの wall_time と t の差から後で評価する。
                wall_time = time.time() - wall_t0

                # ログ記録（指令トルク + 実測値）
                #
                # 注意: ここで読める実測値は「1つ前のイテレーションで送信したコマンド」への応答。
                # update() は状態を読んでからコマンドを送るため、指令を出す前にその応答を測ることは
                # 原理的にできず、この1サンプル分のずれはスクリプト側では解消できない。
                #
                # 追記(2026-08-13): この1サンプルは「記録の帳簿上のずれ」であって、CSV上の指令-実測の
                # 遅れの全部ではない（別に電流ループの物理的なむだ時間 L≈1.85ms がある）。また
                # MuJoCo の rollout は sensor[i] = ctrl[0..i-1] への応答 という同じ規約を持つため、
                # この1サンプル分は rollout 側が自動的に合わせてくれる。したがって sysid 用の整形で
                # 実際に詰めるべき行数は1ではなく約2（＝むだ時間の分）。詳細と実測による確認は
                # my_ak45/Mujoco/identification/identify.py の DEFAULT_SHIFT のコメント参照。
                logger.log(t, wall_time, commanded_torque)

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

            # SoftRealtimeLoop はここまで一度も StopIteration を送出しない（キル
            # シグナルでのみ止まる設計）ため、上の t >= DURATION での break が唯一の
            # 正常終了経路であり、これ自体は変更できない。一方 report=True のタイミング
            # レポート（avg/stddev error 等）は loop.__del__() 内でしか出力されず、
            # __del__ はガベージコレクトされるまで呼ばれない。loop はこの後もスクリプト
            # 末尾までグローバル変数として参照が残るため、del しないとレポートは
            # console_log がstdout/stderrを復元した後（インタプリタ終了時）まで遅延し、
            # console.log に記録されないまま失われる。ここで明示的に破棄することで
            # console_log の中で確実に出力・記録させる。
            del loop  # これで __del__() が呼ばれ、report=True のタイミングレポートが console.log に残る。

    print(f"ログ保存完了: {RUN_DIR}")

    # 取得したデータがsysidに使える品質かを自動検証する。
    # モーターの with ブロックを抜けた後（＝電源オフ済み・CSVクローズ済み）に実行するため、
    # 制御ループのリアルタイム性には影響しない。結果は console_log により console.log にも残る。
    # 初回取得時、飽和域のデータであることにCSVを解析するまで気付けなかった反省による。
    print()
    try:
        check_run(
            LOG_FILE,
            base_freq=BASE_FREQ,
            harmonic_ratios=HARMONIC_RATIOS,
            expected_samples=int(DURATION / DT),
            max_temp=motor_config.max_temp,
            target_usable_duration=TARGET_USABLE_DURATION,
        )
    except Exception as e:
        # 検証はあくまで事後の付随処理であり、ここで失敗しても取得済みデータは有効。
        # 実験そのものを失敗扱いにしないよう、例外は握りつぶして通知だけ行う。
        print(f"（自動検証の実行に失敗しました: {type(e).__name__}: {e}）")
        print("（取得データ自体は保存済みです。sysid_run_check.py を単体で実行して確認してください）")

    print("実験 005 完了")
