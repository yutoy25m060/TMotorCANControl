"""実験 008: sysid validation用の別軌道（インピーダンス制御による三角波位置追従）

`docs_syid/AK45-36_sysid_作業手順.md` フェーズ4 項目19に対応する。項目17のleave-one-run-out
交差検証（`identification/validate.py`）は同定に使ったのと同じ multi-sine 励振ランでしか
検証できておらず、「sysidに使った軌道とは別の軌道」での汎化確認がまだ済んでいない。

exp_005_sysid_excitation.py は開ループ（kp=0/kd=0の純トルク指令）だが、このスクリプトは
`my_ak45/control_mit_can/experiments/exp_004_trajectory.py` と同じインピーダンス制御
（K, B）による位置追従で三角波軌跡を辿らせる、閉ループの別軌道を実機取得する。

exp_004 との違い:
- 出力先を `my_ak45/control_mit_can/logs/`（.gitignore対象）ではなく、exp_005 と同じ
  `my_ak45/Mujoco/data/raw/`（git追跡対象）にする。Windows PC側の identification/ が
  そのまま読めるようにするため。
- `wall_time` 列を記録する（`identification/csv_adapter.py` が時刻軸として優先的に使う）。
- 目標位置（desired_pos）も記録する（診断用。csv_adapter.py は読まない）。

このデータを identification/validate.py 相当の検証に使う場合、MuJoCo側の1関節モデルは
トルク入力しか受け付けないため、`csv_adapter.build_sequences(..., torque_column="output_torque")`
を指定すること。ここでの「入力」はインピーダンス則がモーター内部で計算した結果であり、
exp_005 のような明示的な指令トルク値は存在しない（desired_pos はあくまで位置目標であり
torque_column="desired_torque" は使えない）。

実行方法（実機・Raspberry Pi上でのみ）:
    cd my_ak45/Mujoco/data_collection
    python exp_008_validation_trajectory.py

安全上の注意: 位置制御（インピーダンス制御）のため exp_005 の開ループ実験ほどのリスクは
ないが、初回実行時は目視監視のもとで行うこと。振幅・周期・K/Bは
`config.yaml` の `experiment.trajectory` / `control.impedance` を編集して調整する
（コードは変更不要）。
"""

import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "control_mit_can"))

from lib.config_loader import load_config
from lib.logging_utils import console_log, make_realtime_loop
from lib.motor_setup import build_motor_manager, get_motor_config, zero_position
from lib.safety_monitor import SafetyMonitor

# 設定ファイルの読み込み
config = load_config()
motor_config = get_motor_config(config)
LOG_VARS = ["output_angle", "output_velocity", "current", "output_torque", "mosfet_temperature"]

# 制御パラメータ（exp_004_trajectory.py と同じ config.yaml セクションを使う）
K = config["control"]["impedance"]["K"]
B = config["control"]["impedance"]["B"]

# 軌跡パラメータ
TRAJ_CONFIG = config["experiment"]["trajectory"]
AMPLITUDE = TRAJ_CONFIG["amplitude"]
PERIOD = TRAJ_CONFIG["period"]
DURATION = TRAJ_CONFIG["duration"]

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# 実行フォルダ（my_ak45/Mujoco/data/raw/exp008_validation_trajectory_{timestamp}/）。
# exp_005 と同じ理由（Windows PC側でのgit pullによる受け渡し）で、git追跡対象外の
# control_mit_can/logs/ ではなくこちらに直接保存する。
DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "raw"
RUN_DIR = DATA_DIR / f"exp008_validation_trajectory_{int(time.time())}"
RUN_DIR.mkdir(parents=True)
LOG_FILE = str(RUN_DIR / "log.csv")


def generate_triangle_trajectory(t, amplitude, period):
    """三角波軌跡を生成（exp_004_trajectory.py と同一のロジック）。"""
    phase = (t % period) / period
    if phase < 0.5:
        position = 2 * amplitude * phase
    else:
        position = 2 * amplitude * (1 - phase)
    return position - amplitude


class ValidationTrajectoryLogger:
    """目標位置とwall_timeを含めて1行ずつCSVに記録するロガー（exp_005のExcitationLoggerと同形）。"""

    def __init__(self, csv_file, motor, log_vars):
        self.motor = motor
        self.log_vars = log_vars
        header = ["t", "wall_time", "desired_pos"] + list(log_vars)
        self._file = open(csv_file, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)

    def log(self, t, wall_time, desired_pos):
        row = [t, wall_time, desired_pos] + [self.motor.LOG_FUNCTIONS[var]() for var in self.log_vars]
        self._writer.writerow(row)

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, etype, value, tb):
        self.close()


with console_log(RUN_DIR):
    print("=== 実験 008: sysid validation用の別軌道（インピーダンス制御・三角波） ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"制御ゲイン: K={K} Nm/rad, B={B} Nm/(rad/s)")
    print(f"軌跡: 三角波, 振幅={AMPLITUDE:.3f} rad, 周期={PERIOD} 秒, 実行時間={DURATION} 秒")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    with build_motor_manager(motor_config, csv_file=None, log_vars=LOG_VARS) as motor:
        zero_position(motor)
        motor.set_impedance_gains_real_unit(K=K, B=B)

        motor_name = f"{motor_config.type}(ID={motor_config.id})"
        safety_monitor = SafetyMonitor(
            [motor], [motor_name], MAX_POSITION, MAX_VELOCITY, MAX_TORQUE, emergency_stop=EMERGENCY_STOP_ENABLED
        )

        with ValidationTrajectoryLogger(LOG_FILE, motor, LOG_VARS) as logger:
            print("軌跡追従開始...")
            loop = make_realtime_loop()  # 100Hz制御（exp_004と同じ）
            wall_t0 = time.time()
            max_tracking_error = 0.0

            for t in loop:
                desired_pos = generate_triangle_trajectory(t, AMPLITUDE, PERIOD)

                if safety_monitor.update_and_check():
                    break
                motor.set_output_angle_radians(desired_pos)
                wall_time = time.time() - wall_t0

                current_pos = motor.get_output_angle_radians()
                tracking_error = abs(desired_pos - current_pos)
                max_tracking_error = max(max_tracking_error, tracking_error)

                logger.log(t, wall_time, desired_pos)

                if loop.n % 20 == 0:
                    current_vel = motor.get_output_velocity_radians_per_second()
                    print(
                        f"経過時間: {t:.1f} 秒 | "
                        f"目標位置: {desired_pos:.3f} rad | "
                        f"現在位置: {current_pos:.3f} rad | "
                        f"現在速度: {current_vel:.3f} rad/s | "
                        f"追従誤差: {tracking_error:.3f} rad"
                    )

                if t >= DURATION:
                    break

            total_time = t
            print(f"実行時間: {total_time:.2f} 秒 | 最大追従誤差: {max_tracking_error:.3f} rad")
            del loop  # report=True のタイミングレポートを console.log に残す（exp_005と同じ理由）

    print(f"ログ保存完了: {RUN_DIR}")
    print("実験 008 完了")
    print()
    print("次のステップ: このCSVをコミットしてWindows PC側でgit pullし、")
    print('identification/csv_adapter.py の build_sequences(..., torque_column="output_torque") で読み込む。')
