"""実験 009: sysid validation用の別軌道を、振幅・周期・K・Bをランダム化して連続取得する

`exp_008_validation_trajectory.py`（インピーダンス制御による三角波追従、1回の実行=1条件）を
複数条件に拡張したもの。`AK45-36_sysid_作業手順.md` フェーズ4 項目19の「条件を変えた追加データ
（振幅違い等）もここで検討する」に対応し、config.yaml を手で書き換えて何度も実行し直す代わりに、
1回の実行で複数試行（既定5回）を連続取得する。

各試行は振幅・周期・K・Bを `config.yaml` の `experiment.trajectory_randomized.*_range` から
一様分布でサンプリングする（乱数シード固定で再現性を確保）。試行ごとに独立したサブフォルダへ
CSVを保存し、実際にサンプリングされたパラメータは実行フォルダ直下の `manifest.csv` にまとめる
（CSV自体には desired_pos はあっても K/B の値は入らないため、後で参照するにはこれが必要）。

安全対策（既存スクリプトからの積み増し分）:
- 振幅とKの組み合わせ次第では、試行開始直後の追従誤差（最大で振幅そのもの）に対して
  K*amplitude が safety.max_torque を大きく超えるコマンドになりうる（インピーダンス則は
  トルクをclampしない）。サンプリング後に `_clamp_amplitude_for_torque()` で
  K*amplitude が max_torque の80%を超えないよう振幅を切り詰める（安全マージン込みの
  ソフト制限。実測トルクの最終防御は従来通り SafetyMonitor が担う）。
- 1試行が緊急停止で終わった場合、以降の試行は実行しない（電源が既に落ちているため）。
  KeyboardInterrupt（Ctrl+C）で中断した場合も同様に以降の試行を打ち切り、
  それまでの manifest だけを書き出してから終了する。
- 試行間はゼロ化 + 静定待ち（`rest_time`）を挟み、次の試行の軌跡が急な位置ジャンプから
  始まらないようにする（それでも各試行の開始点は三角波の位相0=-amplitudeなので、
  ゼロ位置からの初期ジャンプ自体はexp_004/exp_008と同じく残る）。

実行方法（実機・Raspberry Pi上でのみ）:
    cd my_ak45/Mujoco/data_collection
    python exp_009_validation_trajectory_randomized.py

初回実行時は目視監視のもとで行うこと。試行回数・範囲は config.yaml の
experiment.trajectory_randomized を編集して調整する（コードは変更不要）。
"""

import csv
import random
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

RAND_CONFIG = config["experiment"]["trajectory_randomized"]
N_TRIALS = RAND_CONFIG["n_trials"]
DURATION_PER_TRIAL = RAND_CONFIG["duration_per_trial"]
AMPLITUDE_RANGE = RAND_CONFIG["amplitude_range"]
PERIOD_RANGE = RAND_CONFIG["period_range"]
K_RANGE = RAND_CONFIG["K_range"]
B_RANGE = RAND_CONFIG["B_range"]
REST_TIME = RAND_CONFIG["rest_time"]
SEED = RAND_CONFIG["seed"]

# 安全制限パラメータ
MAX_POSITION = config["safety"]["max_position"]
MAX_VELOCITY = config["safety"]["max_velocity"]
MAX_TORQUE = config["safety"]["max_torque"]
EMERGENCY_STOP_ENABLED = config["safety"]["emergency_stop"]

# K*amplitude（初期ジャンプ時の想定最大トルク）が max_torque を大きく超えないための
# ソフト制限の余裕率。1.0にすると理論上ぎりぎりmax_torqueに達するまで許すことになり、
# 実機の遅れ・オーバーシュートを考えると危険なため8割に抑える。値そのものは
# sysid手法とは無関係な安全パラメータなのでconfig.yamlではなくここに定数として持つ
# （exp_005_sysid_excitation.py の HARMONIC_RATIOS と同じ考え方）。
TORQUE_SAFETY_MARGIN = 0.8

# 実行フォルダ（my_ak45/Mujoco/data/raw/exp009_validation_trajectory_randomized_{timestamp}/）
DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "raw"
RUN_DIR = DATA_DIR / f"exp009_validation_trajectory_randomized_{int(time.time())}"
RUN_DIR.mkdir(parents=True)


def generate_triangle_trajectory(t, amplitude, period):
    """三角波軌跡を生成（exp_004_trajectory.py と同一のロジック）。"""
    phase = (t % period) / period
    if phase < 0.5:
        position = 2 * amplitude * phase
    else:
        position = 2 * amplitude * (1 - phase)
    return position - amplitude


def clamp_amplitude_for_torque(amplitude, K, max_torque, margin):
    """K*amplitude が max_torque*margin を超えないよう振幅を切り詰める。"""
    limit = margin * max_torque / K
    return min(amplitude, limit)


def sample_trial_params(rng):
    """振幅・周期・K・Bを一様分布からサンプリングし、トルク安全制限を適用する。"""
    amplitude = rng.uniform(*AMPLITUDE_RANGE)
    period = rng.uniform(*PERIOD_RANGE)
    K = rng.uniform(*K_RANGE)
    B = rng.uniform(*B_RANGE)
    amplitude = clamp_amplitude_for_torque(amplitude, K, MAX_TORQUE, TORQUE_SAFETY_MARGIN)
    return dict(amplitude=amplitude, period=period, K=K, B=B)


class ValidationTrajectoryLogger:
    """目標位置とwall_timeを含めて1行ずつCSVに記録するロガー（exp_008と同形）。"""

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


def run_trial(motor, safety_monitor, trial_idx, params, duration, trial_dir):
    """1試行分の軌跡追従を実行する。

    Returns:
        (status, max_tracking_error): status は "completed" | "emergency_stop" | "interrupted"。
    """
    trial_dir.mkdir(parents=True)
    log_file = str(trial_dir / "log.csv")
    amplitude, period, K, B = params["amplitude"], params["period"], params["K"], params["B"]

    print(f"\n--- 試行 {trial_idx}: 振幅={amplitude:.3f} rad, 周期={period:.2f} 秒, K={K:.2f}, B={B:.3f} ---")

    zero_position(motor, label=f"試行{trial_idx}")
    motor.set_impedance_gains_real_unit(K=K, B=B)

    max_tracking_error = 0.0
    status = "completed"

    with ValidationTrajectoryLogger(log_file, motor, LOG_VARS) as logger:
        loop = make_realtime_loop()  # 100Hz制御
        wall_t0 = time.time()

        try:
            for t in loop:
                desired_pos = generate_triangle_trajectory(t, amplitude, period)

                if safety_monitor.update_and_check():
                    status = "emergency_stop"
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
                        f"  経過時間: {t:.1f} 秒 | 目標: {desired_pos:.3f} rad | "
                        f"現在: {current_pos:.3f} rad | 速度: {current_vel:.3f} rad/s | "
                        f"追従誤差: {tracking_error:.3f} rad"
                    )

                if t >= duration:
                    break
        except KeyboardInterrupt:
            status = "interrupted"
            print(f"  試行{trial_idx}: Ctrl+Cで中断されました")
        finally:
            del loop  # report=True のタイミングレポートをconsole.logに残す

    print(f"  試行{trial_idx} 結果: {status} | 最大追従誤差: {max_tracking_error:.3f} rad")
    return status, max_tracking_error


with console_log(RUN_DIR):
    print("=== 実験 009: sysid validation用の別軌道（ランダム化・複数試行） ===")
    print(f"モーター: {motor_config.type} (ID: {motor_config.id})")
    print(f"試行回数: {N_TRIALS} | 1試行あたり: {DURATION_PER_TRIAL} 秒 | 乱数シード: {SEED}")
    print(f"振幅範囲: {AMPLITUDE_RANGE} rad | 周期範囲: {PERIOD_RANGE} 秒")
    print(f"K範囲: {K_RANGE} Nm/rad | B範囲: {B_RANGE} Nm/(rad/s)")
    print(f"ログ保存先: {RUN_DIR}")
    print("=" * 50)

    rng = random.Random(SEED)
    manifest_rows = []

    with build_motor_manager(motor_config, csv_file=None, log_vars=LOG_VARS) as motor:
        motor_name = f"{motor_config.type}(ID={motor_config.id})"
        safety_monitor = SafetyMonitor(
            [motor], [motor_name], MAX_POSITION, MAX_VELOCITY, MAX_TORQUE, emergency_stop=EMERGENCY_STOP_ENABLED
        )

        for trial_idx in range(N_TRIALS):
            params = sample_trial_params(rng)
            trial_dir = RUN_DIR / f"trial_{trial_idx:02d}"

            status, max_tracking_error = run_trial(motor, safety_monitor, trial_idx, params, DURATION_PER_TRIAL, trial_dir)
            manifest_rows.append(
                dict(
                    trial=trial_idx,
                    status=status,
                    amplitude=params["amplitude"],
                    period=params["period"],
                    K=params["K"],
                    B=params["B"],
                    duration=DURATION_PER_TRIAL,
                    max_tracking_error=max_tracking_error,
                )
            )

            if status != "completed":
                print(f"\n{status} のため以降の試行（{trial_idx + 1}〜{N_TRIALS - 1}）は実行しません。")
                break

            if trial_idx < N_TRIALS - 1:
                print(f"  次の試行まで {REST_TIME} 秒静定...")
                time.sleep(REST_TIME)

    manifest_path = RUN_DIR / "manifest.csv"
    with open(manifest_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(manifest_rows[0].keys()))
        writer.writeheader()
        writer.writerows(manifest_rows)

    n_completed = sum(1 for r in manifest_rows if r["status"] == "completed")
    print(f"\n完了: {n_completed}/{N_TRIALS} 試行")
    print(f"manifest: {manifest_path}")
    print(f"ログ保存完了: {RUN_DIR}")
    print("実験 009 完了")
    print()
    print("次のステップ: このディレクトリ（trial_*/log.csv・manifest.csv）をコミットして")
    print("Windows PC側でgit pullし、各trial_*/log.csvをcsv_adapter.build_sequences(...,")
    print('torque_column="output_torque") で読み込む（K/Bの値そのものはCSVに含まれず、')
    print("必要なら manifest.csv を参照する）。")
