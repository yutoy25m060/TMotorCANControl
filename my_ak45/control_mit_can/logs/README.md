# 実験ログディレクトリ

このディレクトリには、各実験のログファイルが保存されます。

## フォルダ構成

スクリプトを実行するたびに、`logs/` 直下に `{実験名}_{タイムスタンプ}/` という実行フォルダが
1つ自動作成され（`lib/logging_utils.py` の `make_run_dir()`）、その実行で生成するCSV・
コンソールログはすべてこのフォルダの下にまとめて保存されます。

各フォルダには必ず `console.log`（その実行中にターミナルに表示された内容の複製。進捗表示・
警告・未捕捉の例外のトレースバックを含む）が入ります。CSVのファイル名は実験スクリプトごとに
異なります：

- `0_template_basic.py`: `logs/basic_control_{タイムスタンプ}/log.csv`
- `1_template_impedance.py`: `logs/impedance_control_{タイムスタンプ}/log.csv`
- `2_template_current.py`: `logs/current_control_{タイムスタンプ}/log.csv`
- `exp_001_gain_tuning.py`: `logs/exp001_gain_tuning_{タイムスタンプ}/gain_{連番}_{ゲインセット名}.csv`（1回の実行フォルダの中に、ゲインセットごとに1ファイル）
- `exp_002_step_response.py`: `logs/exp002_step_response_{タイムスタンプ}/log.csv`
- `exp_003_multi_motor.py`: `logs/exp003_multi_motor_{タイムスタンプ}/sync_log.csv`（**全モーターを共通タイムラインで1ファイルに記録**。`sync_logger.py` の `SyncMultiMotorLogger` を使用）
- `exp_004_trajectory.py`: `logs/exp004_trajectory_{タイムスタンプ}/log.csv`
- `exp_006_thermal_baseline_check.py`: `logs/exp006_thermal_baseline_{タイムスタンプ}/log.csv`
- `exp_007_thermal_baseline_multi.py`: `logs/exp007_thermal_baseline_multi_{タイムスタンプ}/sync_log.csv`

`exp_005_sysid_excitation.py` は `my_ak45/Mujoco/data_collection/` に移動しており、出力先も
このディレクトリではなく `my_ak45/Mujoco/data/raw/exp005_sysid_excitation_{タイムスタンプ}/` に
変更されている（MuJoCo sysid の最適化を別PCで行うため、git 追跡対象の場所に直接保存する設計。
詳細は [`my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`](../../Mujoco/docs_syid/AK45-36_sysid_作業手順.md) 参照）。

例:
- `logs/exp002_step_response_1703123567/log.csv`
- `logs/exp002_step_response_1703123567/console.log`
- `logs/exp003_multi_motor_1703123678/sync_log.csv`
- `logs/exp003_multi_motor_1703123678/console.log`

## ログ変数

`exp_003_multi_motor.py` 以外（0/1/2番テンプレート、`exp_001`/`exp_002`/`exp_004`）は
`TMotorManager_mit_can` が直接CSVを書き出す方式で、CSV の1列目は経過時間 `pi_time` [秒]、
それ以降の列は `config.yaml` の `logging.vars` で指定した変数です（デフォルトは以下）：

- `pi_time`: モーター制御開始からの経過時間 [秒]
- `output_angle`: 出力角度 [rad]
- `output_velocity`: 出力速度 [rad/s]
- `output_torque`: 出力トルク [Nm]
- `mosfet_temperature`: MOSFET 温度 [℃]

`exp_003_multi_motor.py` は `sync_logger.py` の `SyncMultiMotorLogger` を使って
**複数モーターを1つの共通タイムラインで記録**します。1列目は制御ループ共通の経過時間 `t` [秒]、
それ以降は `{モーター名}_{変数名}`（例: `motor1_output_angle`, `motor2_output_torque`）という
列名で、モーターの台数分・`logging.vars` の変数分だけ列が並びます。

## データ分析

ログデータを分析するには、以下のツールを使用できます：

### Python での分析例

```python
import pandas as pd
import matplotlib.pyplot as plt

# ログファイル読み込み
df = pd.read_csv('exp002_step_response_1703123567/log.csv')

# 時間軸の作成
df['time'] = df.index / 100  # 100Hz サンプリングの場合

# プロット
plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(df['time'], df['output_angle'])
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.title('Output Angle')

plt.subplot(2, 2, 2)
plt.plot(df['time'], df['output_velocity'])
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.title('Output Velocity')

plt.subplot(2, 2, 3)
plt.plot(df['time'], df['output_torque'])
plt.xlabel('Time [s]')
plt.ylabel('Torque [Nm]')
plt.title('Output Torque')

plt.subplot(2, 2, 4)
plt.plot(df['time'], df['mosfet_temperature'])
plt.xlabel('Time [s]')
plt.ylabel('Temperature [℃]')
plt.title('MOSFET Temperature')

plt.tight_layout()
plt.show()
```

### MATLAB での分析例

```matlab
% ログファイル読み込み
data = readtable('exp002_step_response_1703123567/log.csv');

% 時間軸の作成
time = (0:length(data.output_angle)-1)' / 100;

% プロット
figure;

subplot(2, 2, 1);
plot(time, data.output_angle);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('Output Angle');

subplot(2, 2, 2);
plot(time, data.output_velocity);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('Output Velocity');

subplot(2, 2, 3);
plot(time, data.output_torque);
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Output Torque');

subplot(2, 2, 4);
plot(time, data.mosfet_temperature);
xlabel('Time [s]');
ylabel('Temperature [℃]');
title('MOSFET Temperature');
```

## 注意事項

- ログファイルは実験ごとに自動生成されます
- 大容量のログファイルは適宜削除してください
- 重要な実験データはバックアップを取ってください