# 実験ログディレクトリ

このディレクトリには、各実験のログファイルが保存されます。

## ログファイル命名規則

命名パターンは実験スクリプトごとに異なります：

- `0_template_basic.py`: `basic_control_{タイムスタンプ}.csv`
- `1_template_impedance.py`: `impedance_control_{タイムスタンプ}.csv`
- `2_template_current.py`: `current_control_{タイムスタンプ}.csv`
- `exp_001_gain_tuning.py`: `exp001_gain_{連番}_{ゲインセット名}_{タイムスタンプ}.csv`（ゲインセットごとに1ファイル）
- `exp_002_step_response.py`: `exp002_step_response_{タイムスタンプ}.csv`
- `exp_003_multi_motor.py`: `exp003_{モーター名}_{タイムスタンプ}.csv`（**モーターごとに1ファイル**、2モーター構成なら計2ファイル）
- `exp_004_trajectory.py`: `exp004_trajectory_{タイムスタンプ}.csv`

例:
- `exp002_step_response_1703123567.csv`
- `exp003_motor1_1703123678.csv`
- `exp003_motor2_1703123678.csv`
- `exp004_trajectory_1703123789.csv`

## ログ変数

CSV の1列目は経過時間 `pi_time` [秒] で、それ以降の列は `config.yaml` の `logging.vars` で指定した変数です（デフォルトは以下）：

- `pi_time`: モーター制御開始からの経過時間 [秒]
- `output_angle`: 出力角度 [rad]
- `output_velocity`: 出力速度 [rad/s]
- `output_torque`: 出力トルク [Nm]
- `mosfet_temperature`: MOSFET 温度 [℃]

## データ分析

ログデータを分析するには、以下のツールを使用できます：

### Python での分析例

```python
import pandas as pd
import matplotlib.pyplot as plt

# ログファイル読み込み
df = pd.read_csv('exp001_gain_tuning_1703123456.csv')

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
data = readtable('exp001_gain_tuning_1703123456.csv');

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