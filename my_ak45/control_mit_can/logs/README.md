# 実験ログディレクトリ

このディレクトリには、各実験のログファイルが保存されます。

## ログファイル命名規則

`exp{実験番号}_{実験名}_{タイムスタンプ}.csv`

例:
- `exp001_gain_tuning_1703123456.csv`
- `exp002_step_response_1703123567.csv`
- `exp003_multi_motor_1703123678.csv`
- `exp004_trajectory_1703123789.csv`

## ログ変数

各ログファイルには以下の変数が含まれます：

- `output_angle`: 出力角度 [rad]
- `output_velocity`: 出力速度 [rad/s]
- `output_torque`: 出力トルク [Nm]
- `mosfet_temperature`: MOSFET 温度 [℃]
- `timestamp`: タイムスタンプ [秒]

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