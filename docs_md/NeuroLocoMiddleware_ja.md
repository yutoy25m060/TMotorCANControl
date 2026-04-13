# NeuroLocoMiddleware - ロボティクス制御ミドルウェア

## 概要

`NeuroLocoMiddleware` は、ロボティクス制御システムの開発を支援するための Python ライブラリです。TMotorCANControl プロジェクトでは、オプションの依存関係として使用され、主にリアルタイム制御ループの実装とシステム同定を支援します。

### 開発元
- **開発者**: Neurobionics Lab (ミシガン大学)
- **目的**: ロボティクス制御システムのプロトタイピングと開発支援
- **言語**: Python 3.x
- **ライセンス**: MIT License (推定)

## 主な機能

### 1. SoftRealtimeLoop - ソフトリアルタイム制御ループ

制御ループのタイミング管理とリアルタイム実行を支援するクラスです。

#### 特徴
- 指定した周期でのループ実行
- タイミングの自動調整
- 統計情報のレポート
- フェードイン/アウト機能

#### 基本的な使用方法

```python
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# 10ms周期（100Hz）の制御ループを作成
loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.1)

for t in loop:
    # 制御コードをここに記述
    motor.update()
    motor.set_output_angle_radians(target_position)
```

#### パラメータ説明

- `dt`: ループ周期 [秒]（例: 0.01 = 100Hz）
- `report`: 統計レポートを表示するかどうか
- `fade`: フェードイン/アウトの持続時間 [秒]

#### TMotorCANControl での使用例

```python
from TMotorCANControl.mit_can import TMotorManager_mit_can
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

with TMotorManager_mit_can(motor_type='AK80-9', motor_ID=3) as motor:
    motor.set_zero_position()
    time.sleep(1.5)

    # インピーダンス制御設定
    motor.set_impedance_gains_real_unit(K=10.0, B=0.5)

    # リアルタイム制御ループ
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
    for t in loop:
        motor.update()
        motor.position = 3.14  # πラジアン（180°）
```

### 2. Chirp - システム同定用チャープ信号

周波数スイープ信号を生成し、システムの周波数応答を分析するためのクラスです。

#### 特徴
- 線形チャープ信号の生成
- 周波数範囲の指定
- システム同定に最適化

#### 基本的な使用方法

```python
from NeuroLocoMiddleware.SysID import Chirp

# 25Hzから250Hzまで1秒間のチャープ信号
chirp = Chirp(end_freq=250, start_freq=25, duration=1)

# 信号生成
for t in time_points:
    signal_value = chirp.next(t)
```

#### TMotorCANControl での使用例

```python
from TMotorCANControl.mit_can import TMotorManager_mit_can
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp

def chirp_demo(motor, amplitude=1.0):
    print("チャープ信号によるシステム同定開始")

    # チャープ信号設定
    chirp = Chirp(250, 25, 1)  # 250Hz→25Hz, 1秒

    # 電流制御モードに設定
    motor.set_current_gains()

    # 制御ループ
    loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0.1)
    for t in loop:
        motor.update()

        # チャープ信号を電流指令として使用
        desired_current = loop.fade * amplitude * chirp.next(t)
        motor.current_qaxis = desired_current

# 使用例
with TMotorManager_mit_can(motor_type='AK80-9', motor_ID=3) as motor:
    chirp_demo(motor, amplitude=2.0)
```

## TMotorCANControl での使用状況

### 使用ファイル一覧

NeuroLocoMiddleware は以下のファイルで使用されています：

#### MIT CAN デモ
- `demo_current_chirp_mit_can.py` - 電流チャープ制御
- `demo_current_step_mit_can.py` - 電流ステップ制御
- `demo_position_step_mit_can.py` - 位置ステップ制御
- `demo_position_tracking_mit_can.py` - 位置追従制御
- `demo_two_DOF_mit_can.py` - 2自由度制御

#### Servo Serial デモ
- `demo_velocity_servo_serial.py` - 速度制御
- `demo_current_step_servo_serial.py` - 電流ステップ制御
- `demo_position_step_servo_serial.py` - 位置ステップ制御
- `demo_position_tracking_servo_serial.py` - 位置追従制御
- その他複数のデモスクリプト

#### ユーティリティ
- `record_servo_log.py` - ログ記録スクリプト

### 使用パターン

#### パターン1: 基本的な制御ループ
```python
loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
for t in loop:
    motor.update()
    # 制御指令
```

#### パターン2: システム同定
```python
chirp = Chirp(end_freq, start_freq, duration)
loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0.1)
for t in loop:
    signal = chirp.next(t)
    motor.current_qaxis = signal
```

## インストール方法

### PyPI からのインストール

```bash
pip install NeuroLocoMiddleware
```

### 開発版のインストール

```bash
git clone https://github.com/Neurobionics/NeuroLocoMiddleware.git
cd NeuroLocoMiddleware
pip install -e .
```

### 依存関係

- Python 3.6+
- NumPy
- その他のロボティクス関連ライブラリ

## 利点と特徴

### 利点

1. **リアルタイム性の確保**
   - 指定した周期での安定したループ実行
   - タイミングずれの自動補正

2. **使いやすいAPI**
   - シンプルなインターフェース
   - 統計情報の自動レポート

3. **システム同定支援**
   - チャープ信号による周波数応答解析
   - 制御性能の評価

4. **フェード制御**
   - 滑らかな起動と停止
   - 衝撃の軽減

### 技術的特徴

- **ソフトリアルタイム**: 厳密なリアルタイムではないが、実用的
- **クロスプラットフォーム**: Windows, macOS, Linux で動作
- **軽量**: 依存関係が最小限

## 代替手段

NeuroLocoMiddleware は TMotorCANControl の**オプション依存**であり、必須ではありません。以下のように代替実装が可能です：

### SoftRealtimeLoop の代替

```python
import time

def simple_realtime_loop(duration, dt=0.01, report=True):
    start_time = time.time()
    loop_count = 0
    total_loops = int(duration / dt)

    for i in range(total_loops):
        loop_start = time.time()

        # 制御コードをここに記述
        motor.update()
        motor.set_output_angle_radians(target)

        # タイミング調整
        elapsed = time.time() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

        loop_count += 1
        if report and loop_count % 100 == 0:
            current_time = time.time() - start_time
            print(f"経過時間: {current_time:.1f}秒, ループ: {loop_count}")

# 使用例
simple_realtime_loop(duration=10.0, dt=0.01)
```

### Chirp の代替

```python
import numpy as np

def generate_chirp_signal(t, f0, f1, T, amplitude=1.0):
    """
    チャープ信号を生成

    Parameters:
    t : float - 時間
    f0 : float - 開始周波数 [Hz]
    f1 : float - 終了周波数 [Hz]
    T : float - 持続時間 [秒]
    amplitude : float - 振幅
    """
    if t > T:
        return 0.0

    # 線形チャープ
    k = (f1 - f0) / T
    phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
    return amplitude * np.sin(phase)

# 使用例
time_points = np.linspace(0, 1, 1000)
signals = [generate_chirp_signal(t, 25, 250, 1) for t in time_points]
```

## トラブルシューティング

### よくある問題

1. **タイミングが安定しない**
   - OSの優先度設定を確認
   - 他のプロセスによるCPU負荷を軽減

2. **メモリ使用量が多い**
   - 長時間のログ記録を避ける
   - report=False に設定

3. **チャープ信号が歪む**
   - サンプリングレートを上げる
   - dtパラメータを小さくする

### パフォーマンス最適化

```python
# 高性能設定
loop = SoftRealtimeLoop(
    dt=0.001,      # 1kHz
    report=False,  # レポート無効化
    fade=0.0       # フェード無効化
)
```

## 参考文献

1. [NeuroLocoMiddleware PyPI](https://pypi.org/project/NeuroLocoMiddleware/)
2. [TMotorCANControl ドキュメント](https://tmotorcancontrol.readthedocs.io/)
3. [Neurobionics Lab](https://neurobionics.github.io/)

## 更新履歴

- **2024-12**: 初回作成
- **2026-04**: TMotorCANControl 統合版更新

---

このドキュメントは NeuroLocoMiddleware の TMotorCANControl プロジェクトでの使用を前提に記述されています。詳細なAPI仕様については公式ドキュメントを参照してください。