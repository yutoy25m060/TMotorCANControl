# TMotor AK シリーズ MIT CAN 制御ガイド

## 概要

`mit_can.py` は、CAN バス経由で TMotor AK シリーズアクチュエータを MIT 制御モード（インピーダンス制御＋フルステートフィードバック）で制御するための Python ライブラリです。Raspberry Pi 上の SocketCAN ドライバを使用して CAN 通信を行います。

## 主要クラスと役割

### `CAN_Manager`
CAN バスの初期化・管理、CAN メッセージの送受信を担当するシングルトンクラスです。

**主なメソッド：**
- `__new__()` - CAN バスの初期化（シングルトンパターン）
- `add_motor(motor)` - モーターをリスナーとして登録
- `send_MIT_message(motor_id, data)` - CAN メッセージを送信
- `MIT_controller(motor_id, motor_type, position, velocity, Kp, Kd, I)` - MIT 制御信号を送信
- `power_on(motor_id)` - パワーオンコマンド送信
- `power_off(motor_id)` - パワーオフコマンド送信
- `zero(motor_id)` - 位置ゼロ化コマンド送信

### `TMotorManager_mit_can`
ユーザーが直接使用するメインクラス。モーターの制御、状態取得、ログ出力を担当します。

**with ブロックのコンテキストマネージャーで使用：**
```python
with TMotorManager_mit_can(motor_type='AK45-36', motor_ID=1) as motor:
    # モーター制御コード
    pass
```

## 制御モード

MIT 制御には 5 つのモードがあります：

### 1. **IDLE（アイドル）**
すべてのコマンドが 0 に設定されます。モーターを待機状態にします。

### 2. **IMPEDANCE（インピーダンス）**
位置ゲイン（Kp）と速度ゲイン（Kd）のみが有効です。
```python
motor.set_impedance_gains_real_unit(K=10, B=0.5)  # K: 剛性 [Nm/rad], B: 減衰 [Nm/(rad/s)]
motor.set_output_angle_radians(1.57)  # 目標位置を設定
```

### 3. **CURRENT（電流制御）**
電流（q 軸電流）のみが有効です。位置・速度ゲインは送信されません。
```python
motor.set_current_gains()
motor.set_motor_current_qaxis_amps(5.0)  # 5A を指令
```

### 4. **FULL_STATE（フルステートフィードバック）**
位置ゲイン、速度ゲイン、フィードフォワード電流がすべて有効です。
```python
motor.set_impedance_gains_real_unit_full_state_feedback(K=10, B=0.5)
motor.set_output_angle_radians(1.57)
motor.set_motor_current_qaxis_amps(2.0)  # 追加の電流 2A
```

### 5. **SPEED（速度制御）**
速度ゲイン（Kd）と速度コマンドのみが有効です。
```python
motor.set_speed_gains(kd=1.0)
motor.set_output_velocity_radians_per_second(3.14)  # pi rad/s で回転
```

## 基本的な使用方法

### セットアップ

```python
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# モーターマネージャーを作成（AK45-36, CAN ID=1）
with TMotorManager_mit_can(motor_type='AK45-36', motor_ID=1, CSV_file='log.csv') as motor:
    # モーターが自動的にパワーオンされる
    
    # 位置をゼロ化（約 1 秒待つ必要がある）
    motor.set_zero_position()
    time.sleep(1.5)
    
    # インピーダンス制御を設定
    motor.set_impedance_gains_real_unit(K=10, B=0.5)
    
    # 制御ループ
    for i in range(100):
        motor.update()  # 状態更新 + コマンド送信
        motor.set_output_angle_radians(3.14 * (i / 100.0))  # 段階的に回転
        time.sleep(0.01)

# モーターが自動的にパワーオフされる
```

## 状態取得

モーターの最新状態は `update()` の直後に取得可能です：

```python
motor.update()

# 位置、速度、加速度の取得（出力側：ギアボックス後）
pos = motor.get_output_angle_radians()          # [rad]
vel = motor.get_output_velocity_radians_per_second()  # [rad/s]
acc = motor.get_output_acceleration_radians_per_second_squared()  # [rad/s²]

# 電流・トルクの取得
current = motor.get_current_qaxis_amps()        # [A]
torque = motor.get_output_torque_newton_meters()  # [Nm]

# モーター側の値（ギアボックス前）
motor_pos = motor.get_motor_angle_radians()
motor_vel = motor.get_motor_velocity_radians_per_second()
motor_torque = motor.get_motor_torque_newton_meters()

# 温度・エラーコード
temp = motor.get_temperature_celsius()          # [℃]
error = motor.get_motor_error_code()            # 0 = 正常
```

## パラメータの理解

### `MIT_Params` 辞書

各モーターの制御範囲とゲインの制限を定義します。AK45-36 の例：

```python
"AK45-36": {
    "P_min": -12.5,         # 位置コマンド最小値 [rad]
    "P_max": 12.5,          # 位置コマンド最大値 [rad]
    "V_min": -30.0,         # 速度コマンド最小値 [rad/s]
    "V_max": 30.0,          # 速度コマンド最大値 [rad/s]
    "T_min": -32.0,         # 電流コマンド最小値（トルク相当） [A]
    "T_max": 32.0,          # 電流コマンド最大値（トルク相当） [A]
    "Kp_min": 0.0,          # 位置ゲイン最小値 [Nm/rad]
    "Kp_max": 500.0,        # 位置ゲイン最大値 [Nm/rad]
    "Kd_min": 0.0,          # 速度ゲイン最小値 [Nm/(rad/s)]
    "Kd_max": 5.0,          # 速度ゲイン最大値 [Nm/(rad/s)]
    "Kt_TMotor": 0.1206,    # トルク定数 [Nm/A]
    "GEAR_RATIO": 36.0,     # 減速比
}
```

> ⚠️ この値は `mit_can.py` の `MIT_Params` から転記したものであり、実行時に実際に使われる値はこのコード自体（唯一の情報源）。
> `my_ak45/control_mit_can/docs/` 配下のNotionエクスポート仕様書や `demos/mit_can/demo_full_state_feedback_mit_can.py` には
> `V_max`/`T_max` について異なる数値が残っており、まだ突合できていない。詳細は `mit_can.py` 側のコメントと
> `.ai/logs/2026-08-05_01_ak45-36-spec-inconsistency-flags_01.md` を参照。
>
> 追記(2026-08-05): `docs_mit_can/ak45-36-firmware-and-parameters/` に実機R-Linkエクスポートの生ファームウェア設定が
> 追加され、`GEAR_RATIO=36.0`・`T_max=32.0`（A）は実機値との突合が取れた一方、`Kt_TMotor`/`Kt_actual=0.1206` は
> firmwareの `foc_current_kp`（電流制御ループの比例ゲイン、トルク定数とは物理的に別物）そのものだったと判明した。
> `V_max=30.0` の算出根拠は依然不明。詳細は `.ai/logs/2026-08-05_03_ak45-36-firmware-export-crosscheck_01.md` を参照。
>
> 追記(2026-08-05): `docs_mit_can/公式基本仕様.png`（CubeMars公式基本仕様表）で `GEAR_RATIO=36.0` が確定し、
> `Kt_TMotor=0.1206` も公式値0.11 Nm/Aの1割以内と判明した。一方 `V_max=30.0`・`T_max=32.0` は、公式の無負荷
> 回転速度(約5.45 rad/s、出力軸側)・ピーク電流(6.5A)を大きく上回っており、MITプロトコルのエンコード範囲ではあっても
> 実運用で常用してよい値ではないと考えられる。詳細は `.ai/logs/2026-08-05_04_official-datasheet-crosscheck_01.md` を参照。

## CAN メッセージフォーマット

MIT CAN プロトコルでは、8 バイトのメッセージで以下の情報を送受信します：

### 送信フォーマット（コントローラー → モーター）
```
[位置(16bit) | 速度(12bit) | Kp(12bit) | Kd(12bit) | 電流(12bit)]
```

### 受信フォーマット（モーター → コントローラー）
```
[位置(16bit) | 速度(12bit) | 電流(12bit) | 温度(8bit) | エラー(8bit)]
```

値は浮動小数点数から整数にスケーリングして送受信されます。

## 内部状態管理

### ラップアラウンド処理

モーター位置・速度・電流はハードウエアで制限値に達すると反転（ラップアラウンド）します。このライブラリは以下の手法でトラッキングします：

- **`_times_past_position_limit`** - 位置がレンジを超えた回数
- **`_times_past_velocity_limit`** - 速度がレンジを超えた回数
- **実拡張状態** - `_motor_state.position` と `_motor_state.velocity` は 1.5 倍以上の範囲で追跡可能

### 非同期状態更新

CAN メッセージの受信は `motorListener` クラスで非同期に処理されます。`_update_state_async()` は受信するたびに呼ばれ、`_motor_state_async` が更新されます。`update()` 呼び出し時に、この非同期状態が同期状態（`_motor_state`）に反映されます。

## ロギング

CSV ファイルへの自動ロギングが可能です：

```python
with TMotorManager_mit_can(
    motor_type='AK45-36',
    motor_ID=1,
    CSV_file='motor_log.csv',
    log_vars=[
        'output_angle',
        'output_velocity',
        'output_acceleration',
        'current',
        'output_torque'
    ]
) as motor:
    # ログ出力対象
    pass
```

## エラーハンドリング

### エラーコード一覧

| コード | 意味 |
|--------|------|
| 0 | エラーなし |
| 1 | 過熱 |
| 2 | 過電流 |
| 3 | 過電圧 |
| 4 | 低電圧 |
| 5 | エンコーダ故障 |
| 6 | フェーズ電流不均衡（ハードウェア損傷の可能性） |

### よくある問題

**接続失敗**
```python
if not motor.check_can_connection():
    print("CAN バス接続に失敗しました。接続を確認してください。")
```

**温度上限超過**
デフォルトでは 80℃でエラーが発生します。`__init__()` の `max_mosfett_temp` パラメータで変更可能です。

## Raspberry Pi での セットアップ例

```bash
# CAN ドライバのロード（既にある場合はスキップ）
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x

# CAN インターフェースを起動（500kbps）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Python パッケージをインストール
python -m pip install python-can pyserial numpy
python -m pip install -e /path/to/TMotorCANControl
```

## 追加リソース

- [AK-series 公式マニュアル](https://store.cubemars.com/)
- MIT CAN プロトコル仕様（マニュアルに含まれる）
- デモスクリプト：`demos/mit_can/`
