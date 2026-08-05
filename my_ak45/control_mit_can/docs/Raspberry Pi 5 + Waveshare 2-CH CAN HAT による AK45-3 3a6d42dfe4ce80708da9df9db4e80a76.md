# Raspberry Pi 5 + Waveshare 2-CH CAN HAT による AK45-36 モータ制御・環境構築仕様書

**作成日:** 2026年2月14日

**対象ハードウェア:**

- Raspberry Pi 5 (8GB)
- Waveshare 2-CH CAN HAT
- T-MOTOR (CubeMars) AK45-36 KV80 アクチュエータ

**OS / 環境:** Raspberry Pi OS (Bookworm) 64-bit / Python (`uv` 仮想環境)

## 0. このページの位置づけ

本ページは、Raspberry Pi 5 + Waveshare 2-CH CAN HAT + CubeMars AK45-36 を用いたモータ制御システムの**仕様・配線・安全運用・通信仕様**を整理する。

実際の環境構築作業ログ、インストール手順、設定ファイル編集の詳細は以下を参照する。

[作業ログ：Raspberry Pi 5 + Waveshare 2-CH CAN HAT 環境構築記録](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-306d42dfe4ce80fbbdfdeef302bb141c?pvs=21)

MIT制御の理論、`TMotorCANControl` のAPI、実装テンプレート、ゲイン調整メモは以下を参照する。

[技術作業記録：CubeMars AK45-36におけるMIT制御の実装と検証](https://app.notion.com/p/CubeMars-AK45-36-MIT-3a6d42dfe4ce801abbdfdae33c4e8aa2?pvs=21)

## 1. システム概要

### 1.1 目的

Raspberry Pi 5 から Waveshare 2-CH CAN HAT を介して、CubeMars AK45-36 を CAN 通信で制御する。

本システムでは、主に以下を扱う。

- AK45-36 との CAN 通信
- MITモードによる位置・速度・トルク指令
- 2系統CANバスによる左右脚モータ群の分離
- 24Vモータ電源の安全運用
- 実験時の非常停止・電源投入/遮断シーケンス

### 1.2 ハードウェア構成

- **上位コンピュータ:** Raspberry Pi 5
- **CANインターフェース:** Waveshare 2-CH CAN HAT
- **アクチュエータ:** CubeMars / T-MOTOR AK45-36
- **電源:** 24V 直流安定化電源
- **通信:** CAN 1Mbps
- **制御方式:** MIT CAN mode

## 2. CANバス構成と終端抵抗

### 2.1 ネットワークトポロジー

- **バス構造:** バス型（デイジーチェーン / 数珠繋ぎ）
- **インターフェース配分（推奨）**
    - `can0`: 右側脚モータ群
    - `can1`: 左側脚モータ群

### 2.2 終端抵抗

CANバスの両端に **120Ω** の終端抵抗を配置する。

- HAT側に120Ω
- 最終端モータ側に120Ω
- AK45-36内部には切り替え可能な終端抵抗が存在しないため、末端の接続コネクタで `CAN_H` と `CAN_L` の間に 120Ω 抵抗を外付けする
- 電源OFF時に `CAN_H` - `CAN_L` 間をテスターで測定し、合成抵抗が **約60Ω** であることを確認する

## 3. 電源・非常停止・安全運用仕様

### 3.1 直流安定化電源の設定

- **設定電圧:** 24.0 V
- **電流制限**
    - 初期疎通・無負荷テスト時: 1.0 A 〜 2.0 A
    - 実駆動時: 10.0 A 以上
- **実測待機電力:** 約 0.06 A / 1.4 W（ロジック基板正常動作時の目安）

### 3.2 突入電流・活線挿抜の防止

1. 電源の Output ボタンが **OFF** の状態で 24.0V を設定する。
2. 電源線の極性（赤:+ / 黒:-）を確認して接続する。
3. 電源の Output ボタンを **ON** にして通電する。
4. Raspberry Pi 起動後、CANインターフェースが有効であることを確認する。
5. 制御プログラムを実行する。

<aside>
⚠️

物理クリップでの活線挿抜は基板破壊の原因となるため禁止。

</aside>

### 3.3 電源投入・遮断順序

**電源投入順序**

1. Raspberry Pi 起動
2. CANインターフェースUP確認
3. モータ電源ON
4. 制御プログラム実行

**電源遮断順序**

1. 制御プログラム停止
2. モータ電源OFF
3. Raspberry Pi シャットダウン

### 3.4 非常停止

- モータの24V電源ラインに物理的な非常停止スイッチを配置する
- プログラム暴走時はソフト操作に頼らず、物理スイッチで直接遮断する
- 実験前に非常停止スイッチの位置と動作を確認する

## 4. Raspberry Pi側の前提設定

詳細な導入手順は作業ログページに集約する。ここでは、制御システムとして必要な前提のみを整理する。

### 4.1 必要な前提

- Raspberry Pi OS (Bookworm) 64-bit
- Waveshare 2-CH CAN HAT が認識されていること
- `can0` / `can1` が起動できること
- `can-utils` による送受信確認が済んでいること
- Python 仮想環境（`uv`）が使用可能であること
- `TMotorCANControl` 実行環境が構築済みであること

### 4.2 CANインターフェース起動設定

CANインターフェースは 1Mbps で起動する。

```
ip link set can0 up type can bitrate 1000000
ip link set can1 up type can bitrate 1000000
```

リアルタイム制御の遅延を抑える場合、`txqueuelen` は大きくしすぎず、まずは **1000程度** を目安にする。

```
ifconfig can0 txqueuelen 1000
ifconfig can1 txqueuelen 1000
```

## 5. AK45-36 MITモード通信仕様

### 5.1 パッキング（Packing）ロジック

数値（実数）は指定ビット数の無符号整数にマッピングされ、8バイト（64bit）のCANフレームとして送信される。

$$
I = \frac{(x - x_{min}) \cdot (2^b - 1)}{x_{max} - x_{min}}
$$

### 5.2 パラメータ仕様範囲

| **パラメータ** | **記号** | **範囲** | **ビット数 (b)** | **分解能 / 単位** |
| --- | --- | --- | --- | --- |
| **Position** | $P$ | $-12.5$ 〜 $+12.5$ rad | 16 bit ($0 \sim 65535$) | $\approx 0.00038$ rad |
| **Velocity** | $V$ | $-45.0$ 〜 $+45.0$ rad/s | 12 bit ($0 \sim 4095$) | $\approx 0.022$ rad/s |
| **Kp** | $K_p$ | $0.0$ 〜 $500.0$ | 12 bit ($0 \sim 4095$) | - |
| **Kd** | $K_d$ | $0.0$ 〜 $5.0$ | 12 bit ($0 \sim 4095$) | - |
| **Torque** | $\tau$ | $-18.0$ 〜 $+18.0$ Nm | 12 bit ($0 \sim 4095$) | $\approx 0.0088$ Nm |

<aside>
ℹ️

実装時に使用するライブラリ側の定義値と、モータ個体・ファームウェア側の仕様値が一致しているかを確認すること。

</aside>

## 6. CAN 8バイトデータフレーム構成

```
Byte 0: Position [15:8]
Byte 1: Position [7:0]
Byte 2: Velocity [11:4]
Byte 3: Velocity [3:0] | Kp [11:8]
Byte 4: Kp [7:0]
Byte 5: Kd [11:4]
Byte 6: Kd [3:0] | Torque [11:8]
Byte 7: Torque [7:0]
```

## 7. モータ制御コマンド

MITモードでは、以下の特殊コマンドを使用する。**（なにこれ知らん…）**

- **Enter MIT Mode:** `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC`
- **Exit MIT Mode:** `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD`
- **Zero Position 設定:** `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE`

## 8. USB-TTL による個体設定

モータID変更やモード切り替えをPC直接接続で行う場合は、USB-TTL変換器を使用する。

- **通信方式:** 3.3V TTLレベル
- **注意:** RS-232C直結は不可
- **ソフトウェア:** T-Motor Assistant

### 8.1 配線

- `モータ TX` ↔ `USBシリアル変換器 RX`（クロス接続）
- `モータ RX` ↔ `USBシリアル変換器 TX`（クロス接続）
- `モータ GND` ↔ `USBシリアル変換器 GND`

## 9. トラブルシューティング

### Q1. モータから応答がない / 通信エラーが発生する

**チェック1:** 電源OFF時に `CAN_H` - `CAN_L` 間の抵抗を測定する。

- 約60Ω → 終端抵抗はおおむね正常
- 約120Ω → 片側の終端抵抗が不足している可能性
- 無限大 / 極端に大きい → 配線断線の可能性
- 極端に小さい → 短絡の可能性

**チェック2:** CANバスの状態を確認する。

```
ip -d -s link show can0
```

`BUS-OFF` や `ERROR-COUNTER` の増加がある場合は、以下を確認する。

- ボーレート不一致
- CAN_H / CAN_L の逆接続
- 終端抵抗不足
- GND接続不良
- ノイズ
- 電源電圧不足

必要に応じてインターフェースを再起動する。

```
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

### Q2. `sudo` で Python スクリプトを実行すると `ModuleNotFoundError` になる

`sudo` 実行時に `uv` の仮想環境パスが引き継がれないことが原因。

仮想環境内の Python インタープリタを直接フルパス指定して実行する。

```
sudo .venv/bin/python main.py
```

## 10. 関連ページ

- 
    
    [作業ログ：Raspberry Pi 5 + Waveshare 2-CH CAN HAT 環境構築記録](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-306d42dfe4ce80fbbdfdeef302bb141c?pvs=21)
    
- 
    
    [技術作業記録：CubeMars AK45-36におけるMIT制御の実装と検証](https://app.notion.com/p/CubeMars-AK45-36-MIT-3a6d42dfe4ce801abbdfdae33c4e8aa2?pvs=21)