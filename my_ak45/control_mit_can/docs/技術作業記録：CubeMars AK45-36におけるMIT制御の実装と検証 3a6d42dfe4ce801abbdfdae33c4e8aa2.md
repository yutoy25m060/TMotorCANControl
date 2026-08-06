# 技術作業記録：CubeMars AK45-36におけるMIT制御の実装と検証

会話時期：2026年4月7日

## 0. このページの位置づけ

本ページは、CubeMars AK45-36 を **MIT CAN mode** で制御する際の、理論理解・API使用方法・実装テンプレート・検証時の注意点を整理する。

ハードウェア構成、配線、電源、CAN通信仕様、安全運用の詳細は以下の仕様書を参照する。

[Raspberry Pi 5 + Waveshare 2-CH CAN HAT による AK45-36 モータ制御・環境構築仕様書](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-AK45-36-3a6d42dfe4ce80708da9df9db4e80a76?pvs=21)

Raspberry Pi 5 と Waveshare 2-CH CAN HAT の実際の環境構築作業ログは以下を参照する。

[作業ログ：Raspberry Pi 5 + Waveshare 2-CH CAN HAT 環境構築記録](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-306d42dfe4ce80fbbdfdeef302bb141c?pvs=21)

## 1. MIT制御の目的

MIT制御では、モータを単なる位置サーボとして扱うのではなく、関節に仮想的な**バネ・ダンパ・外力補償**を与えるように制御する。

主な目的は以下。

- 関節に「硬さ」や「柔らかさ」を持たせる
- 外力に対してしなやかに反応させる
- 位置制御だけでなく、速度・トルク・フィードフォワードを組み合わせる
- 歩行ロボットや人間協調ロボットのような、接触を含む運動に適した制御を行う

## 2. MIT制御の基本式

モーターコントローラ内部で計算される基本制御式は以下。

$$
\tau = \underbrace{K_p(q_{des} - q) + K_d(\dot{q}_{des} - \dot{q})}_{\text{インピーダンス制御（PD項）}} + \underbrace{\tau_{ff}}_{\text{フィードフォワード項}}
$$

ここで、

- $q_{des}$: 目標位置
- $q$: 現在位置
- $\dot{q}_{des}$: 目標速度
- $\dot{q}$: 現在速度
- $K_p$: 位置ゲイン / 剛性
- $K_d$: 速度ゲイン / 減衰
- $\tau_{ff}$: フィードフォワードトルク
- $\tau$: 出力トルク

## 3. 各パラメータの意味

### 3.1 $K_p$：剛性 / Stiffness

$K_p$ は、目標位置からのずれに対してどの程度強く戻ろうとするかを決める。

- 大きいほど硬い関節になる
- 小さいほど柔らかい関節になる
- 上げすぎると発振や急激な動作につながる
- 重力補償や摩擦補償なしに $K_p$ だけを上げると、危険な挙動になりやすい

### 3.2 $K_d$：減衰 / Damping

$K_d$ は、関節の動きに対してブレーキをかける項。

- 振動を抑制する
- 応答を落ち着かせる
- 大きすぎると動作が重くなる
- 小さすぎると振動しやすい

### 3.3 $\tau_{ff}$：フィードフォワードトルク

$\tau_{ff}$ は、位置誤差の発生を待たずに直接加えるトルク。

用途例：

- 重力補償
- 摩擦補償
- 慣性補償
- 事前に必要と分かっている外力の補償

### 3.4 q軸電流とトルク

`TMotorCANControl` では、q軸電流やトルクが取得できる。

- **`dev.current_qaxis`**: アンペア [A]
- **`dev.torque`**: ニュートンメートル [Nm]
- **`dev.position`**: ラジアン [rad]
- **`dev.velocity`**: ラジアン毎秒 [rad/s]

`dev.current_qaxis` は、内部で `Current_Factor=0.59`、トルク定数、減速比を用いて算出される。

## 4. MIT制御とPID制御の違い

| **項目** | **PID制御** | **MIT制御 (PD + FF)** |
| --- | --- | --- |
| **主な目的** | 位置誤差の徹底排除（高剛性） | 物理的特性（バネ・ダンパ）の付与 |
| **定常偏差の解消** | 積分項 $K_i$ の蓄積による | フィードフォワード $\tau_{ff}$ のモデル計算による |
| **応答性** | 誤差が発生してから反応する | 必要なトルクを先回りして出力できる |
| **環境との接触** | 高剛性になりやすく、外力衝撃に弱い場合がある | 外力をいなす柔軟な動作が可能 |
| **用途** | 産業用ロボット、工作機械、ドローンなど | 歩行ロボットの脚、人間協調ロボットなど |

## 5. 制御モードとパラメータ設定

MIT制御では、$K_p$、$K_d$、$\tau_{ff}$ の組み合わせによって、複数の制御モードのように扱える。

- **位置（インピーダンス）制御**
    - $K_p > 0$
    - $K_d > 0$
    - $\tau_{ff} = 0$
- **速度制御**
    - $K_p = 0$
    - $K_d > 0$
    - $\tau_{ff} = 0$
- **トルク（電流）制御**
    - $K_p = 0$
    - $K_d = 0$
    - $\tau_{ff} > 0$
- **フル状態フィードバック**
    - $K_p > 0$
    - $K_d > 0$
    - $\tau_{ff} > 0$

## 6. TMotorCANControl APIメモ

### 6.1 初期化

AK45-36、CAN ID 1 の例。

```
from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can

with TMotorManager_mit_can(motor_type='AK45-36', motor_ID=1) as dev:
    ...
```

### 6.2 ゼロ点設定

```
dev.set_zero_position()
```

現在位置をゼロ点として設定する。実験前に、機構が安全な姿勢にあることを確認してから実行する。

### 6.3 インピーダンスゲイン設定

```
dev.set_impedance_gains_real_unit(K=10.0, B=0.5)
```

- `K`: 剛性に対応
- `B`: 減衰に対応

### 6.4 CAN送受信更新

```
dev.update()
```

制御ループ内で周期的に呼び出す。呼び出し周期が不安定だと、制御応答やログの解釈に影響する。

## 7. 基本コードテンプレート

```
import time
from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can

# AK45-36, CAN ID 1 で初期化
with TMotorManager_mit_can(motor_type='AK45-36', motor_ID=1) as dev:
    dev.set_zero_position()  # 現在地をゼロ点に設定
    time.sleep(1.5)

    # バネ感(K)と減衰感(B)を設定
    dev.set_impedance_gains_real_unit(K=10.0, B=0.5)

    while True:
        dev.update()  # CAN送受信の実行

        # 例：1.0 rad の目標位置へ柔らかく保持しつつ、重力補償用に 0.5 Nm のFFを加える
        dev.position = 1.0
        dev.torque = 0.5
```

## 8. ゲイン調整方針

### 8.1 基本方針

1. 低い $K_p$ から始める
    - 最初から高剛性にしない。
2. $K_d$ で振動を抑える
    - 目標位置付近で振動する場合は、減衰を調整する。
3. 必要に応じて $\tau_{ff}$ を加える
    - 重力や静摩擦の影響で目標位置に届かない場合、$K_p$ を上げる前に補償トルクを検討する。
4. **発振・異音・急加速が出たら即停止**
    - ゲインを下げ、配線・電源・CAN状態を確認する。

### 8.2 調整時の注意

- $K_p$ を上げすぎると発振する
- $K_p$ を下げすぎると目標位置まで届かない
- $K_d$ が小さすぎると振動しやすい
- $K_d$ が大きすぎると動作が鈍くなる
- $\tau_{ff}$ を入れすぎると、意図しない方向へ動く可能性がある

## 9. 実験・検証時の注意

### 9.1 通信周期

Raspberry Pi 5側の制御ループ速度を確保するため、`NeuroLocoMiddleware.SoftRealtimeLoop` などを用いて、周期的・安定的に `dev.update()` を呼ぶ。

確認したい項目：

- 制御周期
- 周期のばらつき
- CAN送受信エラー
- `BUS-OFF` の有無
- モータ応答の遅れ
- ログ取得周期

### 9.2 安全確認

実験前に以下を確認する。

- モータ固定治具が十分に強い
- 可動範囲内に手や工具がない
- 24V電源の電流制限が適切
- 非常停止スイッチにすぐ手が届く
- `Zero Position` 実行前の姿勢が安全
- CAN配線と電源配線が緩んでいない

## 10. 今後の課題

- 制御周期の実測と安定化
- `K_p`, `K_d`, $\tau_{ff}$ の調整記録テンプレート作成
- 無負荷状態での応答確認
- 負荷あり状態での重力補償テスト
- 関節機構に取り付けた状態での安全な可動範囲設定
- ROS2 / MuJoCo / SysID への接続を見据えたログ形式の整理

## 11. 関連ページ

- 
    
    [Raspberry Pi 5 + Waveshare 2-CH CAN HAT による AK45-36 モータ制御・環境構築仕様書](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-AK45-36-3a6d42dfe4ce80708da9df9db4e80a76?pvs=21)
    
- 
    
    [作業ログ：Raspberry Pi 5 + Waveshare 2-CH CAN HAT 環境構築記録](https://app.notion.com/p/Raspberry-Pi-5-Waveshare-2-CH-CAN-HAT-306d42dfe4ce80fbbdfdeef302bb141c?pvs=21)