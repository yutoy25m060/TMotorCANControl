# My AK45 Control - 開発環境

このディレクトリは、AK45-36 モーターの制御実験を行うための開発環境です。
TMotorCANControl ライブラリを使用して、様々な制御手法を実装・評価します。

## ディレクトリ構造

```
my_ak45_control/
├── 0_template_basic.py      # 基本制御テンプレート
├── 1_template_impedance.py  # インピーダンス制御テンプレート
├── 2_template_current.py    # 電流制御テンプレート
├── lib/                     # テンプレート・実験スクリプトの共通コード
│   ├── config_loader.py    # config.yaml の読み込み
│   ├── motor_setup.py      # モーター初期化・ゼロ化（単一/複数モーター）
│   ├── logging_utils.py    # ログファイル命名・制御ループ生成
│   ├── sync_logger.py      # 複数モーターの同期ロギング（SyncMultiMotorLogger）
│   └── safety_monitor.py   # 複数モーターの安全監視・緊急停止（SafetyMonitor）
├── config.yaml              # 設定ファイル
├── experiments/             # 実験スクリプト
│   ├── exp_001_gain_tuning.py     # ゲイン調整実験
│   ├── exp_002_step_response.py   # ステップ応答実験
│   ├── exp_003_multi_motor.py     # 多モーター制御実験
│   ├── exp_004_trajectory.py      # 軌跡追従実験
│   └── exp_005_sysid_excitation.py # システム同定用 multi-sine 励振実験
├── logs/                    # 実験ログ
│   └── README.md           # ログ分析ガイド
└── README_ja.md            # このファイル
```
### テンプレート
- `0_template_basic.py`: with ブロック + update() ループの基本骨組み
- 各実験は exp_NN_description.py として、テンプレートをコピーして作成

### 設定管理
- `config.yaml` で CAN 設定、モーター ID、ゲイン上限値を一元管理
- 実験スクリプト側は config.yaml を読み込み、パラメータを上書き

## ロギング
- CSV 出力は自動的に logs/ に保存
- ファイル名に タイムスタンプ を含める
- README_ja.md で進捗・実験結果を記録

## 版管理
- logs/*.csv は .gitignore に
- 実験スクリプト、テンプレート、config.yaml は Git 追跡対象

## 運用
1. 新規実験 → テンプレートをコピー
2. パラメータを config.yaml で管理
3. 実行 → ログ自動保存
4. 結果を README_ja.md に記録

## セットアップ

### 1. 依存関係のインストール

必要なライブラリ:
- `TMotorCANControl`  (ローカルパッケージ)
- `PyYAML`          (設定ファイル読み込み)
- `numpy`           (制御計算)
- `python-can>=4.0.0`  (CAN 通信)
- `pyserial>=3.5`      (シリアル通信、TMotorCANControl の依存)
- `NeuroLocoMiddleware` (TMotorCANControl の依存)

任意の解析ライブラリ:
- `pandas`          (ログ分析)
- `matplotlib`      (可視化)

```bash
cd /path/to/TMotorCANControl
pip install -e .
pip install pyyaml numpy
```

### 2. CAN インターフェースの設定

Raspberry Pi で CAN インターフェースを設定：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 3. 設定ファイルの編集

`config.yaml` を編集して、使用するモーターの設定を変更してください：

```yaml
motor:
  type: "AK45-36"
  id: 1
  max_temp: 50
```

## 使用方法

### テンプレートの実行

各テンプレートは独立して実行できます：

```bash
# 基本制御
python 0_template_basic.py

# インピーダンス制御
python 1_template_impedance.py

# 電流制御
python 2_template_current.py
```

### 実験の実行

experiments ディレクトリ内の実験スクリプトを実行：

```bash
cd experiments

# ゲイン調整実験
python exp_001_gain_tuning.py

# ステップ応答実験
python exp_002_step_response.py

# 多モーター制御実験
python exp_003_multi_motor.py

# 軌跡追従実験
python exp_004_trajectory.py

# システム同定用 multi-sine 励振実験
python exp_005_sysid_excitation.py
```

## リアルタイム制御 (NeuroLocoMiddleware 統合)

この開発環境では、安定したリアルタイム制御を実現するために [NeuroLocoMiddleware](https://pypi.org/project/NeuroLocoMiddleware/) の `SoftRealtimeLoop` を使用しています。

### 特徴

- **安定したタイミング**: 自動タイミング補正により、正確な制御周期 (デフォルト: 100Hz) を維持
- **パフォーマンス監視**: 制御ループのタイミング情報をリアルタイムでレポート
- **柔軟な設定**: `config.yaml` で制御周期やレポート設定を調整可能

### 設定パラメータ

```yaml
control:
  realtime:
    dt: 0.01        # 制御周期 [秒] (100Hz)
    report: true     # パフォーマンスレポート有効
    fade: 0          # フェード時間 [秒]
```

### 使用例

```python
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# リアルタイム制御ループの初期化
loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)

# 制御ループ
for t in loop:
    # 制御処理
    motor.update()
    motor.set_output_angle_radians(desired_position)

    # 定期的な情報表示
    if loop.n % 10 == 0:  # 100msごと
        print(f"経過時間: {t:.1f} 秒")

    # 実験時間チェック
    if t >= runtime_seconds:
        break
```

### 利点

1. **タイミングの安定性**: 手動の `time.sleep()` よりも正確な周期制御
2. **パフォーマンス監視**: 制御ループの遅延やジッターを検知
3. **コードの簡潔化**: タイミング管理をミドルウェアに委譲

### 1. インピーダンス制御

モーターをバネ-ダンパー系として制御します。

**パラメータ:**
- `K`: 剛性ゲイン [Nm/rad]
- `B`: 減衰ゲイン [Nm/(rad/s)]

**使用例:**
```python
motor.set_impedance_gains_real_unit(K=10.0, B=0.5)
motor.set_output_angle_radians(desired_angle)
```

### 2. 位置制御

モーターの角度位置を直接制御します。目標位置を指定すると、モーターがその位置に移動します。

**特徴:**
- 位置ベースのフィードバック制御
- 軌跡追従や精密位置決めに適している
- インピーダンス制御と組み合わせ可能

**使用例:**
```python
motor.set_output_angle_radians(desired_position)  # 目標位置 [rad]
```

**実験例:**
- `exp_002_step_response.py`: ステップ応答特性の評価
- `exp_004_trajectory.py`: 軌跡追従制御

### 3. 電流制御

モーターに直接電流を指令します。

**パラメータ:**
- `Kp`: 比例ゲイン
- `Ki`: 積分ゲイン

**使用例:**
```python
motor.set_current_gains(Kp=0.1, Ki=0.01)
motor.set_output_torque_newton_meters(desired_torque)
```

### 4. 速度制御

モーターの速度を制御します。

**使用例:**
```python
motor.set_speed_radians_per_second(desired_speed)
```

### 5. システム同定用励振信号（sysid excitation）

MuJoCo sysid toolbox 用に、純トルク指令（kp=0, kd=0）で multi-sine 励振信号を送ります。
`set_current_gains()` の引数は実際には使われないダミー引数（`mit_can.py` の docstring 参照）で、
呼び出すと電流制御モードに入るだけです。このモードでは位置・速度・Kp・Kd が常に 0 で CAN フレームに
エンコードされるため、位置・速度フィードバックによる復元力を持たない開ループのトルク指令になります
（他の制御モードと異なり、目標位置や目標速度への収束を保証しません）。

**使用例:**
```python
motor.set_current_gains()  # kp/ki/ff/spoof はダミー引数
motor.set_output_torque_newton_meters(desired_torque)  # desired_torque は multi-sine 励振式で計算
```

**実験例:**
- `exp_005_sysid_excitation.py`: multi-sine 励振信号によるログ取得

詳しい励振式・パラメータ選定の考え方は
[`my_ak45/Mujoco/docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md`](../Mujoco/docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md)
を参照してください。

## 安全上の注意

1. **温度監視**: MOSFET 温度が 50℃ を超えないよう監視してください
2. **位置制限**: モーターの可動範囲を超えないよう制限を設定してください
3. **緊急停止**: 異常時はすぐに電源を切断してください
4. **負荷確認**: モーターに過大な負荷をかけないよう注意してください

`exp_003_multi_motor.py` は `config.yaml` の `safety.max_position`/`max_velocity`/`max_torque`/`emergency_stop`
を実際に読み込み、`lib/safety_monitor.py` の `SafetyMonitor` でモーターごとに監視しています。
いずれかのモーターがしきい値を超えると（`emergency_stop: true` の場合）自動的に全モーターへ
`power_off()` を送って停止します。他のテンプレート・実験スクリプトはまだこの監視機構を使っていません。

`exp_005_sysid_excitation.py` も `SafetyMonitor` を使用しています。この実験は位置・速度フィードバック
による復元力を持たない開ループのトルク指令であるため、実測値ベースの `SafetyMonitor` によるしきい値
超過時の緊急停止に加えて、指令トルク自体も `safety.max_torque` でクランプする2層の保護を行っています。
それでも初回実行時は目視監視のもとで行ってください。

## トラブルシューティング

### CAN 接続エラー

```
エラー: CAN 接続に失敗しました
```

**解決方法:**
1. CAN インターフェースが正しく設定されているか確認
2. モーターの電源が投入されているか確認
3. CAN ID が正しいか確認

### モーター応答なし

```
モーターが応答しません
```

**解決方法:**
1. モーターの CAN ID を確認
2. ケーブルの接続を確認
3. モーターの電源を確認

### 温度異常

```
MOSFET 温度が高すぎます
```

**解決方法:**
1. モーターの負荷を軽減
2. 冷却を改善
3. 制御パラメータを調整

## 実験結果の分析

実験ログは `logs/` ディレクトリに CSV 形式で保存されます。
詳細な分析方法は `logs/README.md` を参照してください。

## 拡張方法

### 新しい実験の追加

1. `experiments/` ディレクトリに新しいスクリプトを作成
2. `config.yaml` に必要なパラメータを追加
3. ログ変数を適切に設定

### 新しい制御モードの実装

1. テンプレートファイルをコピー
2. TMotorManager_mit_can のメソッドを適切に使用
3. 安全チェックを追加

## 参考資料

- [TMotorCANControl ドキュメント](https://github.com/YutoUchimi/TMotorCANControl)
- [AK45-36 データシート](https://tmotor.com/product/ak-series/)
- [MIT CAN プロトコル仕様](https://github.com/YutoUchimi/TMotorCANControl/blob/main/docs/source/mit_can.rst)

## 連絡先

質問や問題がある場合は、以下の方法で連絡してください：

- GitHub Issues: [TMotorCANControl リポジトリ](https://github.com/YutoUchimi/TMotorCANControl/issues)
- メール: your-email@example.com

---

最終更新: 2024年12月