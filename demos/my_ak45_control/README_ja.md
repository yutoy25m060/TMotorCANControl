# My AK45 Control - 開発環境

このディレクトリは、AK45-36 モーターの制御実験を行うための開発環境です。
TMotorCANControl ライブラリを使用して、様々な制御手法を実装・評価します。

## ディレクトリ構造

```
my_ak45_control/
├── 0_template_basic.py      # 基本制御テンプレート
├── 1_template_impedance.py  # インピーダンス制御テンプレート
├── 2_template_current.py    # 電流制御テンプレート
├── config.yaml              # 設定ファイル
├── experiments/             # 実験スクリプト
│   ├── exp_001_gain_tuning.py     # ゲイン調整実験
│   ├── exp_002_step_response.py   # ステップ応答実験
│   ├── exp_003_multi_motor.py     # 多モーター制御実験
│   └── exp_004_trajectory.py      # 軌跡追従実験
├── logs/                    # 実験ログ
│   └── README.md           # ログ分析ガイド
└── README_ja.md            # このファイル
```

## セットアップ

### 1. 依存関係のインストール

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
```

## 制御モード

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

### 2. 電流制御

モーターに直接電流を指令します。

**パラメータ:**
- `Kp`: 比例ゲイン
- `Ki`: 積分ゲイン

**使用例:**
```python
motor.set_current_gains(Kp=0.1, Ki=0.01)
motor.set_output_torque_newton_meters(desired_torque)
```

### 3. 速度制御

モーターの速度を制御します。

**使用例:**
```python
motor.set_speed_radians_per_second(desired_speed)
```

## 安全上の注意

1. **温度監視**: MOSFET 温度が 50℃ を超えないよう監視してください
2. **位置制限**: モーターの可動範囲を超えないよう制限を設定してください
3. **緊急停止**: 異常時はすぐに電源を切断してください
4. **負荷確認**: モーターに過大な負荷をかけないよう注意してください

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