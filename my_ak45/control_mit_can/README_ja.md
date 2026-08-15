# My AK45 Control - 開発環境

このディレクトリは、AK45-36 モーターの制御実験を行うための開発環境です。
TMotorCANControl ライブラリを使用して、様々な制御手法を実装・評価します。

## ディレクトリ構造

```
my_ak45_control/
├── 0_template_basic.py      # 基本制御テンプレート
├── 1_template_impedance.py  # インピーダンス制御テンプレート
├── 2_template_current.py    # 電流制御テンプレート
├── 3_template_speed.py      # 速度制御テンプレート
├── dashboard_demo.py        # リアルタイムWebダッシュボード配信テンプレート
├── lib/                     # テンプレート・実験スクリプトの共通コード
│   ├── config_loader.py    # config.yaml の読み込み
│   ├── motor_setup.py      # モーター初期化・ゼロ化（単一/複数モーター）
│   ├── logging_utils.py    # ログファイル命名・制御ループ生成
│   ├── sync_logger.py      # 複数モーターの同期ロギング（SyncMultiMotorLogger）
│   ├── safety_monitor.py   # 複数モーターの安全監視・緊急停止（SafetyMonitor）
│   └── dashboard_server.py # ブラウザ向けリアルタイムダッシュボード配信（DashboardServer）
├── config.yaml              # 設定ファイル
├── experiments/             # 実験スクリプト
│   ├── exp_001_gain_tuning.py             # ゲイン調整実験
│   ├── exp_002_step_response.py           # ステップ応答実験
│   ├── exp_003_multi_motor.py             # 多モーター制御実験
│   ├── exp_004_trajectory.py              # 軌跡追従実験
│   ├── exp_006_thermal_baseline_check.py  # 温度ベースライン確認（単一モーター、能動指令なし）
│   └── exp_007_thermal_baseline_multi.py  # 温度ベースライン確認（3台並列、能動指令なし）
├── logs/                    # 実験ログ（実行ごとにサブフォルダが作られる）
│   ├── README.md           # ログ分析ガイド
│   └── {実験名}_{タイムスタンプ}/  # 1回の実行につき1フォルダ（CSV + コンソールログ）
└── README_ja.md            # このファイル
```
### テンプレート
- `0_template_basic.py`: with ブロック + update() ループの基本骨組み
- 各実験は exp_NN_description.py として、テンプレートをコピーして作成

### 設定管理
- `config.yaml` で CAN 設定、モーター ID、ゲイン上限値を一元管理
- 実験スクリプト側は config.yaml を読み込み、パラメータを上書き

## ロギング
- スクリプト実行のたびに `logs/{実験名}_{タイムスタンプ}/` フォルダが自動作成され、
  その実行のCSV・コンソールログはすべてこのフォルダの下に保存される
  （`lib/logging_utils.py` の `make_run_dir()`）
- CSV: 従来どおり `TMotorManager_mit_can`/`SyncMultiMotorLogger` が記録（内容は変更なし）
- コンソールログ: 実行中にターミナルへ表示された内容（進捗表示・警告・未捕捉の例外の
  トレースバックを含む）を `console.log` として複製記録する（`lib/logging_utils.py` の
  `console_log`）。ターミナルへの表示自体はそのまま行われる
- README_ja.md で進捗・実験結果を記録

## 版管理
- logs/ 以下の `*.csv`/`*.log`（console.log を含む）は .gitignore に
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

（リアルタイムWebダッシュボード機能は Python 標準ライブラリのみで実装されているため、
上記に加えて追加でインストールするパッケージはありません）

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
  max_temp: 75  # 根拠は config.yaml 冒頭のコメント参照（アイドル時の温度上昇に対する実用マージン）
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

# 速度制御
python 3_template_speed.py

# リアルタイムWebダッシュボード
python dashboard_demo.py
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

# 温度ベースライン確認（単一モーター、能動指令なし）
python exp_006_thermal_baseline_check.py

# 温度ベースライン確認（3台並列、能動指令なし）
python exp_007_thermal_baseline_multi.py
```

システム同定用 multi-sine 励振実験（`exp_005_sysid_excitation.py`）は
[`my_ak45/Mujoco/data_collection/`](../Mujoco/data_collection/) に移動しました
（詳細は下記「MuJoCo sysid との連携」参照）。

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
motor.set_current_gains(kp=0.1, ki=0.01)  # 引数名は小文字 kp/ki（mit_can.py のシグネチャ参照）
motor.set_output_torque_newton_meters(desired_torque)
```

### 4. 速度制御

モーターの速度を制御します（プレーン速度モード。位置ゲイン・フィードフォワード電流は常に0）。

**パラメータ:**
- `kd`: 速度ゲイン（制御則: `(v_des - v_actual)*kd = iq`）

**使用例:**
```python
motor.set_speed_gains(kd=1.0)
motor.set_output_velocity_radians_per_second(desired_speed)  # desired_speed [rad/s]
```

**テンプレート:**
- `3_template_speed.py`: 速度制御の基本骨組み

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
- [`my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py`](../Mujoco/data_collection/exp_005_sysid_excitation.py):
  multi-sine 励振信号によるログ取得。MuJoCo sysid のモデル最適化を別PC（Windows、GPU利用）で行う
  ため、このディレクトリ（`control_mit_can/experiments/`）ではなく `my_ak45/Mujoco/` 配下に置かれて
  いる（本体の `lib/`・`config.yaml` は引き続きここのものを再利用する）。出力データも
  `control_mit_can/logs/` ではなく git 追跡対象の `my_ak45/Mujoco/data/raw/` に直接保存される。

詳しい励振式・パラメータ選定の考え方、Pi/Windows PC間の作業分担は
[`my_ak45/Mujoco/docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md`](../Mujoco/docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md)
と
[`my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`](../Mujoco/docs_syid/AK45-36_sysid_作業手順.md)
を参照してください。

### 6. リアルタイムWebダッシュボード

制御スクリプト実行中のモーター状態（位置・速度・トルク・電流・温度）を、同一LAN上の
別端末のブラウザからリアルタイムに閲覧できます。CLAUDE.md の「ヘッドレスRaspberry Pi/Linux
向け構成のためGUI依存関係を持ち込まない」という方針に従い、標準ライブラリの
`http.server`（`ThreadingHTTPServer` + Server-Sent Events）のみで実装されており、
Flask 等の追加インストールは不要です。ブラウザ側もネイティブの `EventSource` API と
バニラJSのみで、外部ライブラリ・CDNには依存しません。

**アクセス方法:**
1. Piと閲覧する端末を同一LAN上に置く
2. 制御スクリプト側で `DashboardServer` を起動すると、コンソールに `http://<PiのIP>:8000/`
   のようなURLが表示される（IPアドレスの自動検出に失敗した場合は `hostname -I` 等でPi自身の
   IPアドレスを確認する）
3. 別端末のブラウザで、表示されたURLを開く

**API:** `SafetyMonitor`/`SyncMultiMotorLogger` と同じく `motors`/`motor_names` のパラレル
リストを受け取るため、単一モーターは `motors=[motor]` の1要素リストで、複数モーター
（exp_003/007相当）にもそのまま使えます。

**使用例:**
```python
from lib.dashboard_server import DashboardServer

with DashboardServer([motor], ["motor1"], LOG_VARS, port=8000) as dashboard:
    print(f"ダッシュボード: {dashboard.url}")
    loop = make_realtime_loop()
    for t in loop:
        motor.update()
        motor.set_output_angle_radians(desired_angle)
        dashboard.publish(t)  # 制御ループの周期に関わらず、ブラウザへは10Hz固定で配信される
```

**テンプレート:**
- `dashboard_demo.py`: インピーダンス制御 + ダッシュボード配信の最小例

## 安全上の注意

1. **温度監視**: MOSFET 温度が config.yaml の `motor.max_temp`（現在75℃。根拠は config.yaml
   冒頭のコメント参照）を超えないよう監視してください
2. **位置制限**: モーターの可動範囲を超えないよう制限を設定してください
3. **緊急停止**: 異常時はすぐに電源を切断してください
4. **負荷確認**: モーターに過大な負荷をかけないよう注意してください

`exp_003_multi_motor.py` は `config.yaml` の `safety.max_position`/`max_velocity`/`max_torque`/`emergency_stop`
を実際に読み込み、`lib/safety_monitor.py` の `SafetyMonitor` でモーターごとに監視しています。
いずれかのモーターがしきい値を超えると（`emergency_stop: true` の場合）自動的に全モーターへ
`power_off()` を送って停止します。

`0_template_basic.py`〜`3_template_speed.py`（全テンプレート）と `exp_001_gain_tuning.py`・
`exp_002_step_response.py`・`exp_004_trajectory.py`（単一モーター実験）も `SafetyMonitor` を
使用しています。単一モーターの場合は `motors=[motor]` の1要素リストとして構築し、`update()` と
安全上限チェックをまとめて行う `SafetyMonitor.update_and_check()`（戻り値が `True` なら呼び出し側の
ループを `break` する想定）を制御ループの先頭で呼びます。

`my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py` も `SafetyMonitor` を使用しています。この実験は位置・速度フィードバック
による復元力を持たない開ループのトルク指令であるため、実測値ベースの `SafetyMonitor` によるしきい値
超過時の緊急停止に加えて、指令トルク自体も `safety.max_torque` でクランプする2層の保護を行っています。
それでも初回実行時は目視監視のもとで行ってください。

`dashboard_server.py`（`DashboardServer`）はモーターへ一切のコマンドを送らない監視専用の機能ですが、
`host="0.0.0.0"` を既定として認証なしで同一LAN上に読み取り専用データを公開します。信頼できない
ネットワークでは使用しないでください。

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

### 既存スクリプトへのダッシュボード追加

`DashboardServer` を `motors`/`motor_names` のリストと共に構築し、制御ループ内で
`dashboard.publish(t)` を1回呼ぶだけです（`dashboard_demo.py` 参照）。`SyncMultiMotorLogger`
と同じ `log_vars` を渡せば、複数モーター構成でもそのまま使えます。

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