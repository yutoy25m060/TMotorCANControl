# 複数モーター向け安全制限・緊急停止機構の追加

## 冒頭メタ情報

- 日時: 2026-07-31 17:20
- 対象ファイル:
  - `my_ak45/control_mit_can/experiments/safety_monitor.py`（新規）
  - `my_ak45/control_mit_can/experiments/exp_003_multi_motor.py`
  - `my_ak45/control_mit_can/README_ja.md`
- 種別: 機能追加

## 設計判断と理由

`TMotorManager_mit_can.update()`（`src/TMotorCANControl/mit_can.py`）はMOSFET温度チェックと、MIT プロトコル生の範囲でのラップアラウンド処理は行うが、運用者が `config.yaml` で決めた位置/速度/トルクのソフトウェア上限のチェックや、複数モーター横断の緊急停止（1台の異常で全台を止める）は存在しない。`config.yaml` の `safety.*`（`max_position`/`max_velocity`/`max_torque`/`emergency_stop`）は宣言されているだけでどのスクリプトからも読まれていなかった（過去の監査で確認済み）。ワイヤー駆動の脚機構は単体モーターより過張力・断線・詰まりのリスクが高く、複数の脚が同時に動くため1台の異常が転倒や他の脚への連鎖ダメージにつながりやすい。

- 採用: `my_ak45/control_mit_can/experiments/safety_monitor.py` に `SafetyMonitor` クラスを新規追加。コアパッケージ（`src/TMotorCANControl/`）の制御モード状態機械には手を入れず、ワークスペース層で監視ロジックを実装した。CLAUDE.md が「制御モード／コマンド状態機械の変更は狭い範囲に留める（モーターの安全動作に直結するため）」と明記していることに沿った判断。
  - `check()`: 全モーターの出力角度・速度・トルクを取得し、`config.yaml` の `safety.max_position`/`max_velocity`/`max_torque` のいずれかを超えていれば `(True, メッセージ)` を返す。
  - `trigger_emergency_stop()`: 該当した場合に全モーターへ `power_off()` を送る（1台のみでなく全台を止めるのが今回のポイント）。
- `exp_003_multi_motor.py` 側では、目標位置コマンドを送信する前に `np.clip(target_pos, -MAX_POSITION, MAX_POSITION)` でクランプする「コマンド段階の安全弁」も追加した。これは `SafetyMonitor.check()`（実際の状態を見て事後的に検知）とは独立した防御層で、そもそも上限を超える指令を送らないようにする。
- `config["safety"]["emergency_stop"]` が `false` の場合は、超過を検知しても停止せず警告メッセージのみ表示する分岐にした（設定でオン/オフできるようにするため）。
- 却下案1: コアパッケージの `TMotorManager_mit_can.update()` 自体に位置/速度/トルクの上限チェックと緊急停止を組み込む案は、CLAUDE.md の「制御モード変更は狭い範囲に」という方針、および単一モーター用の他のデモ・テンプレートへの影響範囲が広がることを踏まえて却下。ワークスペース固有の `safety.*` 設定はワークスペース層で完結させる方が安全。
- 却下案2: `SafetyMonitor` を `sync_logger.py` と同じファイルにまとめる案は、責務が異なる（ロギング vs 安全監視）ため、可読性・再利用性の観点から別ファイルに分離した。
- スコープ: 今回は複数モーター実験である `exp_003_multi_motor.py` にのみ適用した。0/1/2番テンプレートや `exp_001`/`exp_002`/`exp_004`（単一モーター用）は対象外とした。これは事前の計画合意（`/root/.claude/plans/jazzy-soaring-dawn.md`）で確認済みのスコープ。

## 未対応・既知の課題

- `SafetyMonitor` は状態（位置・速度・トルク）ベースの事後チェックとコマンドクランプの2層構成だが、電流の瞬間的なスパイクなど、`update()` 呼び出し間の変化を捉えられない異常には対応できない。より高頻度・低レイテンシな保護が必要な場合は、モーター側のハードウェア保護（MIT プロトコル自体の限界値、CubeMars 側のファームウェア保護）に頼る必要がある。
- 0/1/2番テンプレートおよび `exp_001`/`exp_002`/`exp_004`（単一モーター用スクリプト）には `SafetyMonitor` を適用していない。単一モーターでも同様の恩恵はあるため、必要になれば同じパターンを流用して追加できる。
- テンション（張力）そのものを直接測定するセンサーは現状の TMotor ライブラリには存在せず、`get_output_torque_newton_meters()` から間接的にトルクを見ているのみ。実際のワイヤー駆動機構ができた際は、トルク上限だけでなく機構側の物理的なテンションリミッター（スリップ機構等）も併用することを推奨する。

## テスト状況

- [ ] 単体テスト実行（本リポジトリに自動テストスイート無し）
- [ ] 統合テスト実行（同上）
- [x] 手動確認（`python3 -m py_compile` で `exp_003_multi_motor.py`/`safety_monitor.py` の構文確認、`ruff check` で新規警告が増えていないことを確認。既存の I001/F541 は変更前から存在する無関係な指摘）
- [ ] リグレッションテスト（実機（CAN バス・AK45-36、複数モーター）が無い環境のため、実際の緊急停止動作の確認は未実施）
