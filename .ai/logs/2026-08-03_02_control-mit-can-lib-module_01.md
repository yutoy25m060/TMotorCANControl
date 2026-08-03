# control_mit_can/ の共通コード抽出（lib/ モジュール化）

## 冒頭メタ情報

- 日時: 2026-08-03 18:57
- 対象ファイル:
  - `my_ak45/control_mit_can/lib/__init__.py`（新規）
  - `my_ak45/control_mit_can/lib/config_loader.py`（新規）
  - `my_ak45/control_mit_can/lib/motor_setup.py`（新規）
  - `my_ak45/control_mit_can/lib/logging_utils.py`（新規）
  - `my_ak45/control_mit_can/lib/sync_logger.py`（`experiments/` から移動）
  - `my_ak45/control_mit_can/lib/safety_monitor.py`（`experiments/` から移動）
  - `my_ak45/control_mit_can/0_template_basic.py`
  - `my_ak45/control_mit_can/1_template_impedance.py`
  - `my_ak45/control_mit_can/2_template_current.py`
  - `my_ak45/control_mit_can/experiments/exp_001_gain_tuning.py`
  - `my_ak45/control_mit_can/experiments/exp_002_step_response.py`
  - `my_ak45/control_mit_can/experiments/exp_003_multi_motor.py`
  - `my_ak45/control_mit_can/experiments/exp_004_trajectory.py`
  - `my_ak45/control_mit_can/README_ja.md`
- 種別: リファクタリング

## 設計判断と理由

ユーザーから「これから具体的な開発を進める中でフォルダ構成や機能別モジュール化をした方がいいか」と
いう相談を受け調査した結果、`control_mit_can/` 直下のテンプレート3本（`0_/1_/2_template_*.py`）と
`experiments/` の単一モーター実験3本（`exp_001`/`exp_002`/`exp_004`）の計6ファイルで、設定読み込み・
モーター初期化・ゼロ化・ログファイル命名・ループ生成という定型処理がほぼ丸ごとコピペされていることが
判明した。一方、複数モーター実験 `exp_003_multi_motor.py` は既に `sync_logger.py`
（`SyncMultiMotorLogger`）と `safety_monitor.py`（`SafetyMonitor`）という「機能ごとに1ファイル」の
共有モジュールを持っていた（`.ai/logs/2026-07-31_04_sync-logger_01.md`／
`2026-07-31_05_safety-monitor_01.md` で実装済み）。

- 採用: `sync_logger.py`/`safety_monitor.py` と同じ「機能ごとに1ファイル」規約を単一モーター側にも
  一般化することにした。設定読み込み（`config_loader.py`）・モーター初期化とゼロ化
  （`motor_setup.py`）・ログ命名と制御ループ生成（`logging_utils.py`）の3モジュールを新設し、
  `sync_logger.py`/`safety_monitor.py` も `experiments/` から `lib/` に移動して「共有コードは
  `lib/` に集約する」という置き場所を統一した。
  - `config_loader.py`/`logging_utils.py` は `Path(__file__)` 基準でパス解決するため、呼び出し元が
    `control_mit_can/` 直下（テンプレート）と `experiments/`（実験スクリプト）のどちらから実行
    されても同じ `config.yaml`/`logs/` を指す（従来の `"config.yaml"` と `"../config.yaml"` という
    2パターンの相対パス依存を解消）。
- これは `my_ak45/quadruped_prep_ja.md`（2026-07-31付、ワイヤー駆動4脚ロボット化に向けた助言メモ）
  が既に完了済みとしている「多モーター一般化」「同期ロギング／安全監視」の路線の延長にある整理であり、
  歩容生成・逆運動学・CAN複数バス化など機構がまだ決まっていない先回り実装は、同メモの結論
  （「機構設計が固まってから着手」）を踏襲して明示的にスコープ外とした。
- 却下案: 単一の `common.py` にまとめる案は、既存の `sync_logger.py`/`safety_monitor.py` が確立して
  いた「機能ごとに1ファイル」規約と矛盾するため却下し、複数の小さいモジュールに分ける方を採用した。
- 調査の過程で `config.yaml` の `control.realtime.dt/report/fade` がどのスクリプトからも読まれて
  いない「死んだ設定」（全スクリプトが `SoftRealtimeLoop(dt=0.01, report=True, fade=0)` を直接
  ハードコードしている）と判明したが、構造整理と挙動変更を分離するため、ユーザーに確認の上で今回は
  配線しなかった（`make_realtime_loop()` は現状のハードコード値をそのまま返す）。
- 各スクリプトのヘッダー/フッターの `print()` ブロックは、内容が実質的にスクリプトごとに異なるため
  共通化の対象から除外した（共通化すると各テンプレートの可読性が下がるため意図的に除外）。

## 未対応・既知の課題

- `config.yaml` の `control.realtime.*` は引き続き死んだ設定のまま残っている。配線するかどうかは
  今回の判断（配線しない）とは独立した、別途の判断が必要な項目。
- CAN配線・モーターID命名規則の決定（`quadruped_prep_ja.md` の優先順位3番目の項目）は今回もスコープ
  外のまま。
- `ruff check` で検出される `F541`（不要なf-stringプレフィックス、13件）はこのリファクタ前から
  存在していた既存の指摘であり、今回のスコープ外として意図的に手を入れていない。新規に追加した
  import文の並び順に関する指摘（I001）のみ `ruff check --fix --select I001` で修正済み。
- 実機（CAN接続されたAK45-36）での最終動作確認は、この開発環境にハードウェアが無いため実施できて
  いない。

## テスト状況

- [ ] 単体テスト実行（本リポジトリに自動テストスイート無し）
- [ ] 統合テスト実行（同上）
- [x] 手動確認（`ruff check` で新規のI001を修正しF541のみ既存指摘として残ることを確認、
      `python -c "import TMotorCANControl"` の成功、全 `.py` ファイルの `py_compile` 成功、
      `control_mit_can/` 直下と `experiments/` の両方から実行した場合に `load_config()`/
      `make_log_path()` のパス解決結果が一致することを確認）
- [ ] リグレッションテスト（実機（CAN バス・AK45-36）が無い環境のため、実際の制御動作の確認は
      未実施）
