# exp_005_sysid_excitation.py を my_ak45/Mujoco/ 配下へ移動、出力先をgit追跡対象のdata/raw/に変更

## 冒頭メタ情報

- 日時: 2026-08-11（時刻未記録）
- 対象ファイル:
  - `my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py`（新規、
    `my_ak45/control_mit_can/experiments/exp_005_sysid_excitation.py` から `git mv`）
  - `my_ak45/Mujoco/data/raw/README.md`（新規）
  - `my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`（新規）
  - `my_ak45/control_mit_can/README_ja.md`
  - `my_ak45/control_mit_can/logs/README.md`
  - `my_ak45/control_mit_can/lib/safety_monitor.py`
- 種別: リファクタリング

## 設計判断と理由

MuJoCo sysid toolbox によるAK45-36のシステム同定作業を、実機（Raspberry Pi、CAN通信）と
GPUが使えるWindows PC（Piをリモート操作している母艦）とで分担して進めることになった。
Pi側で `exp_005_sysid_excitation.py` を実行して実機データ（multi-sine励振下の位置・速度・
電流・トルク・温度）を取得し、Windows PC側でMuJoCo sysidの最適化計算を行う。両者間の
データ受け渡しをリポジトリ（git）経由で行いたいという要望があった。

ここで、`my_ak45/control_mit_can/.gitignore` は `*.csv`/`*.log` をディレクトリ以下すべてに
適用しており（`logs/*.csv`をgit追跡対象外にする意図的な設計、
`.ai/logs/2026-08-11_06_logs-per-run-folder-and-console-log_01.md` 参照）、従来通り
`exp_005_sysid_excitation.py` を `control_mit_can/experiments/` に置いたまま
`lib.logging_utils.make_run_dir()`（`control_mit_can/logs/` 固定）で出力すると、
sysidに使う実機データそのものがgit追跡対象外になり、Windows PC側に渡らない。

- **採用した対応**:
  1. `exp_005_sysid_excitation.py` を `my_ak45/Mujoco/data_collection/` へ `git mv` した
     （`my_ak45/Mujoco/` 配下には `.gitignore` が存在せず、通常通り追跡されるため）。
  2. モーター制御自体は `my_ak45/control_mit_can/` の `lib/`（`config_loader`/`motor_setup`/
     `safety_monitor`/`logging_utils`）と `config.yaml` に強く依存しているため、これらは
     複製せず、スクリプト冒頭の `sys.path.insert()` を
     `Path(__file__).resolve().parent.parent.parent / "control_mit_can"` に変更して
     引き続き `import lib.xxx` できるようにした（`lib/config_loader.py` はモジュール自身の
     ファイル位置基準で `config.yaml` を解決するため、呼び出し元スクリプトの位置には依存しない
     — 変更不要）。
  3. 出力先を `lib.logging_utils.make_run_dir()`（`control_mit_can/logs/` 固定）の利用から、
     スクリプト内で直接 `Path(__file__).resolve().parent.parent / "data" / "raw"` を起点に
     `exp005_sysid_excitation_{タイムスタンプ}/` フォルダを作成する方式に変更した
     （`console_log`/`make_realtime_loop` は汎用実装のため引き続き `lib.logging_utils` から
     再利用）。`lib.logging_utils.make_run_dir()` 自体は他の9本のテンプレート/実験スクリプトが
     `control_mit_can/logs/` を前提に使い続けているため変更していない。
  4. `my_ak45/Mujoco/data/raw/` ディレクトリを新設。空ディレクトリはgit追跡されないため、
     用途を説明する `README.md` を置いてコミット対象にした。
  5. `README_ja.md`（ディレクトリ構成図・実行例・sysid excitationの節・安全上の注意）、
     `logs/README.md`（フォルダ構成一覧からexp_005を除去し新しい保存先を注記）、
     `safety_monitor.py`（docstring内のファイル参照）を新しいパス・構成に合わせて更新した。
- **却下案**:
  - `control_mit_can/.gitignore` に sysid用CSVだけの例外パターン（`!logs/exp005_*/**`）を
    追加する案も検討したが、（a）既存のgitignore方針（試行錯誤ログを追跡しない）を局所的に
    崩すことになる、（b）スクリプトの実行場所と出力先の対応関係が分かりにくくなる、という
    理由で見送り、スクリプトごとMuJoCo側へ移動する方式を採った（ユーザーの明示的な指示でもある）。
  - `my_ak45/Mujoco/` 側に `lib/`（config_loader等）を複製する案も検討したが、
    `config.yaml`・`SafetyMonitor`・`motor_setup` の二重管理は将来の追従漏れリスクが高いため
    見送り、`sys.path` 経由で `control_mit_can/lib` を再利用する方式にした。

## 未対応・既知の課題

- `my_ak45/Mujoco/data/raw/` に蓄積されるCSVのリポジトリサイズへの影響は未評価
  （1kHz×10秒＝10,000行/ファイル。試行数が増えた場合は要検討、
  `AK45-36_sysid_作業手順.md`の「未確定事項」参照）。
- Windows PC側の `mujoco[sysid]` インストール方法・Python環境は未決定。
- 移動後のスクリプトはPi実機上での実行確認がまだ行われていない（インポート・
  config読み込みのみサンドボックスで検証済み、下記テスト状況参照）。
- `my_ak45/Mujoco/data/raw/` に既に不要な試行が溜まった場合の整理・削除運用は未定義。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `python3 -m py_compile my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py` —
    構文エラーなし
  - `python -c "import TMotorCANControl"` — 問題なくインポートできることを確認
  - `ruff check my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py
    my_ak45/control_mit_can/lib/safety_monitor.py` — 修正箇所に起因する新規エラーなし
    （既存の無関係な `F541` 1件は移動前から存在、今回のスコープ外）
  - サンドボックス上で `sys.path` 修正後の `from lib.config_loader import load_config` 等の
    import と `load_config()` による `config.yaml`（`sysid_excitation` セクション）読み込みを
    実行して確認（CAN/モーターへの実アクセスは行わない範囲）
- [ ] リグレッションテスト: 実機（Pi）での `exp_005_sysid_excitation.py` 実行、および
  `my_ak45/Mujoco/data/raw/` への出力・Windows PC側での `git pull` 後の取得確認は未実施。
  ユーザー側での実機実行予定。
