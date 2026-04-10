# TMotorCANControl ワークスペース指示

このリポジトリは、CAN およびシリアルインターフェース経由で TMotor AK シリーズアクチュエータを制御する Python パッケージです。

## リポジトリの内容

- `src/TMotorCANControl/`: Python パッケージのソースコード。
- `demos/`: MIT CAN、Servo CAN、Servo Serial の各制御モード用サンプルスクリプト。
- `docs/`: Sphinx ドキュメントのソースと生成された HTML 出力。
- `README.md`: プロジェクト概要、使用例、セットアップ参照。
- `setup.cfg`, `pyproject.toml`: パッケージングとリンティングの設定。

## 推奨ワークフロー

- API ドキュメントや例は `README.md` と `docs/source/` を参照する。
- コア実装は `src/TMotorCANControl/`、実行例は `demos/` を使う。
- 既存のマネージャークラスとデモパターンとの API 互換性を維持する。
- 制御モードロジックやコマンド／状態フローの変更は、小さく絞った修正にする。

## ビルド / インストール

- 開発用にローカルインストールする:
  - `python -m pip install -e .`
- 実行時依存関係をインストールする:
  - `python -m pip install -r requirements.txt`
- 開発用ツールをインストールする:
  - `python -m pip install ruff`

## リンティング / 形式設定

- このプロジェクトは Ruff をリンティングとフォーマットに使用する。
- `pyproject.toml` の `tool.ruff` 設定に従う。
- 推奨スタイル:
  - 文字列はダブルクォーテーションを使用する。
  - 行長制限 (`E501`) は無視する。
- リント例:
  - `ruff check .`
- フォーマット例:
  - `ruff format .`

## ドキュメント

- リポジトリルートから Sphinx ドキュメントをビルドする:
  - Linux/macOS: `cd docs && make html`
  - Windows: `cd docs && make.bat html`
- ドキュメントソースは `docs/source/` にある。
- 生成済みドキュメントは `docs/build/html/` に格納される。

## テストと検証

- このリポジトリには専用の自動テストスイートはない。
- コード変更は `demos/` の該当サンプルスクリプトを実行し、パッケージのインポートが正しく行えることを確認して検証する。
- 新機能を追加する場合は、必要に応じて `README.md` やドキュメントも更新する。

## 編集のガイダンス

- 既存のパッケージ構成と命名規則を保持する。
- 新しい例は既存のデモスクリプトをテンプレートとして使用する。
- ドキュメントや API 例が変更された場合は `README.md` および `docs/source/` を更新する。
- GUI を前提とせず、ヘッドレスの Python 制御スクリプトとして扱う。

## AI 支援のための補足

- このリポジトリは CAN およびシリアル経由の TMotor アクチュエータ制御に特化している。
- 主要クラスは `src/TMotorCANControl/` にある。
- 実際の利用例は `demos/` が最も参考になる。
- Raspberry Pi / Linux 風のシリアルと CAN バスで動作する解決策を優先する。
