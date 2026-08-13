# sysid環境の依存関係を requirements-sysid.txt として分離（mujoco[sysid] は 3.5.0 以降でのみ提供）

## 冒頭メタ情報

- 日時: 2026-08-12（時刻未記録）
- 対象ファイル:
  - `my_ak45/Mujoco/requirements-sysid.txt`（新規）
  - `my_ak45/Mujoco/requirements.txt`（冒頭に分離理由の注記を追加）
  - `my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`（前提・役割分担の補足、手順6、未確定事項）
- 種別: バグ修正（依存関係の欠落・記述誤りの修正）

## 設計判断と理由

`AK45-36_sysid_作業手順.md` の手順6は「Windows PC に `mujoco[sysid]` をインストール
（`my_ak45/Mujoco/requirements.txt` 参照）」と書いていたが、参照先の `requirements.txt` には
`mujoco==3.3.4` しかなく `sysid` extra の記載がない、という不整合をユーザーから指摘された。

調査のため PyPI のパッケージメタデータ（`https://pypi.org/pypi/mujoco/{version}/json` の
`provides_extra` / `requires_dist`）をバージョンごとに確認したところ、次が判明した。

- `sysid` extra は **mujoco 3.5.0 で追加**されている（3.3.3〜3.4.0 の `provides_extra` は `usd` のみ）。
  つまり現在ピンしている `mujoco==3.3.4` では、単に extra を書き足すだけでは解決せず、
  バージョンを上げないと `mujoco.sysid` は入らない。
- `sysid` extra の依存は absl-py / colorama / imageio[ffmpeg] / jinja2 / matplotlib / plotly /
  pyyaml / scipy / tabulate / typing_extensions の10件。これらを個別に書く必要はない。
- 公式チュートリアル（`docs_syid/sysid_mujoco_vscodeへの移設コード途中.py:43`）にある
  `pip install -q mujoco[sysid] --pre -f https://py.mujoco.org/` というプレリリース指定は、
  現在は不要（安定版PyPIに取り込み済み。最新安定版は 3.11.0）。ノートブックの記述が古い。

- **採用した対応**: sysid用の依存を `requirements-sysid.txt` として**別ファイルに分離**し、
  `mujoco[sysid]>=3.5.0` + `pandas`（CSV→`sysid.TimeSeries` アダプタ用）を記載した。
  既存 `requirements.txt` には分離理由を指す注記のみ追加した。
- **却下案1**: `requirements.txt` の Core セクションに `mujoco[sysid]>=3.5.0` を追記する。
  → `mujoco==3.3.4` と同居させると pip が依存解決に失敗する（同一パッケージへの矛盾する制約）ため不可。
- **却下案2**: `requirements.txt` の `mujoco==3.3.4` を `mujoco[sysid]>=3.5.0` に引き上げて一本化する。
  → このファイルは冒頭コメントの通り RL（stable-baselines3）・MJX/JAX など複数用途を1ファイルで
  管理しており、3.3.4 固定に依存している既存環境を壊す恐れがある。sysid は「Windows PC 側の
  独立した解析環境」という位置づけ（作業手順書の役割分担）であり、環境ごとにファイルを分ける方が
  実態と一致すると判断した。
- バージョン指定を `>=3.5.0`（下限のみ）としたのは、チュートリアルのAPIがどのバージョンで動くか
  未検証のため。実際に動作確認が取れた時点で固定する旨を、ファイル内のTODOと作業手順書の
  未確定事項の両方に明記した。

### 副次的に判明した記述誤りの訂正

チュートリアル本文（同ファイル108行目/121行目）に、最適化は「有限差分ヤコビアンを用いた
Gauss-Newton/Levenberg-Marquardt」であり、各パラメータ摂動のロールアウトは
`mujoco.rollout` への単一バッチ呼び出しで **CPUスレッド間で並列実行**される、と明記されている。
実際、ノートブックのインポートは `mujoco` / `mujoco.rollout` / `mujoco.sysid` のみで、
jax / mjx は使っていない（GPU設定のセルは Colab 上での動画レンダリング用）。

作業手順書の「前提・役割分担」は Windows PC を使う理由を「GPUが使えるから」としていたが、
これは誤り。分担そのもの（Pi=実機データ取得 / PC=最適化）は妥当なので変更せず、理由を
「CPUコア数が多い」「Piの実機制御を止めずに解析できる」に訂正する補足を追記した。

## 未対応・既知の課題

- `requirements-sysid.txt` は実際にインストールを実行して検証していない（本作業環境は
  sysid実行環境ではないため）。Windows PC 側での `pip install -r` 実行と、チュートリアルAPI
  （`sysid.ParameterDict` / `build_residual_fn` / `sysid.optimize`）が 3.5.0 以降の最新版で
  そのまま動くかの確認は未実施。
- uv / pip どちらで sysid 環境を管理するかは未決定のまま（作業手順書の未確定事項に残置）。
- `pandas` を含めたが、CSV→`sysid.TimeSeries` アダプタを numpy のみで書く場合は不要になる。
  アダプタ実装（フェーズ3の手順10）時に確定する。
- `docs_syid/` のColabノートブック本体（`.ipynb` / `.py`）に残っている古い
  `--pre -f https://py.mujoco.org/` の記述自体は修正していない。これは公式チュートリアルの
  和訳という位置づけの参照資料であり、原文を書き換えるより作業手順書側で注記する方が
  適切と判断した。

## テスト状況

- [ ] 単体テスト実行（本変更はrequirements/ドキュメントのみでコード変更なし）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - PyPI メタデータ照会により `sysid` extra の初出が mujoco 3.5.0 であること、および
    3.3.4/3.3.5/3.3.6/3.3.7/3.4.0 には存在しないことを確認
  - `sysid` extra が引き込む依存10件を確認し、明示記載が不要であることを確認
  - 本変更はrequirements/Markdownのみで、Pythonファイルは1行も変更していないため
    `ruff check` / インポート確認の対象外
    （なお作業環境には `python-can` 等の依存が入っておらず
    `python -c "import TMotorCANControl"` は元から実行不可。今回の変更とは無関係）
- [ ] リグレッションテスト: Windows PC 側での実インストール・sysid実行は未実施
