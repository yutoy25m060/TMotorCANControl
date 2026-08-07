# フェーズB実装: `wire_mechanism/wire_kinematics.py`（ワイヤー駆動関節の幾何計算）

## 冒頭メタ情報

- 日時: 2026-08-06 (時刻未記録)
- 対象ファイル:
  - `my_ak45/wire_mechanism/__init__.py`（新規）
  - `my_ak45/wire_mechanism/wire_kinematics.py`（新規）
  - `my_ak45/wire_mechanism/tests/test_wire_kinematics.py`（新規）
  - `pyproject.toml`（`pytest` を dev 依存に追加、`[tool.pytest.ini_options]` を追加）
  - `my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`（第3部A-1に「確定した規約」を追記、B-1/B-2の注意点を更新。本コミットに先行するやり取りで反映済み）
  - `my_ak45/docs_mechanism/A-1_座標系規約_回答シート.md`（同上、反映済みステータス追記。同じく先行のやり取りで反映済み）
- 種別: 機能追加（コード新規実装）

## 設計判断と理由

`ワイヤー駆動関節の運動学と定滑車配置の検討.md` 第3部フェーズA（座標系・符号規約）が、
ユーザーとの往復（回答シートへの記入＋手書き検討画像 `IMG_0789`〜`IMG_0805` でのフォローアップ）で
確定した。次のフェーズB（幾何モデルの実装と検証）を実装した。

- **配置場所**: `my_ak45/wire_mechanism/` を新設。ノート第4部が想定していた
  `wire_kinematics.py`（フェーズB）/ `wire_statics.py`（フェーズC）/ `pulley_placement_search.py`
  （フェーズD）の3ファイル構成の置き場所として、既存の `control_mit_can/lib/`（実機CAN制御用）
  とは別ディレクトリに分けるというノート自身の方針に従った。`control_mit_can/lib/` と同様
  `__init__.py` を持つパッケージとした。
- **API設計**: 小さな純関数（`pulley_polar_from_xy` / `anchor_angle` / `included_angle` /
  `wire_length` / `moment_arm` 等）＋それらを束ねるオーケストレーター `solve_wire_geometry`
  （`WireGeometry` frozen dataclass を返す）という二段構成にした。フェーズCで `τ(θ0)` と
  `l5(θ0)` を個別にプロットする必要がある（ノートC-2の注意点）ため、個々の量を単独で
  呼び出せることを優先した。全関数を `numpy` ufunc のみで実装し、スカラー・`ndarray`
  どちらの入力でも分岐なしで動作するようにした（フェーズCの `θ0` 掃引を見込んだ設計）。
- **命名規約**: `A-1_座標系規約_回答シート.md` のG3回答（「意味のある名前にする」）に従い、
  `l2`→`l_pulley`、`l3`→`l_anchor`、`l4`→`l_wire`、`l5`→`l_moment_arm`、`θ0`→`theta_joint`、
  `α`→`theta_anchor_offset`、`θ1`→`theta_anchor`、`θ2`→`theta_pulley`、`θ3`→`theta_included`
  という記述的な識別子を採用した。ノート記号との対応はモジュール先頭のdocstringに一覧化し、
  トレーサビリティを確保した。
- **l5の符号**: `l5 = l_pulley・l_anchor・sin(θ3) / l_wire` を符号付きのまま実装し、
  `abs()`/`sqrt()` で潰していない（E1の決定＝後続フェーズの `T ≥ 0` 判定に必要）。
  ノート第1部の非簡略化式（`sqrt(l3² − (...)²)`、常に非負）は公開APIに含めず、
  テストファイル内のプライベートヘルパー（クロスチェック専用）としてのみ実装した。
  本番コードが誤って非負版を掴んでしまう経路を作らないための判断。
- **ゼロ除算のガード方針**: `l_wire → 0`（`l_pulley == l_anchor` かつ `θ3 == 0` の完全退化点）
  での `moment_arm` のゼロ除算は、本フェーズでは意図的にガードしなかった。ノートC-2の
  「`l5=0` でのゼロ除算ガード」は `T = τ / l5` 側（フェーズC `wire_statics.py`、未実装）の話であり、
  本モジュールの分母 `l_wire` とは別の除算のため。挙動は
  `test_degenerate_equal_lengths_zero_angle_is_unguarded_nan` で `NaN` になることを明示的に
  固定し、後日の「仕様か不具合か」の混乱を防いだ。
  - 却下案: `wire_length`/`moment_arm` 内で `l_wire` に下限クリップを入れる案も検討したが、
    フェーズBは「ノートの数式をそのまま純粋関数化する」段階と位置づけ、安全装置的な変更は
    ガードが本来必要な箇所（フェーズCのT計算）で入れる方が責務が明確なため見送った。
- **テスト基盤**: 本リポジトリで初めてpytestを導入した（`uv add --dev pytest`）。
  `pyproject.toml` に `[tool.pytest.ini_options]` で `pythonpath = ["my_ak45"]` /
  `testpaths = ["my_ak45/wire_mechanism/tests"]` を追加し、`sys.path.insert` を使わずに
  `from wire_mechanism import wire_kinematics as wk` で解決できるようにした
  （`sys.path.insert` は既存の `experiments/` スクリプトの流儀だが、テストコード内で使うと
  ruffの `E402`（import前にコードがある）に抵触するため避けた）。
- **本体ノートの更新**: 実装に先立ち、ユーザーとの検算のやり取りで座標系規約の細部
  （`z = −l2 sinθ2` の符号反転、`θ3 = θ1 − θ2`、`l5` の `cos`→`sin` 修正、`l4` の挟角が
  `θ2` ではなく `θ3` であること）が確定・修正された。この変更は本コミットより前の会話ターンで
  ノート本体（第3部A-1「確定した規約」節）と回答シートに反映済みであり、本コミットはその
  確定後の規約をそのままコード化したもの。

## 未対応・既知の課題

- フェーズC（`wire_statics.py`: `τ(θ0)`, `T(θ0)` の算出、`l5≈0` 近傍のゼロ除算ガード）は未着手。
- フェーズD（`pulley_placement_search.py`: `(x,z)` グリッド探索、非干渉制約・`l5_min` 制約）は未着手。
- `α`（リンクと`l3`のなす角）の具体的な数値・機構上の実現方法は未定。現状は独立パラメータとして
  関数の引数に持たせているのみ。
- `matplotlib`/`scipy` は未導入（フェーズC/Dのプロット・最適化で必要になった時点で追加する想定）。
- ノート第1部（原ノートの構造化）自体は今回変更していない。数式の食い違い・修正はすべて
  第3部側の「確定した規約」節に反映する既存方針を踏襲した。

## テスト状況

- [x] 単体テスト実行: `uv run pytest -v` — `my_ak45/wire_mechanism/tests/test_wire_kinematics.py`
  の11件すべてpass（ノートB-2の検証表5項目＋符号保持・原式クロスチェック・ラウンドトリップ・
  パイプライン整合性・配列入力対応・退化点挙動の回帰テスト）。
- [ ] 統合テスト実行（対象外：フェーズC/Dが未実装のため統合対象がまだ無い）
- [x] 手動確認:
  - `uv run ruff format` / `uv run ruff check --fix` を新規ファイルに適用し、警告ゼロを確認
    （import順の自動修正1件のみ）
  - `uv run ruff check my_ak45/wire_mechanism/ pyproject.toml` で新規分のみ再チェックし
    `All checks passed!` を確認
  - リポジトリ全体の `ruff check .` は実行したが、`demos/` 等の**既存**警告（本変更と無関係）が
    多数あり、今回の変更範囲外として対応していない
- [ ] リグレッションテスト（既存の `TMotorCANControl` パッケージ・`control_mit_can/` には
  コード変更なし。念のため `python -c "import TMotorCANControl"` の成功は確認済み）
