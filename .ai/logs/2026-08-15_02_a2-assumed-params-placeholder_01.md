# A-2の未確定4事項に仮値を設定（正式決定待ち）、ドキュメントの矛盾も1件修正

## 冒頭メタ情報

- 日時: 2026-08-15 19:41
- 対象ファイル:
  - `my_ak45/wire_mechanism/assumed_params.py`（新規）
  - `my_ak45/wire_mechanism/tests/test_assumed_params.py`（新規、3件）
  - `my_ak45/wire_mechanism/a2_drive_mode_comparison.py`（LINK/TENSION_MINを`assumed_params`参照に変更）
  - `my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`（A-2節に仮決定表・訂正記録を追加）
- 種別: 機能追加（仮値の一元管理モジュール）＋バグ修正（ドキュメント内の数値矛盾）
- ブランチ: `wire-mechanism-docs`

## 設計判断と理由

ユーザーから「A-2の未確定4事項（目標揺動周波数・振幅、`r_drum`、リンク実慣性、`T_min`）は
後で正式に決めるので、今は仮値を設定しておいてほしい」という依頼を受けた。

### 仮値を1箇所（`assumed_params.py`）にまとめた

- **代替案として検討し却下**: `a2_drive_mode_comparison.py` にこれまで通り直接ハードコードする案。
  却下理由: 依頼の趣旨が「後で正式に決める」ことなので、更新箇所が1箇所で済む設計の方が
  ユーザーの意図に沿う。今後 `pulley_placement_search.py` を実際のパラメータで走らせる
  スクリプトやフェーズEを実装する際も、同じ値を参照できる。
- `a2_drive_mode_comparison.py` の `LINK`/`TENSION_MIN` を `assumed_params.ASSUMED_LINK`/
  `ASSUMED_TENSION_MIN` からの参照に変更した（`L_ANCHOR`/`V_MAX_MOTOR`/`TAU_MOTOR_RATED`/
  `L_ARM_MIN` は今回の4事項に含まれないため変更していない）。

### 仮値を決める過程でドキュメントの数値矛盾を1件発見・修正した

振幅・周波数の仮値を決めるにあたり `drive_modes.py` で実際に検算したところ、
A-2再検討節の結論表にある「~0.5Hz以下なら可動域±80°以下」という記載が、
**`T_min=5N` を込みで見ると成立しない**ことが判明した。この記載は `T_min=0`
（たるみを無視した場合の破綻周波数、振幅±80°で0.40Hz）を根拠にしたものだったが、
`T_min=5N` を課すと0.5Hzで振幅±80°は明確にinfeasible（`minT`が負に転じる）。
検算の結果、`T_min=5N` 込みで0.5Hzにfeasibleな振幅の上限は概ね±65°程度
（±60°はOK、±70°はNG）と判明した。

- ドキュメントには「訂正記録」として経緯を残し（既存の8-5節・8-6節・D-3節と同じ書式）、
  数値そのもの（結果1〜3の各表）は書き換えていない——これらの表自体は
  `T_min=0`または`T_min=5N`の前提を明記した個別の計算であり、内部的には矛盾していない。
  矛盾していたのは「結論」節がこれらを混同して1つの推奨に丸めた部分のみ。

### 仮値の具体的な選定

| 項目 | 仮値 | 選定理由 |
|---|---|---|
| 目標揺動周波数・振幅 | 0.5 Hz・±60° | `T_min=5N`込みでfeasibleな組み合わせとして検算済み |
| ドラム半径 `r_drum` | 40 mm | A-2再検討節の「`r_drum<42mm`では速度上限が先に効く」の閾値未満、かつ上記揺動に必要な角速度(約3.3rad/s)が速度上限(4.80rad/s)を下回ることを確認 |
| リンク実慣性 `I` | 0.030 kg·m²（一様棒近似） | 既存の `a2_drive_mode_comparison.py` の仮値をそのまま継承（CAD由来の推定値待ち） |
| 最低張力 `T_min` | 5 N | 既存の仮値をそのまま継承（実測根拠なし） |

### 回帰テストで仮値の無矛盾性を固定した

`test_assumed_params.py` で「仮値の組み合わせが単方向1本でfeasibleである」
「`r_drum`が速度上限の閾値未満である」ことをテストとして固定した。
将来誰かが `assumed_params.py` の値だけを書き換えて矛盾する組み合わせにしてしまった場合、
このテストが落ちて気づける設計にしている。

## 未対応・既知の課題

- **これらはあくまで仮値であり、正式決定ではない**。ユーザーが歩容仕様を確定させ次第、
  `assumed_params.py` を書き換える必要がある。
- `assumed_params.py` はまだ `pulley_placement_search.py` の実行スクリプトからは
  参照されていない（そのようなスクリプト自体がまだ存在しない）。仮値を使った実際の
  フェーズD探索結果の記録は今回のスコープ外。
- リンク実慣性・`T_min` は検算していない（既存の仮値をそのまま継承したのみ）。

## テスト状況

- [x] 単体テスト実行: `uv run pytest -q my_ak45/wire_mechanism/tests` — 47 passed（既存44 + 新規3）
- [x] Lint: `uv run ruff check my_ak45/wire_mechanism/` — All checks passed
- [x] フォーマット: `uv run ruff format --check my_ak45/wire_mechanism/` — 全ファイル整形済み
- [x] 手動確認: `python -m wire_mechanism.a2_drive_mode_comparison` が
      `assumed_params.py` 参照後も同じ出力（リンク慣性0.0300kg·m²等）を再現することを確認
- [ ] 統合テスト実行（対象外：実機非依存の純粋数値計算のみ）
- [ ] リグレッションテスト（対象外：`TMotorCANControl`パッケージ本体には変更なし）
