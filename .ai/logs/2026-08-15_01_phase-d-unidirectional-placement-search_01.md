# フェーズD: 定滑車配置グリッド探索を単方向ワイヤー1本限定で実装、CLAUDE.mdの記載を更新

## 冒頭メタ情報

- 日時: 2026-08-15 16:16
- 対象ファイル:
  - `my_ak45/wire_mechanism/pulley_placement_search.py`（新規）
  - `my_ak45/wire_mechanism/plotting.py`（`plot_pulley_placement_heatmap()` 追加）
  - `my_ak45/wire_mechanism/tests/test_pulley_placement_search.py`（新規、13件）
  - `my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md`（D-1〜D-3・成果物表を更新）
  - `CLAUDE.md`（`my_ak45/wire_mechanism/` 節を全面更新）
- 種別: 機能追加（フェーズD実装）＋ドキュメント修正（CLAUDE.mdの陳腐化した記述を訂正）
- ブランチ: `wire-mechanism-docs`（PR #6 が既にマージ済みのため、`master` から作り直し）

## 設計判断と理由

### CLAUDE.mdの記載修正

調査の結果、CLAUDE.mdの「Known unresolved bug」節が2026-08-07（コミット`427236c`）に
解決済みの疑義をそのまま「未解決」と記載し続けていたことが判明した。実際には
`test_wire_kinematics.py`のxfailは既に解消され、`pulley_polar_from_xy()`の実装は
修正不要だったと判明している（疑いの根拠だった比較テスト側の手計算がA-1規約に
反していたことが原因）。この古い記述を訂正し、あわせてA-2再検討（2026-08-13、
`drive_modes.py`）の内容も追記した。

### フェーズDのスコープを単方向1本（2次元探索）に限定

`.ai/logs/2026-08-13_09_a2-drive-mode-reevaluation_01.md`に「A-2が確定していないため
探索次元（2 or 4）が決まらずフェーズDに着手できない」と明記されていた。ユーザーに
確認した結果、A-2の暫定結論（単方向1本、~0.5Hz以下）に沿って**単方向1本・2次元探索
(x,z)のみ**を実装する方針で合意した。拮抗2本用の4次元探索(x1,z1,x2,z2)はA-2確定後に
別途実装する前提とし、モジュールdocstringと設計ドキュメントの両方に明記した。

- **代替案として検討し却下**: A-2を確定させてから着手する案。
  却下理由: A-2確定には歩容仕様（未定）が必要で、このセッション内では決定不可能。
  単方向1本の探索自体はA-2の値によらず独立して実装・テスト可能なので、先に作る方が
  手戻りが少ないと判断した。

### 8-3（ワイヤー・リンク非干渉）・物理的取り付け可能性は未実装のまま明記

D-2は本来4つの制約を全て実装すべきとドキュメントに書かれているが、8-3と物理的取り付け
可能性の判定には**リンクの太さ・断面形状・フレーム外形**という、本リポジトリのどこにも
定義されていない仕様が要る。存在しない仕様を仮定して実装するとかえって誤った安全指標を
返すため、**あえて未実装のままにし、`feasible=True`を「実機で組める」の意味に使わないよう
モジュールdocstring・設計ドキュメント・CLAUDE.mdの3箇所に明記**した。
- **代替案として検討し却下**: リンクを単純な線分（原点→アンカー点）とみなした簡易交差判定を
  実装する案。却下理由: 交差判定に必要な「リンクの太さ」を仮の値で埋めることになり、
  仮定次第で結果が変わる指標を「実装済み」と称するのはミスリーディング。未実装であることを
  明記する方が、後で正しい仕様が決まったときに実装する際の判断を誤らせない。

### `PlacementGridResult`の設計: 制約違反理由を分離したフラグにした

D-3が「制約違反の領域はマスクして色分けする（除外された理由が分かるように）」と要求して
いたため、`feasible`（総合可否）に加えて`singular`（8-2: l5_min違反）と
`slack_or_reversed`（8-1: T<tension_min）を独立したbool配列として返す設計にした。
`drive_modes.unidirectional()`の`DriveModeResult`はNaN/infのみを検出し、l5_minの閾値
チェックは持たないため、`pulley_placement_search.py`側で別途`np.min(np.abs(l_moment_arm))`
を計算して判定している。

- **バグ**: NaN比較（`np.nan < l_moment_arm_min`）は常に`False`を返すため、
  `l_wire=0`による完全な退化（`l_pulley == l_anchor`かつ`theta_included=0`）でNaNが
  発生するケースを`np.min()`任せにすると特異点判定を見逃す。
  `np.any(~np.isfinite(abs_arm)) or ...`で先にNaN/infを短絡的に検出するよう修正した。

### D-1（評価指標）は2つとも計算する設計にした

ドキュメントが「必ず一方に決める、あるいは両方計算して比較する」としていたため、
`max_tension`（推奨・第一候補）と`tension_range`（第二候補）の両方を全グリッドセルに
ついて計算し、`best_by_max_tension()`/`best_by_tension_range()`をそれぞれ独立に提供した。
`mean(T)`（第三候補）は実装しなかった——ドキュメントが「必ず一方、あるいは両方」としており
3つ目まで実装する必要はないと判断（過剰実装の回避）。

### `tau_external_sweep`は呼び出し側が計算する設計にした（既存の層構造を維持）

`wire_statics.py`/`drive_modes.py`と同様、本モジュールはどの物理モデル（重力のみ／動力学込み）
を使うかに依存しない。`gravity_torque()`や`wire_torque_demand()`の結果をそのまま渡せる。

## 未対応・既知の課題

- **拮抗2本用の4次元探索は未実装**。A-2が確定してから着手する。
- **8-3（ワイヤー・リンク非干渉）と物理的な取り付け可能性は未実装**（上記参照）。
  `search_unidirectional_placement()`の`feasible=True`は「8-1・8-2を満たす」の意味でしかない。
- グリッド探索は総当たり（`O(len(x_grid)*len(z_grid))`のPythonループ）で、粗いグリッド
  （数十×数十点）では十分高速だが、ドキュメントが将来的に想定する細かいグリッドでは
  ベクトル化が要るかもしれない（未計測、現時点では最適化不要と判断）。
- `plot_pulley_placement_heatmap()`はmatplotlibの`pcolormesh`を2回重ね描きする実装
  （本体のヒートマップ＋制約違反の色分けオーバーレイ）で、大きいグリッドでの見た目や
  パフォーマンスは未確認（テストは非対話的な`Agg`バックエンドでのスモークテストのみ）。

## テスト状況

- [x] 単体テスト実行: `uv run pytest -q my_ak45/wire_mechanism/tests` — 44 passed
      （既存34 + 新規13、うち1件は本モジュール変更に伴う`test_drive_modes.py`側の既存件数変更なし）
- [x] Lint: `uv run ruff check my_ak45/wire_mechanism/` — All checks passed
- [x] フォーマット: `uv run ruff format --check my_ak45/wire_mechanism/` — 全ファイル整形済み
- [x] 手動確認: `plot_pulley_placement_heatmap()`を`Agg`バックエンドで実行し、
      ヒートマップ・制約違反の色分け・bestマーカーが期待通り描画されることをPNG出力で確認
- [ ] 統合テスト実行（対象外：実機非依存の純粋数値計算のみ）
- [ ] リグレッションテスト（対象外：`TMotorCANControl`パッケージ本体・`control_mit_can/`には変更なし）
