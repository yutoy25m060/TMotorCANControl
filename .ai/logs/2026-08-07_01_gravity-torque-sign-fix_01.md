# gravity_torque() の符号バグ修正、およびフェーズB「未解決の疑義」の誤検知判明

## 冒頭メタ情報

- 日時: 2026-08-07（時刻未記録）
- 対象ファイル:
  - `my_ak45/wire_mechanism/wire_statics.py`（`gravity_torque()` の符号修正、モジュール
    docstringの「既知の課題」節を更新）
  - `my_ak45/wire_mechanism/tests/test_wire_statics.py`（平衡点・符号の期待値を修正）
  - `my_ak45/wire_mechanism/tests/test_wire_kinematics.py`（xfailテストを撤回し、
    anchor座標の手計算式を修正した上で通常のpassテストとして復元）
- 種別: バグ修正

## 設計判断と理由

`origin/claude/claude-md-docs-kdujzg` ブランチ（フェーズB `b9180e2` / フェーズC `ad5d3ea`、
未マージ）のコードレビュー依頼を受け、A-1確定規約（設計ノート第3部、
`5347f5e` で確定）との整合性を検証した結果、2件の符号関連の問題を発見・整理した。

- **`gravity_torque()` の符号バグ（実際のバグ、今回修正）**:
  A-1確定規約では `θ0`（`theta_joint`）は `θ1`・`θ2` と同一の「x軸正方向基準・CCW正」の
  回転で定義され、この座標系でx軸から角度θの位置にある任意の点は
  `z = -l・sinθ`（`wire_kinematics.pulley_xy_from_polar()` が採用する符号）に従う。
  これは3D回転行列（右手系、y軸まわり）による独立検算でも再確認した
  （`X = L cosθ`, `Z = -L sinθ` が厳密に成立）。
  ところが `gravity_torque()` は重心位置を `z_com = +l_com・sin(theta_joint)`
  （符号反転なし）としてポテンシャルエネルギーを立てており、A-1規約と矛盾する式になっていた。
  この誤りは開発者自身の検証（有限差分によるエネルギー法チェック、平衡点の数値確認）を
  通過していたが、それは `gravity_torque()` 自身の式との自己整合性を確認しているに過ぎず、
  `wire_kinematics.py` 側で既に確立された座標規約とのクロスチェックにはなっていなかった
  ため見逃されていた。
  修正により `tau_gravity = +mass・g・l_com・cos(theta_joint)` となり、安定平衡点は
  `theta_joint = -90°` ではなく `+90°`（A-1規約でz軸負方向＝鉛直下向き）に訂正した。
  `solve_static_tension_gravity()` は `l_moment_arm`（`wire_kinematics.py` 由来、
  A-1規約に正しく整合）と `gravity_torque()` を同じ `theta_joint` で組み合わせるため、
  この符号ミスは `T(θ0)` 曲線・実現可否判定（`T≥0`）の物理的な正しさに直接影響していた。
- **フェーズBの「未解決の疑義」は誤検知と判明（コード修正は不要、テストのみ更新）**:
  `wire_statics.py` のdocstring・`.ai/logs/2026-08-06_02_...md` には、
  `pulley_polar_from_xy()` の `theta_pulley = atan2(-z, x)` が `theta_anchor` 側と
  符号規約が揃っておらず `l_wire` が実ユークリッド距離と一致しない、という未解決の疑いが
  記録されていた（`test_wire_kinematics.py` に `xfail(strict=True)` として再現ケース付きで
  記録済み）。
  検証のため、A-1規約に従い anchor 座標も pulley と同じ `z = -l・sinθ` で計算した上で
  20万点の乱数パラメータで `l_wire`（余弦定理）と実ユークリッド距離を突合したところ、
  最大誤差 `3e-14`（浮動小数点誤差の範囲内）で完全一致した。
  すなわち `theta_pulley = atan2(-z, x)` にバグはなく、疑いの根拠になった
  `test_l_wire_matches_direct_euclidean_distance_to_pulley_xy` の「直接距離」の手計算
  （`z_anchor = +l_anchor・sin(...)`、符号反転なし）自体がA-1規約に反する誤った比較対象
  だったことが原因と判明した。`z_anchor = -l_anchor・sin(...)` に修正したところテストは
  xfailではなく通常passになった。
  - 却下案: 「`theta_pulley` の反転をやめる」「`theta_included` を `θ1+θ2` に変える」等、
    ノート・実装側を書き換える修正方針も `.ai/logs/2026-08-06_02_...md` に候補として
    挙がっていたが、検証の結果 pulley 側の実装は正しかったため、この方針は採用しなかった
    （実装ではなくテスト側の比較式のみを修正）。

## 未対応・既知の課題

- 今回の修正は重力項（`gravity_torque()`）のみが対象。動力学項（フェーズE-1、
  ニュートン・オイラー法）は引き続き未着手。
- A-2（拮抗 or 単方向ワイヤー）の判断は、今回の符号修正後の `τ(θ0)` を使って
  改めて可動域全体の符号反転有無を確認する必要がある（符号バグ修正前のフェーズC実装時に
  行った予備確認は前提が誤っていたため、やり直しが必要）。
- フェーズD（定滑車位置のグリッド探索、`pulley_placement_search.py`）は未着手。
- このバグ修正は `origin/claude/claude-md-docs-kdujzg`（未マージ）の内容を
  `claude/branch-list-o4tay9` にマージした上で行った。今後 `claude-md-docs-kdujzg` 側に
  追加のコミットが積まれた場合、マージ時にコンフリクトが起きる可能性がある。

## テスト状況

- [x] 単体テスト実行: `uv run pytest my_ak45/wire_mechanism/tests/ -v` — 25件全てpass
  （`test_l_wire_matches_direct_euclidean_distance_to_pulley_xy` はxfailから通常passに復元、
  `test_gravity_torque_equilibria_and_signs` / `test_gravity_torque_matches_energy_derivative` /
  `test_downward_hang_is_stable_equilibrium` は期待値を修正した上でpass）
- [ ] 統合テスト実行（対象外：フェーズDが未実装のため統合対象がまだ無い）
- [x] 手動確認:
  - `uv run ruff check my_ak45/wire_mechanism/` — `All checks passed!`
  - `uv run ruff format --check my_ak45/wire_mechanism/` — 5ファイルとも整形済み
  - `python -c "import TMotorCANControl"` の成功（既存パッケージへの影響なしを確認）
  - A-1規約（`z = -l・sinθ`）の正しさを3D回転行列（右手系、y軸まわり）で独立に数値検算
  - anchor座標を pulley と同じ規約で計算した場合の `l_wire` とユークリッド距離の一致を
    20万点の乱数パラメータで確認（最大誤差 `3e-14`）
- [ ] リグレッションテスト（既存の `TMotorCANControl` パッケージ・`control_mit_can/` には
  コード変更なし）
