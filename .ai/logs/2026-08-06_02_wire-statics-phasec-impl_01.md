# フェーズC実装: `wire_mechanism/wire_statics.py`（重力のみの準静的な τ→T 算出）＋Phase B疑義の発見

## 冒頭メタ情報

- 日時: 2026-08-06（時刻未記録）
- 対象ファイル:
  - `my_ak45/wire_mechanism/wire_statics.py`（新規）
  - `my_ak45/wire_mechanism/tests/test_wire_statics.py`（新規）
  - `my_ak45/wire_mechanism/tests/test_wire_kinematics.py`（xfailテスト1件追加、Phase Bの既知の課題を記録）
- 種別: 機能追加（コード新規実装）＋バグ調査（未修正）

## 設計判断と理由

`ワイヤー駆動関節の運動学と定滑車配置の検討.md` フェーズC（準静的な `τ→T` の算出）を実装した。
ユーザーから「駆動方式（拮抗 or 単方向）は今すぐ決めなくてよいか」という質問があり、
「まず `τ(θ0)` を計算して符号反転の有無を実データで見てから決める方が合理的」と回答した流れで、
その `τ(θ0)` 計算そのものであるフェーズCの実装に進んだ。

- **座標系変更の反映**: A-1確定規約では `θ0=0` が「鉛直下向き」ではなく「x軸正方向（水平）」に
  変更されている。ノート第2部8-4で示していた `τ_gravity = M・g・(l1/2)・sinθ0` は旧規約
  （`θ0=0`=鉛直下向き）のままの式であり、新規約では **`cos` 版に置き換える必要がある**。
  重心の高さ `z_com = l_com・sin(theta_joint)`（A-1の非反転パラメータ化）から
  ポテンシャルエネルギー `V=M・g・l_com・sin(theta_joint)` を立て、
  `tau_gravity = -dV/d(theta_joint) = -M・g・l_com・cos(theta_joint)` として導出した。
  `theta_joint=-90°`（鉛直下向き）が安定平衡点、`+90°`（鉛直上向き）が不安定平衡点になることを
  数値確認済み（物理的に妥当）。
- **`τ=T・l5` の符号根拠を仮想仕事の原理で明示的に導出した**: ノートは `τ=F×r` という
  単純な静力学の式で `τ=T・l5` としているが、`l5`（`l_moment_arm`）を符号付きで扱う
  （E1確定規約）以上、`θ_joint` に対する一般化力としての符号を厳密に決める必要がある。
  仮想仕事 `tau_wire = -T・d(l_wire)/d(theta_joint)` から出発し、
  `d(l_wire)/d(theta_joint) = +l_moment_arm`（下記の数値較正で確認）であることを用いて
  `tau_wire = -T・l_moment_arm` を導出、静的つり合い `tau_external + tau_wire = 0` を解いて
  `T = tau_external / l_moment_arm` とした（途中に符号反転を挟まない、ノートの式とも一致）。
  - **作業中に自分自身の符号設定ミスを1回発見・訂正した**: 当初、有限差分による較正スクリプトで
    「`l5 = -d(l_wire)/d(theta_joint)`」という誤った結論を出してしまった。原因は較正コードが
    `min(|l5-(-dlw)|, |l5-(+dlw)|)` という「どちらか近い方」を採用する書き方をしており、
    実際には常に `+dlw` 側が正解であるにもかかわらず誤りに気づけない構造になっていたため。
    `l_moment_arm = l_pulley・l_anchor・sinθ3/l4` は `d(l_wire)/dθ3` の定義式そのもの
    （余弦定理を`θ3`で微分すると同じ式になる恒等式）であり、`d(theta_included)/d(theta_joint)=+1`
    （pulley側は`theta_joint`に依存しないため）と合わせて `+` が正しいと判明し、
    `wire_statics.py`・テストとも修正済み。
- **API設計**: `gravity_torque()`（外力側、差し替え可能）と `solve_wire_tension()`（力学に依らない
  純粋な `T=τ/l_moment_arm` の実現可否判定）を分離した。フェーズE-1で動力学項を追加する際、
  `gravity_torque()` の代わりに（あるいは加算して）別の `tau_external` を `solve_wire_tension()` に
  渡せば済む設計にしている。当初 `required_actuator_torque()`（`-tau_external`の符号反転）という
  中間関数を用意していたが、上記の仮想仕事による再導出で「符号反転は不要」と判明したため削除した
  （二重に符号反転を挟むと分かりにくくなるため、直接 `T=tau_external/l_moment_arm` の1段で完結させた）。
- **実現可否判定（E3確定規約）**: `abs(l_moment_arm) < l_moment_arm_min`（特異点近傍）または
  `T < 0`（ワイヤーは引く方向にしか力を出せない）のいずれかで `feasible=False` の
  ブールフラグを立てる。`tension` 自体はNaN/infを含みうる生の除算結果をそのまま返し、
  丸めたり置き換えたりしない（E3で決めた「ブールフラグで表現」という方針を、数値の破壊なしで
  実現するため）。

### 実装中に発見した Phase B（`wire_kinematics.py`）の疑義（未修正）

上記の符号較正作業の過程で、`wire_kinematics.pulley_polar_from_xy()` の
`theta_pulley = atan2(-z, x)` が `theta_anchor` 側の符号規約（反転なし）と揃っておらず、
独立に指定した定滑車座標 `(x, z)` に対して `solve_wire_geometry().l_wire` が実際の
ユークリッド距離と一致しないケースがあると判明した（`x`軸上の点以外で系統的に食い違う。
例: `l_anchor=1, α=0`, 定滑車`(1,1)` のとき `θ_joint=45°`で `l_wire`(式)=1.732 に対し
直接距離=0.414）。原因は `theta_pulley = -（真の極角）` となっており、
`theta_included = theta_anchor - theta_pulley` が実質「真の極角の差」ではなく「和」に
なってしまうため（`z=0`のときのみ偶然一致する）。

この件は**ユーザーに確認中で未修正**。`wire_kinematics.py`は別セッションで実装・マージ済みの
共有コードであり、修正方針（`theta_pulley`の反転をやめる／`theta_included`を`θ1+θ2`にする等）が
`l_moment_arm`の符号解釈に波及する可能性があるため、独断で変更せずユーザーの確認を待っている。

- `test_wire_kinematics.py` に `test_l_wire_matches_direct_euclidean_distance_to_pulley_xy` を
  `xfail(strict=True)` として追加し、問題を再現可能な形で記録した。`strict=True`により、
  今後この問題が別の変更で偶然直っても（あるいは直っていないのに別の理由で通っても）検知できる。
- 本コミットの `wire_statics.py` 自体の符号導出（`l5=+d(l_wire)/d(theta_joint)`）は、
  `theta_included`が`theta_joint`に対して傾き+1の線形関数であることのみに依存する恒等式であり、
  上記のPhase Bの疑義（pulleyの`(x,z)`→角度変換の絶対的な正しさ）とは独立に成立する
  （pulley側の角度自体は`theta_joint`に依存しないため）。したがって本コミットの内容は
  Phase Bの疑義が今後どう修正されても影響を受けず、そのままコミットして問題ない。
  ただし `l_moment_arm`・`l_wire`の**絶対値**が意図した物理配置を正しく表しているかどうかは
  Phase Bの疑義の解決を待つ必要がある。

## 未対応・既知の課題

- **最重要**: 上記の`wire_kinematics.py`の疑義（`(x,z)`→`theta_pulley`変換）は未修正。
  フェーズD（定滑車位置のグリッド探索）は`(x,z)`を直接探索変数とするため、この疑義を解決
  しないまま進めると、探索結果が意図した配置を表さない可能性が高い。フェーズDに進む前に
  必ず解決すること。
- 動力学項（フェーズE-1）は未着手。`gravity_torque()`のみの準静的モデル。
- A-2（拮抗 or 単方向）は未確定。本フェーズCの実装により`τ(θ0)`の符号反転有無を実データで
  確認できる状態になったので、次はこれを使って可動域全体で符号反転が起きるかを確認し、
  A-2の判断材料にする想定（ユーザーとの直前のやり取りで合意した進め方）。
- `l_com`（重心距離）は独立パラメータとして関数引数に持たせているのみで、具体的な値は未定
  （F1確定規約: 独立パラメータとして持たせる、を反映）。

## テスト状況

- [x] 単体テスト実行: `uv run pytest my_ak45/wire_mechanism/tests/ -v` —
  25件中24件pass、1件は既知の課題を記録する`xfail(strict=True)`（想定通りの失敗、正常）。
- [ ] 統合テスト実行（対象外：フェーズDが未実装のため統合対象がまだ無い）
- [x] 手動確認:
  - `uv run ruff format` / `uv run ruff check --fix` を適用し、警告ゼロを確認
  - `l_moment_arm = +d(l_wire)/d(theta_joint)`を2000点の乱数パラメータで有限差分較正
    （最大誤差1e-9）
  - `tau_gravity`をエネルギー法の有限差分（`-dV/dtheta`）と200点で突合（最大誤差1e-6）
  - 平衡点（`θ=±90°`）・安定性（`θ=-90°`が安定、`+90°`が不安定）を数値確認
  - `python -c "import TMotorCANControl"` の成功（既存パッケージへの影響なしを確認）
- [ ] リグレッションテスト（既存の`TMotorCANControl`パッケージ・`control_mit_can/`にはコード
  変更なし）
