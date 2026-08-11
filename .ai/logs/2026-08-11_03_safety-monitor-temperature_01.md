# SafetyMonitor に温度監視を追加、exp_003/exp_005 のupdate()呼び出しを緊急停止フローに統合

## 冒頭メタ情報

- 日時: 2026-08-11（時刻未記録）
- 対象ファイル:
  - `my_ak45/control_mit_can/lib/safety_monitor.py`（`check()`に温度分岐を追加、docstring更新）
  - `my_ak45/control_mit_can/experiments/exp_003_multi_motor.py`（`motor.update()`をtry/exceptで囲む）
  - `my_ak45/control_mit_can/experiments/exp_005_sysid_excitation.py`（同上）
- 種別: 機能追加

## 設計判断と理由

`config.yaml`の`max_temp`を50→75℃に変更した際、`SafetyMonitor`（位置/速度/トルクの複数モーター
横断監視）が温度を監視対象に含めていないことを指摘し、追加することになった。

調査の結果、`TMotorManager_mit_can.update()`は各モーター自身の`max_temp`を超えると即座に
`RuntimeError`を送出する既存の仕組みを持つが、これは`SafetyMonitor`の「メッセージ表示→
`trigger_emergency_stop()`」という統一フローの外側で起きる生の例外であり、`exp_003`/`exp_005`
のようにwith/ExitStackの外まで伝播してスクリプトをクラッシュさせる（電源は`__exit__`経由で
安全に切れるが、他の安全チェックとUXが非対称）という課題があった。

- **採用した対応**:
  1. `SafetyMonitor.check()`に温度分岐を追加。新規のコンストラクタ引数（`max_temperature`のような
     もの）は増やさず、各モーターが既に保持している`motor.max_temp`属性（`TMotorManager_mit_can`
     構築時にconfig.yamlの`motor(s).max_temp`から設定済み）をそのまま使う。理由: config.yamlの
     `max_temp`はモーターごとに異なりうる値であり、`MAX_POSITION`等のような単一スカラーを
     `SafetyMonitor`側に別途持たせると、config.yamlの値と二重管理になり食い違いのリスクが生まれる。
  2. `exp_003_multi_motor.py`・`exp_005_sysid_excitation.py`の`motor.update()`呼び出しを
     `try/except RuntimeError`で囲み、捕捉したら`safety_monitor.trigger_emergency_stop(str(e))`
     を呼んでから`break`する。これにより、`update()`自身が先に温度超過を検知した場合でも、
     既存の位置/速度/トルク超過時と同じ緊急停止メッセージ・全台停止フローに合流する。
     `emergency_stop_enabled`フラグでは分岐させない（`update()`が例外を送出した時点で該当モーター
     の制御は既に継続不能なため、無条件で全台停止する）。
- **却下案**: `SafetyMonitor`に`max_temperature`という新規コンストラクタ引数を追加し、各モーター
  の`max_temp`とは独立した「早期警告用のより低いしきい値」を持たせる案も検討した。これなら
  `update()`自身の内部チェックより先に`SafetyMonitor.check()`側が発火し、"保険的な位置づけ"という
  但し書きが不要になる。しかし、しきい値を2箇所（config.yamlの各`max_temp`と、新設の
  `safety.max_temperature`）で管理することになり、将来どちらか一方だけ変更されて食い違う
  リスクを増やすと判断し、不採用とした（既存の`motor.max_temp`を再利用する設計を優先）。
  この結果、`SafetyMonitor.check()`の温度分岐は、通常の呼び出し順序（全モーターを`update()`した
  後で`check()`を呼ぶ）では実質的に`update()`側のtry/exceptに先を越されるため、その旨をdocstring
  に明記した。

## 未対応・既知の課題

- `SafetyMonitor`の温度チェックは、現在の呼び出しパターン（`update()`全台実行後に`check()`を呼ぶ）
  では実際には発火しない「保険」的な分岐になっている。将来、`update()`を呼ばずに`check()`だけを
  呼ぶような使い方が出てきた場合に初めて意味を持つ。この非対称性はdocstringに記録したが、根本的に
  解消はしていない。
- `exp_001`/`exp_002`/`exp_004`/テンプレート類は`SafetyMonitor`を使用していないため、今回の修正は
  適用していない（そもそも単一モーター・`SafetyMonitor`未使用のため対象外）。
- 実機での緊急停止動作の実地確認（温度超過を実際に起こしてtry/exceptが機能するかのテスト）は
  未実施。正常系（温度超過なしで誤発火しないこと）の実機確認も未実施。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `ruff check my_ak45/control_mit_can/lib/safety_monitor.py
    my_ak45/control_mit_can/experiments/exp_003_multi_motor.py
    my_ak45/control_mit_can/experiments/exp_005_sysid_excitation.py` —
    修正箇所に起因する新規エラーなし（exp_003/exp_005に既存の無関係なF541が1件ずつ残存、
    未着手）
  - `python -c "import TMotorCANControl"` — 成功
  - 3ファイルとも`ast.parse()`で構文エラーなしを確認
- [ ] リグレッションテスト（実機での温度超過再現・正常系動作確認は未実施）
