# DashboardServerにSafetyMonitor連携（安全状態バナー）を追加、複数モーター向けデモを新設

## 冒頭メタ情報

- 日時: 2026-08-15（時刻未記録）
- 対象ファイル:
  - `my_ak45/control_mit_can/lib/dashboard_server.py`（`safety_monitor` オプション引数、
    安全状態バナーUIを追加）
  - `my_ak45/control_mit_can/dashboard_demo.py`（既存の`safety_monitor`を`DashboardServer`へ渡すよう更新）
  - `my_ak45/control_mit_can/dashboard_demo_multi_motor.py`（新規、複数モーター版デモ）
  - `my_ak45/control_mit_can/README_ja.md`（「6. リアルタイムWebダッシュボード」節・
    ディレクトリ構成・使用方法を更新）
- 種別: 機能追加

## 設計判断と理由

前回（`2026-08-15_03_realtime-dashboard-server_01.md`）で実装したリアルタイムWebダッシュボード
（`DashboardServer`）は、意図的に`SafetyMonitor`と非連携（`LOG_FUNCTIONS`のみを読む読み取り専用）
で作った。ユーザーからの追加要望を受け、以下2点を実装した（「ブラウザからのコマンド送信で
監視→制御へ方針転換する」案は今回のスコープ外と回答済み）。

### 1. SafetyMonitor連携：`check()`を使い`update_and_check()`は使わない

`DashboardServer.__init__`に`safety_monitor=None`のオプション引数を追加し、`publish(t)`の中で
`self._safety_monitor.check()`（安全上限超過の有無を読むだけの純粋関数）を呼んで結果を
共有状態に格納、`/api/state`・`/events`のJSONに`safety_ok`（true/false/null）・
`safety_message`（文字列/null）として公開するようにした。

**`update_and_check()`ではなく`check()`を選んだ理由**: `SafetyMonitor.update_and_check()`は
内部で`for motor in self.motors: motor.update()`を呼ぶ。制御ループ側は既に自分の周期の中で
`safety_monitor.update_and_check()`（またはテンプレート/実験スクリプトによっては旧来の
`motor.update()`）を呼んだ後に`dashboard.publish(t)`を呼ぶ想定であり、`publish()`の中でも
`update_and_check()`を呼んでしまうと、1制御周期に対して`motor.update()`が2回走ることになる。
`update()`はCANへコマンドを実際に送信する副作用を持つメソッドのため、これは単なる無駄な
CPU消費ではなくCAN送信タイミング・状態遷移を余計に乱すおそれがある。一方`check()`は
既に`update()`済みの状態（各種`get_output_*`ゲッター、`LOG_FUNCTIONS`と同様にCAN通信を
発生させない）を読むだけの副作用なし関数のため、`publish()`から独立して安全に呼べる。

**`safety_monitor`未指定時は`null`のままにする理由**: 「安全監視なし」を「正常」と区別する
ため。`safety_monitor`を渡さないダッシュボードで`safety_ok: true`を返してしまうと、実際には
一切チェックしていないのに「安全」と誤って伝えることになる。UIも同様に、`safety_ok === null`
の場合はバナー自体を非表示にする（緑色で「安全」と表示することはしない）。

**ダッシュボードはあくまで表示のみ**: `publish()`は`safety_monitor.check()`の結果を読んで
配信するだけで、`trigger_emergency_stop()`は一切呼ばない。緊急停止の実行は従来どおり制御
ループ側の`safety_monitor.update_and_check()`の責務のまま変更していない（ダッシュボードが
安全機構の一部になってしまう＝ダッシュボードのバグや接続断が安全機構に影響する、という
設計を避けるため）。

UI側は、既存の鮮度バナー（`#freshness`、データが古いかどうか）とは別に安全状態バナー
（`#safety-banner`）を独立要素として追加した。「データが古い」（制御ループ停止・接続断）と
「値が上限を超えている」（値は届いているが危険域）は原因の異なる別種の警告のため、意図的に
分離した。

### 2. 複数モーター向けデモ：新規ファイルとして追加、`exp_003_multi_motor.py`自体は変更しない

`DashboardServer`自体は当初から`motors`/`motor_names`の複数要素リストに対応する設計だった
ため、本体への機能追加は不要（Aで追加した`safety_monitor`引数を渡すだけ）。複数モーターでの
使い方を示すため、`experiments/exp_003_multi_motor.py`と同一の制御内容・安全策
（`build_motor_managers()`→`ExitStack`→`zero_positions()`→`SafetyMonitor`→ゼロ化後の
残留誤差チェック→正弦波軌跡）を踏襲した新規ファイル`dashboard_demo_multi_motor.py`を
トップレベルに追加した（`dashboard_demo.py`と同じく番号なし配置）。

相違点は2つ:
1. `exp_003`独自の`try/except RuntimeError` + `check()`という古いインラインパターンの代わりに、
   今回すでに0〜3番テンプレート・`exp_001/002/004`で確立済みの
   `if safety_monitor.update_and_check(): break`パターンを採用した（`exp_003`自体は実機
   検証済みのため今回も変更せず、新規ファイルのみ新しい共通パターンを使う）。
2. `sync_logger.log(t)`に加えて`dashboard.publish(t)`を呼び、`DashboardServer`を`ExitStack`
   に載せて全モーター・CSV・ダッシュボードサーバーの後始末を一括で保証する。

**却下案**: `exp_003_multi_motor.py`自体にダッシュボードを直接組み込む案も検討したが、
実機で検証済みの既存実験スクリプトへの変更は前回（`2026-08-15_02_*`のSafetyMonitor拡張時）
と同じ理由で避け、新規ファイルとして独立させた。

## 未対応・既知の課題

- 実機CANバスがないため、`dashboard_demo_multi_motor.py`の`ExitStack`内での複数モーター
  初期化自体（実際のCAN通信を伴う`build_motor_managers()`・`zero_positions()`）は動作確認
  できていない。`DashboardServer`の複数モーター配信自体はフェイクモーター2台での検証は
  行っていない（v1ではフェイクモーター2台で検証済み。今回のv2差分は`safety_monitor`連携のみ
  のため、単一モーター構成のフェイクモーターでの検証にとどめた）。
- `safety_ok`/`safety_message`は`SafetyMonitor.check()`の戻り値をそのまま表示するだけで、
  実際に緊急停止（`power_off()`）が発動済みかどうかまでは区別できない（`check()`は上限超過の
  有無を返すだけで、その後実際に停止されたかは呼び出し側の`update_and_check()`次第）。
  停止済みかどうかも見たい場合は将来の拡張課題。
- ダッシュボードの安全バナーは`SafetyMonitor`の`emergency_stop_enabled=False`（警告のみ、
  実際には停止しない設定）でも`safety_ok: false`を表示する。これは意図した挙動（値としては
  上限を超えている事実を伝える）だが、「警告のみで動作継続中」なのか「まもなく緊急停止する」
  なのかをUI上で区別していない。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `uv run ruff check my_ak45/control_mit_can/` — 全件パス
  - `uv run python -c "import TMotorCANControl"` — 成功
  - `uv run python -m py_compile` で変更・新規ファイルすべて構文エラーなしを確認
  - フェイクモーター1台 + 実際の`SafetyMonitor`インスタンスで`DashboardServer`を起動し、
    `urllib.request`経由で`/api/state`の`safety_ok`/`safety_message`が (1) 正常時
    `(True, None)`、(2) 位置上限超過時`(False, "...位置上限超過...")`、(3) `safety_monitor`
    未指定時`(None, None)`の3パターンすべて期待通りであることを確認
  - Playwright（headless Chromium）で実際にページを開き、安全バナーが正常時は緑「安全: 正常」、
    上限超過時は赤「⚠ 安全上限超過: ...」に切り替わることをスクリーンショット付きで確認
  - `dashboard_demo_multi_motor.py`は静的チェック（ruff/py_compile）のみ（実機なしのため
    ExitStack内の複数モーター初期化自体は未検証）
- [ ] リグレッションテスト（実機がないため未実施。次回ハードウェアアクセス時に、
  `dashboard_demo.py`・`dashboard_demo_multi_motor.py`双方を実モーターで動作確認予定）
