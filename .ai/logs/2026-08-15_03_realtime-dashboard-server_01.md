# 標準ライブラリのみで実装するリアルタイムWebダッシュボード（DashboardServer）を追加

## 冒頭メタ情報

- 日時: 2026-08-15（時刻未記録）
- 対象ファイル:
  - `my_ak45/control_mit_can/lib/dashboard_server.py`（新規、`DashboardServer`クラス）
  - `my_ak45/control_mit_can/dashboard_demo.py`（新規、使用例デモスクリプト）
  - `my_ak45/control_mit_can/README_ja.md`（ディレクトリ構成・使用方法・新規節「6. リアルタイム
    Webダッシュボード」・安全上の注意・拡張方法・セットアップ節を更新）
- 種別: 機能追加

## 設計判断と理由

制御スクリプト実行中のモーター状態（位置・速度・トルク・電流・温度）を、実行中にリアルタイムで
確認したいという要望を受けて実装した。実機CANバスのないサンドボックス環境での作業のため、
実装・検証ともにハードウェアなしで完結できる範囲で行った。

### 1. 実装方式：標準ライブラリのみ（Flask等は追加しない）

CLAUDE.mdに明記されている「この構成はヘッドレスRaspberry Pi/Linux向けであり、GUI依存関係を
持ち込まないこと」という方針を踏まえ、ユーザーに`AskUserQuestion`で確認の上、以下を確定した：
- tkinter/Qt等のデスクトップGUIトースキットではなく「Webページ」として提供する（別端末の
  ブラウザから閲覧する。Piにディスプレイは不要）
- Webサーバー自体もFlask/FastAPI等の新規依存を追加せず、`http.server.ThreadingHTTPServer` +
  `threading` + `json`という標準ライブラリのみで自前実装する
- ブラウザ側もChart.js等の外部JSライブラリを使わず、`EventSource`（SSE、ブラウザネイティブAPI）
  とバニラJS + `<canvas>`のみで完結させる

これにより`pyproject.toml`の`dependencies`には一切変更がなく、`pip install`不要でこの機能が
使える（README_ja.mdのセットアップ節にも明記した）。

**却下案**: Flask等の軽量フレームワークを使う案も検討したが、CLAUDE.mdの方針と正面から
矛盾するため不採用。実装コストは標準ライブラリのみでも許容範囲（`dashboard_server.py`全体で
約300行、うち半分弱がHTML/CSS/JS文字列）と判断した。

### 2. スコープ：新規lib機能 + デモ1本のみ、既存スクリプトは変更しない

既存の`0_template_basic.py`〜`3_template_speed.py`・`experiments/exp_001/002/004`には一切
手を入れず、`lib/dashboard_server.py`（`DashboardServer`クラス）と、使い方を示す
`dashboard_demo.py`（1本、`1_template_impedance.py`と同じ制御則を使用）のみを追加した。
オプトイン機能として、必要な既存スクリプトへは`README_ja.md`「拡張方法」節の手順
（`motors`/`motor_names`のリストと共に構築し、ループ内で`publish(t)`を呼ぶだけ）に従って
利用者自身が組み込む想定。

**採用理由**: `SafetyMonitor`のときのように全既存スクリプトへ機械的に組み込む案もあり得たが、
ダッシュボードは（安全監視と異なり）「常に動いているべき」機能ではなく、必要なときにだけ
起動するデバッグ・観測ツールという性質が強いため、全スクリプトへの強制組み込みは過剰と判断した。

### 3. API設計：`SafetyMonitor`/`SyncMultiMotorLogger`と対称的な設計、複数モーター対応を最初から

`DashboardServer(motors, motor_names, log_vars, host="0.0.0.0", port=8000, push_interval=0.1)`
というコンストラクタは、既存の`SafetyMonitor(motors, motor_names, ...)`・
`SyncMultiMotorLogger(csv_file, motors, motor_names, log_vars)`と同じ「motors/motor_namesの
パラレルリストを受け取る」設計にした。単一モーターは`motors=[motor]`の1要素リストとして渡す
だけで、複数モーター（exp_003/007相当のN台構成）にもコード変更なしで使える。

`publish(t)`は`TMotorManager_mit_can.LOG_FUNCTIONS`（`update()`でキャッシュ済みの状態を返す
だけのゼロ引数ゲッター）を読むだけで、CAN通信・ネットワークI/Oを一切行わないため、1kHz制御
ループ（`exp_005_sysid_excitation.py`相当）から毎イテレーション呼んでも安全。ブラウザへの
実際のSSE配信は、`publish()`とは別スレッド（HTTPサーバーのdaemon thread）が`push_interval`
（既定10Hz）ごとに独立して行うため、制御ループの実周期やブラウザ側の接続状況が制御ループの
タイミングに一切影響しない設計にした。

**`SafetyMonitor`とは非連携（v1）**: `DashboardServer`は`LOG_FUNCTIONS`のみを読む独立クラスとし、
安全監視状態（緊急停止の有無等）は表示しない。`SyncMultiMotorLogger`も同様に`SafetyMonitor`と
非連携なため、既存の設計パターンと対称にした。将来、安全状態バナー表示が欲しくなった場合は
`safety_monitor`引数を追加する拡張余地がある旨をdocstringに残した。

### 4. `host="0.0.0.0"`・認証なし（設計判断として明示）

別端末のブラウザから閲覧するという要件上、全インターフェースへのバインドがほぼ必須のため
`host="0.0.0.0"`を既定にした。認証機構は実装していない（LAN内限定の開発・デバッグ用ツールという
位置づけとして許容）。README_ja.mdの「安全上の注意」節に「認証なしで読み取り専用データをLAN上に
公開する」旨を明記し、信頼できないネットワークでは使わないよう注意書きを添えた。表示用URLは、
実際にパケットを送らないソケットのルーティング解決トリック（`connect(("8.8.8.8", 80))`して
`getsockname()`でローカルIPを取得）でLAN上の実IPを推定して組み立てる（`0.0.0.0`のままでは
他端末から使えないため）。

### 5. UIスコープ：数値表示＋モーターごとに1変数のみのローリングチャート

各モーターカードに`log_vars`で指定した全変数を日本語ラベル付き数値表示し、加えて
`log_vars[0]`（既定では`config.yaml`の`logging.vars`の先頭＝`output_angle`）のみを
バニラJSの`<canvas>`折れ線グラフでローリング表示する。ゲイン調整・軌跡追従のようなユースケースは
数値の羅列より傾向（振動・オーバーシュート）を見たいことが多いため、チャート表示自体は
外部ライブラリなしで安価に実現できる範囲で含めた。ただし全変数をプロットする多変量チャートは
v1のスコープ外とし、複雑化を避けた。

鮮度表示は2系統用意した: (1) `age_seconds`（サーバー側で計算した最終`publish()`からの経過秒）
が1秒を超えるとカードに警告スタイル＝「制御ループが止まっている」ことを示す、(2)
クライアント側で直近にSSEメッセージ自体を受信したかを別途タイマー監視し、2秒無応答なら
「サーバーとの接続が切れた」ことを示す別メッセージを出す。両者は原因が異なる異常（制御ループ
停止 vs. 接続断）なので意図的に区別した。`EventSource`はブラウザ標準で自動再接続するため、
手動再接続ロジックは実装していない。

## 未対応・既知の課題

- 実機CANバスがないサンドボックス環境のため、実際のモーター・実機CAN通信を使った動作確認は
  未実施。代わりに、`LOG_FUNCTIONS`のみを実装した簡易フェイクモーター（実`TMotorManager_mit_can`
  は使わない）で`DashboardServer`を起動し、`urllib.request`で`/`（200・HTML）、
  `/api/state`（200・JSON、想定キー）、`/events`（SSEストリーム、複数イベントの整形JSON）を
  確認、さらにPlaywright（プリインストール済みheadless Chromium）で実際にページを開き、
  `publish()`呼び出しのたびにDOM上の数値表示が変化すること・鮮度バナーが正しく表示されること
  をスクリーンショット付きで確認した（使い捨てスクリプトのため実行後に削除、リポジトリには残していない）。
- ブラウザのコンソールに`favicon.ico`と思われる404が1件記録された（ページに`<link rel="icon">`
  を用意していないため、ブラウザが自動的にfaviconを要求し、サーバー側の「未定義パスは404」という
  設計通りの応答を返しているだけと考えられる）。機能上の問題ではないためv1では対応していない
  （favicon用の専用ルートを追加するのは過剰と判断）。
- `DashboardServer`の`__exit__`は`_stop_event`をセットして各SSE配信ループに終了を伝えるが、
  ブラウザ側が既にレスポンスの読み取りをブロックしている場合、実際に接続が閉じるまでは
  `wfile.write()`が失敗する（`BrokenPipeError`等）までタイムラグが生じ得る。daemon threadの
  ため、プロセス終了自体をブロックすることはない。
- `SafetyMonitor`との非連携は意図的な設計判断だが、将来「ダッシュボード上に緊急停止状態を
  表示したい」というニーズが出た場合は`safety_monitor`引数の追加を検討する。
- `dashboard_demo.py`は単一モーター構成のみを示すデモであり、複数モーター構成
  （exp_003/007スタイル）向けのデモスクリプトは今回作成していない（`DashboardServer`自体は
  対応済みだが、利用例としては未提供）。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `uv run ruff check my_ak45/control_mit_can/` — 全件パス
  - `uv run python -c "import TMotorCANControl"` — 成功
  - `uv run python -m py_compile` で新規ファイル2つが構文エラーなしを確認
  - フェイクモーター（`LOG_FUNCTIONS`のみ実装したスタブ、実CAN通信なし）で`DashboardServer`を
    起動し、`urllib.request`経由で`/`（200・HTML）、`/api/state`（200・JSON・想定キー）、
    `/nope`（404）、`/events`（SSEストリーム、複数の整形済みJSONイベント）をすべて確認
  - Playwright（headless Chromium、プリインストール済み、`playwright install`は未実行）で
    `/`を開き、`publish()`呼び出しのたびにDOM上の数値表示（`#val-fake1-output_angle`）が
    変化すること、鮮度バナー（`#freshness`）が正しく表示されることをスクリーンショット付きで確認
- [ ] リグレッションテスト（実機がないため未実施。次回ハードウェアアクセス時に、実モーターの
  `TMotorManager_mit_can`と組み合わせた動作確認、複数モーター構成での確認、長時間接続時の
  安定性確認を行う予定）
