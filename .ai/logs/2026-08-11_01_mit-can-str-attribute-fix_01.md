# TMotorManager_mit_can.__str__() の未定義属性参照バグ修正

## 冒頭メタ情報

- 日時: 2026-08-11（時刻未記録）
- 対象ファイル:
  - `src/TMotorCANControl/mit_can.py`（`TMotorManager_mit_can.__str__()`）
- 種別: バグ修正

## 設計判断と理由

Raspberry Pi 4Bの実機（AK45-36、CAN ID=1、`can0`）でモーター電源投入後の動作検証を
行っていたところ、テレメトリ表示用に `print(dev)` / `str(dev)` を呼んだ際に
`AttributeError: 'TMotorManager_mit_can' object has no attribute 'θ'` で例外が発生した。

`__str__()` は `self.θ` / `self.θd` / `self.τ` / `self.i` を参照していたが、これらは
このクラスのどこにも定義されていない属性だった（インスタンス変数でもプロパティでもない）。
一方、実際に定義されているプロパティは `position` / `velocity` / `current_qaxis` / `torque`
（`mit_can.py` の `__str__()` より後、クラス定義末尾付近で `property()` により定義）であり、
これらは意味的に `θ`（角度）/ `θd`（角速度）/ `i`（電流）/ `τ`（トルク）と一対一で対応する。

姉妹モジュール `servo_can.py`（`TMotorManager_servo_can.__str__()`）・`servo_serial.py`
（`TMotorManager_servo_serial.__str__()`）の同名メソッドを確認したところ、どちらも
`self.position` / `self.velocity` / `self.current_qaxis`（またはそれに相当する属性）という
実際に存在する属性名を使っており、`mit_can.py` の `__str__()` だけがギリシャ文字の別名
（存在しない）を使う形になっていた。おそらく他プロジェクト（Open-Source Leg等、
`θ`/`τ`記法を使う流儀）からのコピー時に、対応するプロパティ名へのリネームが漏れたものと
推測される。

- **採用した修正**: `self.θ` → `self.position`、`self.θd` → `self.velocity`、
  `self.i` → `self.current_qaxis`、`self.τ` → `self.torque` に置き換え。
  既存の `property()` 定義をそのまま使うだけで済み、`servo_can.py`/`servo_serial.py` と
  表示形式・スタイルが揃う。
- **却下案**: `θ`/`θd`/`τ`/`i` という別名プロパティを新規に追加する案も検討したが、
  既存の `position`/`velocity`/`torque`/`current_qaxis` という命名規則
  （`README.md`/`demos/`/`my_ak45/` 全体で使われている）と重複するエイリアスを増やすだけで、
  CLAUDE.mdの「3つのマネージャクラス間でAPI互換性を保つ」方針にも反するため不採用。

## 未対応・既知の課題

- 同種の「存在しない属性を参照する」バグが他のメソッド・他の2モジュール
  （`servo_can.py`/`servo_serial.py`）に潜んでいないかは、今回は `__str__()` 周辺のみの
  確認に留めており、網羅的な監査は行っていない。
- 実機での最終確認中、ライブラリ既定の温度上限（`max_mosfett_temp=80℃`、
  `mit_can.py:610`）に対し `my_ak45/control_mit_can/config.yaml` のプロジェクト独自の
  安全上限（`max_temp: 50℃`）を超える65℃という読み値が確認された（能動的な指令を送る前の
  IDLE状態）。これは本バグ修正とは別件だが、実機の温度状態が未解決のまま残っている。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない -- CLAUDE.md参照）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `python -c "import TMotorCANControl"` — 成功
  - `ruff check src/TMotorCANControl/mit_can.py` — 修正箇所に起因する新規エラーなし
    （既存の無関係な3件のみ残存、未着手）
  - 実機（AK45-36 / can0 / ID=1、電源投入済み）に対して `str(dev)` を呼ぶ再現テストを
    試みたが、この時点でモーターとの接続が一時的に確立できず（既知の間欠的な現象、
    電源投入直後の初回接続確認が失敗し再試行で成功するパターンを事前に観測済み）、
    実機上でのエンドツーエンド確認は完了できなかった。プロパティ名自体は
    `position`/`velocity`/`current_qaxis`/`torque` が実在することをコード上で確認済み。
- [ ] リグレッションテスト（`servo_can.py`/`servo_serial.py` には変更なし）
