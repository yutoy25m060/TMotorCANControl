# SysID用 multi-sine 励振実験スクリプトの追加

## 冒頭メタ情報

- 日時: 2026-08-03 19:40
- 対象ファイル:
  - `my_ak45/control_mit_can/experiments/exp_005_sysid_excitation.py`（新規）
  - `my_ak45/control_mit_can/config.yaml`
  - `my_ak45/control_mit_can/README_ja.md`
- 種別: 機能追加

## 設計判断と理由

AK45-36実機データを使ったMuJoCo SysID実装の第一歩として、純トルク指令（kp=0, kd=0）でmulti-sine
励振信号を送りCSVログを取得する実験スクリプトを追加した。励振技術は
`my_ak45/Mujoco/docs_syid/Mujoco_システム識別（SysID_モータ実機MuJoCo）について.md` に記載の
RobStride RS02での実例（`torque(t) = amp × (sin(2π·f·t) + 0.6·sin(2π·3.4f·t) + 0.3·sin(2π·7.4f·t))`）
を踏襲している。

- 実装前に `src/TMotorCANControl/mit_can.py` を直接読んで検証した結果、`set_current_gains()`
  （`mit_can.py:995-1005`）の `kp`/`ki`/`ff`/`spoof` 引数は「Dephyライブラリとの後方互換性のための
  ダミー引数」であり実際には何も使われず、呼び出すと `_TMotorManState.CURRENT` に遷移するだけと判明。
  `CURRENT` 状態での `_send_command()`（`mit_can.py:864-868`）は position=0・velocity=0・Kp=0・Kd=0
  をそのままCANフレームにエンコードするため、**既存の `CURRENT` 状態がsysid手法の要求する「kp=0,
  kd=0の純トルク指令」とすでに等価**であることを確認した。これにより、コアパッケージ
  （`src/TMotorCANControl/`）や既存の `lib/*.py` への変更は不要という判断に至った。
- 採用: `set_output_torque_newton_meters(torque)` を毎ティック呼ぶだけで既存のNm→qaxis電流変換が
  行われるため、これをそのまま利用。振幅・周波数はハードウェアリスクに直結するため、他の
  `exp_00N` と異なり、`config.yaml` に新設した `experiment.sysid_excitation` ブロックを実際に
  読み込む設計にした（他の実験の `experiment.step`/`chirp`/`trajectory` ブロックは実際には
  未使用のまま放置されている既存の不整合があるが、本実験はコードを触らずチューニングできる
  必要性が高いため、意図的にこの慣習から外れることをスクリプトのコメントで明記した）。
- 振幅初期値1.5Nmの根拠: RS02実験ではT_max比11.8〜17.6%（17Nm中2-3Nm）がsweet spotだったが、
  AK45-36の `Kt_actual`（`mit_can.py`上のコメントで「実測パラメータがないため暫定採用」と明記）は
  未検証の暫定値であるため、単純比例換算（32Nm×同比率＝3.8〜5.6Nm）をそのまま初期値にはせず、
  約2.5倍の余裕を持たせた1.5Nm（T_max比4.7%、瞬時最大2.85Nm）から開始することにした。
- harmonic比率（1.0, 3.4, 7.4）と重み（1.0, 0.6, 0.3）はsysid手法固有の固定パラメータのため、
  config.yamlではなくスクリプト内の定数として保持した。config化すると誤って比率を変更してしまう
  リスクがあるため。
- 安全設計は2層構成とした: (1) コマンド段階で `np.clip(raw_torque, -MAX_TORQUE, MAX_TORQUE)`
  によるクランプ（`2_template_current.py` の既存パターンを踏襲）、(2) 実測値ベースの
  `lib/safety_monitor.py` の `SafetyMonitor` を単一要素リスト `[motor]` で再利用し、位置/速度/
  トルクの超過を検知したら緊急停止。この実験は本ワークスペース初の開ループ実験（位置・速度
  フィードバックによる復元力を持たない）であるため、コマンド側の想定バグと実測側の物理的異常の
  両方を独立に検知できる構成とした。
- 却下案: `SafetyMonitor` を再利用せず専用の安全チェックを新規実装する案は、`exp_003_multi_motor.py`
  で既に確立されているクラスがN=1でもそのまま動作するため却下し、既存資産を再利用した。
- ログ設計: `TMotorManager_mit_can` 標準の `CSV_file` 機構は測定値（`LOG_FUNCTIONS`）のみを記録し、
  スクリプトが計算した「指令トルク」自体を記録できない。sysidでは実機に送った指令値そのものが
  MuJoCo側で再生する「入力」になるため、`ExcitationLogger` という小さなインラインクラス
  （`lib/sync_logger.py` の `SyncMultiMotorLogger` を単一モーター向けに簡略化した構造）を
  このスクリプト内に定義し、`t, desired_torque` + 測定値列を1行にまとめて記録する設計にした。
  `lib/` への追加は見送った（直近のリファクタで確立した「2箇所目の利用が出てから共有化する」
  という方針に従うため）。
- `LOG_VARS` に `current`（生の測定q軸電流）を含めたのは、`Kt_actual` が暫定値のため、後日補正した
  Ktで `output_torque` を再計算できるようにするため。
- `set_current_gains()` は引数なしで呼び出し、`config.yaml` の `control.current.Kp/Ki`
  （これも同様にダミー引数）は意図的に渡さなかった。渡すと「PD整形された電流ループ」であるかの
  ように誤読されるおそれがあるため、コード内コメントで明記した。

## 未対応・既知の課題

- 1kHzサンプリング（`dt=0.001`）は本リポジトリ内に `demo_current_chirp_mit_can.py` 等の先行事例が
  あるものの、Raspberry Pi + python-can/socketcanスタックでの実測ベンチマークは存在しない。
  ジッタが問題になる場合は `config.yaml` の `sysid_excitation.dt` を0.002等に緩めることで対応可能
  （コード変更不要）。
- 振幅1.5Nm・周波数4Hzは保守的な初期値であり、AK45-36向けに検証された値ではない。実機での初回実行
  結果（`output_velocity`/`output_angle` のピーク値）を見て、必要であれば段階的に振幅を上げる
  チューニングが別途必要。
- `Kt_actual=0.1206` が暫定値のままのため、`set_output_torque_newton_meters()` で指令したNm値と
  実際にモーターへ加わるトルクとの間に誤差がある可能性がある。`LOG_VARS` に含めた生の `current` から
  後日補正できるようにはしているが、補正自体は未実施。
- MuJoCo側（sysid toolboxへのデータ投入・パラメータ最適化）は本コミットのスコープ外。今回は実機
  データ収集スクリプトの追加のみ。
- 実機（CAN接続されたAK45-36）での動作確認は、開発環境にハードウェアが無いため未実施。

## テスト状況

- [ ] 単体テスト実行（本リポジトリに自動テストスイート無し）
- [ ] 統合テスト実行（同上）
- [x] 手動確認（`ruff check` で新規のI001を修正しF541のみ他の実験スクリプトと同じ既存パターンとして
      残ることを確認、`python -c "import TMotorCANControl"` の成功、`py_compile` での構文確認、
      `config["experiment"]["sysid_excitation"]` の読み込み確認、`multi_sine_torque()` を
      200,000点で数値評価し理論最大値 `amplitude*1.9=2.85` Nm と実測min/max `±2.8499999718...`
      が一致することを確認）
- [ ] リグレッションテスト（実機（CAN バス・AK45-36）が無い環境のため、実際の励振動作・1kHz
      サンプリングのジッタ有無の確認は未実施）
