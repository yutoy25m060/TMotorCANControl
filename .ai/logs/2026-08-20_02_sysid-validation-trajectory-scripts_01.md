# sysid validation用の別軌道取得スクリプト（exp_008/exp_009）を追加

## 冒頭メタ情報

- 日時: 2026-08-20（時刻未記録）
- 対象ファイル:
  - `my_ak45/Mujoco/data_collection/exp_008_validation_trajectory.py`（新規）
  - `my_ak45/Mujoco/data_collection/exp_009_validation_trajectory_randomized.py`（新規）
  - `my_ak45/control_mit_can/config.yaml`（`experiment.trajectory_randomized` セクション追加）
- 種別: 機能追加

## 設計判断と理由

`docs_syid/AK45-36_sysid_作業手順.md` フェーズ4 項目17は「sysidに使った軌道（multi-sine開
ループ励振）と同じランでしかleave-one-run-out交差検証ができていない」という未完了点を残して
おり、項目19で「別軌道（PD制御・インピーダンス制御）のデータをPiで取得する」ことを要求していた。

### 1. exp_008: `exp_004_trajectory.py` と `exp_005_sysid_excitation.py` の組み合わせ

制御則としては `exp_004_trajectory.py`（インピーダンス制御による三角波位置追従）がそのまま
項目19の要件だが、そのままでは使えない2点があった:
- 出力先が `my_ak45/control_mit_can/logs/`（`.gitignore` 対象）で、Windows PC側にデータが
  渡らない。
- `identification/csv_adapter.py` が時刻軸として優先する `wall_time` 列を記録していない。

そこで exp_004 の制御ロジックはそのまま流用し、出力先・ロギング方式のみ `exp_005_sysid_excitation.py`
の作法（`my_ak45/Mujoco/data/raw/` へ直接保存、`wall_time` 付きの自前CSVロガー）に合わせた
新規スクリプトとして実装した。既存の `exp_004_trajectory.py` 自体は変更していない（control_mit_can
側の既存ワークフローに影響を与えないため）。

**`torque_column="output_torque"` を使う前提であることを明記**: このデータは閉ループ（位置制御）
のため、exp_005のような明示的な指令トルク値が存在しない（`desired_pos` は位置目標であり
`desired_torque` ではない）。MuJoCo側の1関節モデルはトルク入力しか受け付けないため、
`csv_adapter.build_sequences()` を使う際は `torque_column="output_torque"`（実測トルク）を
指定する必要がある。フェーズ3で「`desired_torque` の方がフィットが良い」という知見があったのは
開ループ実験（multi-sine励振）の話であり、ここでは選択肢自体がないため影響しない。

### 2. exp_009: ランダム化・複数試行の連続取得

項目19は「条件を変えた追加データ（振幅違い等）もここで検討する」とも要求しており、
config.yamlを手で書き換えて exp_008 を何度も実行し直す運用も考えられたが、以下の理由で
「1回の実行で複数試行を連続取得する」専用スクリプトを別途作成した:

- **却下案（手動で複数回exp_008を実行）**: 実行のたびに人手でconfig.yamlを編集する必要があり、
  条件の記録漏れ・タイポのリスクがある。また試行間の条件（振幅・周期・K・B）の対応関係を
  別途メモしておかないと、後で「どのCSVがどの条件か」を追えなくなる。
- **採用案**: `config.yaml` の新セクション `experiment.trajectory_randomized` に範囲
  （`amplitude_range`/`period_range`/`K_range`/`B_range`）と試行回数・乱数シードを持たせ、
  1回の実行内で `random.Random(seed)` により毎回同じ組み合わせ列を再現可能な形でサンプリングし、
  試行ごとに独立したサブフォルダ（`trial_00/log.csv` 等）へ保存。実際にサンプリングされた
  パラメータは実行フォルダ直下の `manifest.csv` に記録し、CSV自体には残らないK/Bの値を
  後から追跡できるようにした。

**トルク安全マージンをconfig.yamlではなく定数にした理由**: インピーダンス制御はトルクを
clampしないため、振幅とKの組み合わせによっては試行開始直後の追従誤差（最大で振幅相当）に対して
`K * amplitude` が `safety.max_torque` を大きく超えるコマンドになりうる。サンプリング後に
`clamp_amplitude_for_torque()` で `K * amplitude` が `max_torque` の80%を超えないよう振幅を
自動的に切り詰める。この80%というマージン係数（`TORQUE_SAFETY_MARGIN`）は、
`exp_005_sysid_excitation.py` の `HARMONIC_RATIOS` と同じ考え方で「sysid手法とは無関係な
安全パラメータなので誤って変更されないようconfig.yamlではなくコード側の定数として持つ」方針を
踏襲した。なお、これはあくまでソフト側の事前制限であり、実測トルクに基づく最終防御は従来通り
`SafetyMonitor` が担う。

**緊急停止時は以降の試行を中止する設計**: `SafetyMonitor.trigger_emergency_stop()` が
呼ばれた時点で全モーターの電源が落ちているため、後続の試行を続けても意味がない。1試行が
`emergency_stop` または `KeyboardInterrupt` で終わった場合、それまでの結果を `manifest.csv` に
記録した上でループを打ち切る。

## 未対応・既知の課題

- **負荷変更は未対応**: 項目19が言及する「負荷変更」に対応するconfig.yaml項目は存在しない。
  現在のセットアップ（exp_004/008/009共通）は出力軸に付加質量のない状態を前提にしており、
  負荷ありデータを取るには物理的に錘・アームを取り付ける必要がある。その場合、
  `models/ak45_36_joint.xml`（素の出力軸を想定した慣性設定）も合わせて見直しが必要になる。
- **「正逆両方向」は三角波の往復に暗黙に含まれるのみ**: 三角波軌跡は `-amplitude → +amplitude
  → -amplitude` と1回の試行内で両方向を通るため追加対応はしていないが、往路・復路を明示的に
  分離して比較する検証（摩擦の方向依存性の確認等）はできていない。
- **初回実行時にブランチ違いで停止（原因究明済み・再実行で解消確認済み）**: ユーザーが実機
  （Raspberry Pi）で `exp_009` を初回実行した際、シェルが（誰の操作か不明だが）`master` ではなく
  `mujoco-sysid` ブランチにチェックアウトされていたため、`SafetyMonitor.update_and_check()`
  （`master` にのみ存在する、`actuator-control-implementation-nwyd4g` マージ由来のメソッド）が無く
  `AttributeError` で試行0の途中に停止した（この失敗試行の空フォルダは削除済み、コミットに含めない）。
  `master` ブランチへ切り替えて再実行したところ、5試行すべて `completed`（緊急停止なし、
  最大追従誤差0.08〜0.97rad、`SoftRealtimeLoop` の実測 avg error 0.000ms/stddev 0.002ms）で
  正常完了した（`my_ak45/Mujoco/data/raw/exp009_validation_trajectory_randomized_1787187421/`
  としてコミット）。
- exp_008/exp_009 とも `sysid_run_check.py` によるPASS/WARN/FAIL自動検証を組み込んでいない
  （同スクリプトの検証ロジックはmulti-sine開ループ励振の周波数応答解析等を前提としており、
  閉ループの位置追従軌道にはそのまま適用できないため）。取得データの品質確認は現状、
  実行中のログ表示（追従誤差・温度）と `identification/validate.py` 相当の事後検証に委ねている。

## テスト状況

- [ ] 単体テスト実行（該当なし。`my_ak45/wire_mechanism/` 以外に自動テストスイートはない）
- [ ] 統合テスト実行
- [x] 手動確認: `ruff check`・`python -m py_compile` 通過、`config.yaml` のYAML読み込み確認
      （`experiment.trajectory_randomized` セクションが期待通りパースされることを確認）
- [x] 実機検証: `master` ブランチで `exp_009_validation_trajectory_randomized.py` を実行し、
      5/5試行が緊急停止なく `completed`（`exp009_validation_trajectory_randomized_1787187421/`）。
      初回実行時のブランチ違いによる `AttributeError`（上記「未対応・既知の課題」参照）は
      `master` への切り替えで解消を確認
- [ ] レグレッションテスト
