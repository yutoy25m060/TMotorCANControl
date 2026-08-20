# 別軌道（exp_009）でのsysid検証スクリプト `validate_trajectory.py` を追加

## 冒頭メタ情報

- 日時: 2026-08-20 （作業手順書 フェーズ4.5 項目20a）
- 対象ファイル:
  - `my_ak45/Mujoco/identification/validate_trajectory.py`（新規）
  - `my_ak45/Mujoco/identification/csv_adapter.py`（`build_sequences` に `skip_time` 追加、`n_step` の算出元を変更）
  - `my_ak45/Mujoco/identification/validate.py`（`rollout_errors` に `common_grid` オプション追加）
  - `my_ak45/Mujoco/identification/results/validation_trajectory_fit-{desired,output}_torque_seg0.5s_shift1/`（新規・結果）
  - `my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`（項目17・20a を更新）
  - `my_ak45/Mujoco/README.md`（フェーズ4の節・実行コマンド・ディレクトリ図を更新）
- 種別: 機能追加

## 設計判断と理由

### 1. `validate.py` の拡張ではなく新規スクリプトにした

作業手順書の項目20aは「`validate.py` への `--trajectory` オプション追加」も選択肢として
挙げていたが、次の理由で新規ファイルにした。

- `validate.py` の中心は **leave-one-run-out**（同定ラン集合から1本抜く）という制御構造で、
  「同定は exp_005 の3ラン固定・評価は exp_009 の5試行」というこちらの構造と噛み合わない。
  同じ関数に両方を通すと `held_out` が意味を持つ場合と持たない場合ができ、分岐だらけになる。
- 出力（結果テーブルの列・per-trial の内訳・manifest 由来の条件表示）も別物になる。
- 一方で `build_model()` / `rollout_errors()` / `BASELINE` は完全に共通なので、
  `validate.py` から import して重複を避けた（新規ファイル側に再実装はしていない）。

### 2. 共有コード（`csv_adapter.py`）へ手を入れた2点

新規ファイルだけでは成立しなかったため、共有側に最小限の変更を入れている。
**どちらも exp_005（1kHz）の既存結果を変えないことを実測で確認済み**
（ステージ3再走で `n_segments=48` / `cost_after=0.21574950777899846` /
armature `0.012749758…` が既存 `results/stage3_.../summary.txt` と完全一致）。

- **`n_step` を `model.opt.timestep` ではなくCSV自身のサンプル周期から決めるよう修正。**
  `n_step` はCSVの**行数**として使われるのに、モデルのタイムステップ（1ms）から
  計算していた。1kHzで記録した exp_005 では偶然一致するので今まで問題にならなかったが、
  100Hzで記録した exp_009 では区間長が10倍（0.5秒指定で5秒分）になってしまう。
  これは元コードのバグであり、サンプル周期を使うのが本来の意味。
- **`skip_time` 引数を追加（既定0＝従来どおり）。**
  既存の `startup_trim_time()` は multi-sine 励振の起動過渡（速度が飽和域近くまで乗る）を
  速度しきい値3.815 rad/s で検出する仕組みで、exp_009 の「ゼロ位置から三角波の始点
  （-amplitude）へ飛びつく」過渡は試行によって引っかかったり引っかからなかったりする
  （5試行の |v|max は 1.27〜5.12 rad/s とばらつく）。全試行を同じ扱いにするため、
  呼び出し側が既知の助走時間（0.5秒）を明示的に渡せるようにした。
  なお誤差への影響自体は小さい（skip=0/0.5/1.0 で平均62.2/63.3/63.4 mrad）ので、
  これは「数字を良くするため」ではなく「扱いを揃えるため」の変更である。

### 3. `rollout_errors` に `common_grid` オプションを足した

既定の `TimeSeries.resample(target_dt=...)` は区間の実時間長 `span` を保ったまま
`ceil(span/dt)+1` 点へ等分するため、区間ごとに `span` が違うとステップ数が1つずれる。
1kHzのexp_005ではずれないが、100Hzの exp_009 では 490/491 とばらつき、
`sysid_rollout` が要求する「全区間で同形状の制御配列」を満たせず `ValueError` になった。

- **却下案（`rollout_errors` を常に共通格子にする）**: 既定挙動を変えると項目17で
  報告済みの数字（位置RMS 11.58 mrad 等）が再現しなくなる恐れがある。検証スクリプトの
  数字は作業手順書とREADMEに載っており、再現性を壊す変更は割に合わない。
- **採用案**: `common_grid=False` を既定にして既存経路は一切変えず、
  100Hzデータを扱う `validate_trajectory.py` からのみ `True` を渡す。
  `True` のときは最短区間に合わせた `arange(n)*dt` 上へ全区間を再標本化する
  （タイムステップちょうどの格子なので、実時間長を保つ既定より時刻の扱いは素直）。

### 4. `shift`（実測列の前詰め行数）を1にした

`shift` はCSVの**行数**単位なので、1kHzで求めた `DEFAULT_SHIFT=2` を100Hzデータに
そのまま使うと20ms分ずらす過補正になる。100Hzでの内訳は

    記録の帳簿上のずれ 1行 = 10ms
    ＋ 電流ループのむだ時間 約1.9ms
    − sysid_rollout の時刻付けによる暗黙の補正 1モデルステップ = 1ms
    ≒ 10.9ms ≒ 1.09行

で1行が妥当。実測でも shift=0/1/2/3 の平均位置RMSが 65.8/**63.3**/63.7/64.8 mrad と
1で最小になり、勘定と一致した（ただし感度は弱く、どれを選んでも結論は変わらない）。

### 5. 入力トルク列が同定側と揃わない件を「両方走らせる」で処理した

exp_009 は閉ループなので指令トルク値が存在せず、`output_torque`（実測）を使うしかない。
実測トルクは `Kt_actual` の既知の誤り（公式0.11 Nm/A に対し約+10%）を直接受けるため、
同定を `desired_torque` で行うと全ステージに共通のスケール誤差が乗る。

- **却下案（同定側も `output_torque` に固定する）**: それでは「実際に採用した
  パラメータ（`desired_torque` 同定・`models/ak45_36_joint_identified.xml` に焼き込み済み）」
  が別軌道で通用するか、という本来の問いに答えられない。
- **採用案**: `--fit-torque` で切り替えられるようにして両方を実行し、
  **ステージ間の順位が変わらないこと**を確認した（ステージ2→3の改善倍率は
  `desired_torque` 同定で 7.1倍→9.4倍、`output_torque` 同定で 6.7倍→8.2倍）。
  トルクスケール誤差は全ステージに共通に効くので順位を歪めない、という想定が裏付けられた。

### 6. 区間の切り出しは既存と同じ「速度ゼロ交差」を流用した

三角波追従では真のゼロ交差は軌道の頂点（周期あたり2回）しかなく、区間数が少なくなる
（試行3で5区間）。一様分割にすれば区間数は稼げるが、区間先頭の初期速度に実測ノイズが
そのまま乗る（速度が大きい点で切ると誤差の影響が大きい）。ゼロ交差で切れば初期速度が
ほぼ0なのでこの影響が小さく、かつ **`validate.py` と同じ切り出し規則**になるため
手法の差ではなくデータの差だけを見ていることが担保できる。区間数の少なさは
per-trial の内訳表に「区間数」列を出して読み手が判断できるようにした。

## 未対応・既知の課題

- **ステージ2と3の優劣が試行の速さで割れる。** 速い試行（周期2秒台）はステージ3が圧勝、
  遅い試行（周期5〜6秒台）はステージ2の方が良い（位置RMS: 試行1で 54.2 vs 144.7 mrad、
  試行4で 20.8 vs 88.0 mrad）。ステージ3は摩擦の一部をクーロンから粘性へ移す解
  （`frictionloss` 0.160→0.098、`damping` 0→0.027）なので、粘性項が効かない低速域では
  クーロン摩擦を過小評価する。**「ステージ3採用」という結論は平均値ベースであり、
  低速域を主に扱う用途では再検討が要る。** 本来は Stribeck 等の速度依存摩擦モデルが
  必要だが、MuJoCoの関節パラメータにはその表現がないため今回のスコープ外とした。
- **残差の絶対値が大きい**（位置RMSが軌道の振れ幅の約13%）。項目17の11.58 mradとは
  条件が違うので直接比較できないが、閉ループ・100Hz・実測トルク入力という条件では
  この程度しか合わないということ自体は事実として残る。切り分け（モデルの限界か、
  `Kt_actual` の誤差か、100Hzサンプリングの粗さか）はできていない。
- **`Kt_actual` の誤り（公式比+10%）は未修正のまま。** これを直すと `output_torque` の
  スケールが変わるため、本検証の絶対値も同定値も動く。`src/TMotorCANControl/mit_can.py`
  の `MIT_Params["AK45-36"]["Kt_actual"]` に手を入れる話であり、実機の全実験結果に
  影響するので単独の課題として切り出すべき。
- **exp_009 の5試行はすべて無負荷・同一姿勢。** 項目19の「負荷変更」は未実施のままで、
  慣性が変わったときにこのパラメータ組が通用するかは分かっていない。
- `identification/results/figures/` の図は交差検証（`validate.py`）の結果しか描いておらず、
  別軌道検証の可視化（`plot_results.py` への追加）はしていない。

## テスト状況

- [ ] 単体テスト実行（該当なし。`my_ak45/wire_mechanism/` 以外に自動テストスイートはない）
- [x] 統合テスト実行: `uv run --group mujoco-sysid python validate_trajectory.py` を
      `--fit-torque desired_torque` / `output_torque` の両方で完走。
      結果は `results/validation_trajectory_fit-*/summary.txt` と `trials.json` に出力
- [x] リグレッションテスト: `csv_adapter.py` の変更が exp_005（1kHz）の同定結果を
      変えないことを確認。ステージ3の再走で `n_segments=48` /
      `cost_before=15.949764128439249` / `cost_after=0.21574950777899846` /
      armature `0.012749758164055498` / frictionloss `0.097735278042639` /
      damping `0.02701614746069557` が既存の
      `results/stage3_desired_torque_seg0.5s_shift2_off0/summary.txt` と完全一致。
      `validate.py` の `rollout_errors` は既定 `common_grid=False` で従来経路のまま
- [x] 手動確認: `uv run ruff check my_ak45/Mujoco/identification/` 通過。
      `shift` を 0/1/2/3、`skip` を 0/0.5/1.0 で振って感度を確認し、既定値の根拠を
      コード内コメントに記録
- [ ] 実機検証（該当なし。PC側の解析スクリプトのみで、CANバスを使わない）
