# AK45-36 × MuJoCo sysid toolbox 作業手順

## 前提・役割分担

実機（CAN通信）はRaspberry Piでしか扱えないが、MuJoCo sysid の最適化計算はGPUが使える
Windows PC（Piをリモート操作している側の母艦）で行う。このため作業を次のように分担する。

- **Pi 側**: 実機データ取得のみ（`my_ak45/control_mit_can/` の `lib/`・`config.yaml` を再利用）
- **Windows PC 側**: MuJoCoモデル作成・最適化・validation（`my_ak45/Mujoco/` 配下）
- **受け渡し**: リポジトリ（git）経由。ただし下記「データ共有の注意」を参照。

## データ共有の注意（重要・対応済み）

`my_ak45/control_mit_can/.gitignore` は `*.csv` / `*.log` をディレクトリ以下すべてに
適用しており、そこに実機データを置くと git追跡対象外になってしまう（意図的な設計 —
[README_ja.md](../../control_mit_can/README_ja.md) 参照）。手動コピーの手間・コピー漏れの
リスクを避けるため、`exp_005_sysid_excitation.py` を `my_ak45/Mujoco/data_collection/` に
移動し、出力先も `my_ak45/control_mit_can/logs/` ではなく `my_ak45/Mujoco/data/raw/`
（`.gitignore` が存在せず通常通り追跡される場所）に直接保存するよう変更済み。
モーター制御本体（`lib/`・`config.yaml`）は引き続き `my_ak45/control_mit_can/` のものを
`sys.path` 経由で再利用している。

- [x] `my_ak45/Mujoco/data/raw/` ディレクトリを作成
- [x] `exp_005_sysid_excitation.py` を `my_ak45/Mujoco/data_collection/` に移動、
      出力先を `my_ak45/Mujoco/data/raw/exp005_sysid_excitation_{タイムスタンプ}/` に変更
- [ ] Pi で実行して得られた `log.csv`/`console.log` をコミットし、Windows PC 側で `git pull`

## 作業手順

### フェーズ1: 実機データ取得 【Pi】

1. [ ] `config.yaml` の `experiment.sysid_excitation`（`base_freq`/`amplitude`/`duration`/`dt`）
       を確認・必要なら調整する（現状 base_freq=4.0Hz, amplitude=1.5Nm は暫定値）
2. [ ] `cd my_ak45/Mujoco/data_collection && python exp_005_sysid_excitation.py` を
       目視監視のもとで実行する（初回。純トルク指令のためフィードバックによる復元力がない）
3. [ ] 記録された `my_ak45/Mujoco/data/raw/exp005_sysid_excitation_*/log.csv` の波形を確認する
       （速度が乗りすぎていないか＝トルク-速度特性に入っていないか、動きが小さすぎて
       摩擦が見えていないか、を目視でチェック）
4. [ ] 必要なら振幅・周波数を変えて複数回実行する（識別可能性を上げるため、後段で
       条件を変えた複数試行が要る可能性が高い — フェーズ4参照）
5. [ ] 採用する試行のフォルダをコミットしてWindows PC側で `git pull`
       （不要な試行は `my_ak45/Mujoco/data/raw/` から削除してからコミットする）

### フェーズ2: MuJoCo最小モデルの作成 【Windows PC】

6. [ ] Windows PC に `mujoco[sysid]` をインストール（`my_ak45/Mujoco/requirements.txt` 参照。
       GPUレンダリング関連の項目はsysid最適化自体には必須ではない）
7. [ ] AK45-36 用の単一ヒンジ + トルクアクチュエータの最小XMLモデルを新規作成する
       （`docs_syid/sysid_mujoco_vscodeへの移設コード途中.py` のARM_XML/SPRING_MASS_XMLが
       雛形になる）。`GEAR_RATIO=36.0` など `MIT_Params["AK45-36"]`（`src/TMotorCANControl/mit_can.py`）
       の値を初期値の参考にする
8. [ ] 位置・速度センサーをXMLに定義する

### フェーズ3: 同定パラメータの定義と最適化 【Windows PC】

9. [ ] 最初は `armature`（反射慣性）1つだけを同定対象にする（複数同時は識別不能になりやすい）
10. [ ] `my_ak45/Mujoco/data/raw/` のCSVを `sysid.TimeSeries` に変換するアダプタスクリプトを書く
        （CSV列名 `output_angle`/`output_velocity` 等とXMLのセンサー名を対応づける）
11. [ ] `ParameterDict` → `build_residual_fn` → `sysid.optimize` を実行し、収束（残差の低下）を確認する
12. [ ] 収束したら `damping` を追加するなど、対象パラメータを段階的に増やす

### フェーズ4: validation 【Windows PC、必要ならPiで追加データ取得】

13. [ ] sysidに使った軌道とは別の実機データ（例: PD制御での位置追従、
        `1_template_impedance.py` 等の既存インピーダンス制御ログ）で、同定後パラメータが
        実測とシミュレーションで一致するか確認する
14. [ ] パラメータが境界値に張り付いていないか、複数解の曖昧さがないかを確認する
15. [ ] 識別不能・不安定な場合は、条件を変えた追加データをPiで取得する
        （負荷変更・正逆両方向・振幅違いなど）→ フェーズ1に戻る

### フェーズ5: 反映

16. [ ] 同定結果をMuJoCoモデル（XML）に反映し、`my_ak45/Mujoco/` 配下に保存・コミットする
17. [ ] 変更履歴を `.ai/logs/` にCLAUDE.mdの規定フォーマットで記録する
        （このsysid作業自体はメイン package (`src/TMotorCANControl/`) に影響しないため、
        CLAUDE.mdの「テスト状況」項目は「実機検証」の代わりに「sysid収束・validation結果」
        を記載する形になる）

## 未確定事項

- `my_ak45/Mujoco/data/raw/` のCSVはサンプルレート次第で1ファイルあたり数百KB〜数MB程度になりうる
  （1kHz×10秒＝10,000行）。試行数が増えた場合にリポジトリサイズへの影響を再検討する
- Windows PC側の `mujoco[sysid]` インストール方法・Python環境（uv/pipどちらを使うか）は未決定
- フェーズ2のXMLモデルをどこまで詳細化するか（単一関節のみか、将来的な脚機構を見据えるか）は
  `my_ak45/docs_mechanism/`・`quadruped_prep_ja.md` の計画次第で変わりうる
