# AK45-36 × MuJoCo sysid toolbox 作業手順

## 前提・役割分担

実機（CAN通信）はRaspberry Piでしか扱えないため、MuJoCo sysid の最適化計算は
Windows PC（Piをリモート操作している側の母艦）で行う。このため作業を次のように分担する。

> **補足（当初想定の訂正）**: 当初はPC側を使う理由を「GPUが使えるから」としていたが、
> sysid最適化にGPUは不要。最適化は有限差分ヤコビアンによる Gauss-Newton/LM であり、
> 各パラメータ摂動のロールアウトは `mujoco.rollout` が**CPUスレッド並列**で実行する
> （公式ノートブックのGPU設定はColab上での動画レンダリング用）。
> PC側で行う実質的な利点は「CPUコア数が多い」「Piの実機制御を止めずに解析できる」の2点。

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
- [x] `sysid_run_check.py` による自動検証を実装、`exp_005_sysid_excitation.py` の実行末尾に
      組み込み済み（実験直後にPASS/WARN/FAILが自動で出る。詳細はフェーズ1参照）
- [x] 振幅0.9Nm・コマンド送信順序修正・起動過渡対応（duration 10.25s化）を反映した実機データ
      （`exp005_sysid_excitation_1786559877`/`_1786560014`）取得・コミット済み
- [ ] 今後Piで実行して得られた `log.csv`/`console.log` をコミットし、Windows PC 側で `git pull`

## 作業手順

### フェーズ1: 実機データ取得 【Pi】

1. [x] `config.yaml` の `experiment.sysid_excitation` を実機データで検証・調整済み
       （`amplitude`: 1.5→0.9Nm、`duration`: 10.0→10.25s。根拠は
       `.ai/logs/2026-08-13_01_*`・`2026-08-13_02_*` 参照）
2. [ ] `cd my_ak45/Mujoco/data_collection && python exp_005_sysid_excitation.py` を
       目視監視のもとで実行する（初回。純トルク指令のためフィードバックによる復元力がない）
3. [x] 記録された `log.csv` は `sysid_run_check.py` により自動検証される（実行末尾で自動実行、
       11項目をPASS/WARN/FAILで判定）。FAILが出た場合はsysidに使わないこと。手動で個別に
       確認したい場合は `python sysid_run_check.py <log.csvのパス>` を単体実行できる
4. [ ] 必要なら振幅・周波数を変えて複数回実行する（識別可能性を上げるため、後段で
       条件を変えた複数試行が要る可能性が高い — フェーズ4参照）
5. [ ] 採用する試行のフォルダをコミットしてWindows PC側で `git pull`
       （不要な試行は `my_ak45/Mujoco/data/raw/` から削除してからコミットする）

### フェーズ2: MuJoCo最小モデルの作成 【Windows PC】

6. [ ] Windows PC に環境をインストール: `pip install -r my_ak45/Mujoco/requirements.txt`
       - `mujoco[sysid]`（= `mujoco.sysid` モジュール）は **mujoco 3.5.0 以降**でのみ提供される。
         旧来の `mujoco==3.3.4` ピンには理由の記録がなく、3.5.0台への引き上げに支障がなかったため
         `requirements.txt` 自体を `mujoco[sysid]>=3.5.0` に更新して統合済み（別ファイルへの分離は撤回）
       - 公式ノートブックにある `--pre -f https://py.mujoco.org/` は現在不要
         （sysid は通常のPyPI安定版に取り込み済み）
       - GPU関連パッケージはsysid最適化には不要（上記「前提・役割分担」の補足を参照）
7. [ ] AK45-36 用の単一ヒンジ + トルクアクチュエータの最小XMLモデルを新規作成する
       （`docs_syid/sysid_mujoco_vscodeへの移設コード途中.py` のARM_XML/SPRING_MASS_XMLが
       雛形になる）。`GEAR_RATIO=36.0` など `MIT_Params["AK45-36"]`（`src/TMotorCANControl/mit_can.py`）
       の値を初期値の参考にする
8. [ ] 位置・速度センサーをXMLに定義する

### フェーズ3: 同定パラメータの定義と最適化 【Windows PC】

> **重要（2026-08-13の分析で判明した制約）**: 10秒通しを1本の `ModelSequences` として
> そのまま同定に使ってはいけない可能性が高い。同一励振を独立に2回実行した2ラン
> （`exp005_sysid_excitation_1786559877`/`_1786560014`、指令トルクは完全に同一）を比較した
> ところ、速度の差は時間によらずほぼ一定（0.10〜0.14 rad/s、測定ノイズ）だったのに対し、
> **位置の差は時間とともに増大**し t=10sで可動範囲(0.89rad)の19%（0.168 rad）に達した。
> これは開ループ系（フィードバックによる復元力がない）特有の初期状態鋭敏性であり、
> 実機自身がこの量だけ非決定的にばらつく以上、MuJoCoの残差がこれより小さくなることはない。
> 軌道を長さLの区間に分割し各区間先頭で位置・速度をそろえ直した場合の終端位置ずれ
> （＝原理的に再現しきれない量の目安）は L=10s で可動範囲比61%、L=2sで21.7%、L=1sで8.8%、
> L=0.5sで4.5%と急減する。詳細は `.ai/logs/2026-08-13_02_startup-transient-and-auto-check_01.md`
> 参照。

9. [ ] 実機CSVを**0.5〜1秒程度の区間に分割**し、`sysid.ModelSequences` に複数シーケンスとして
       渡す（公式ノートブック4節が推奨する「複数軌道による識別可能性向上」と同じ発想。
       分割せず10秒通しで1シーケンスにすると、上記の再現性の限界に埋もれてフィットが
       意味をなさない可能性が高い）
10. [ ] 各区間の切り出し点は**速度ゼロ交差付近**を選ぶ（初期速度をほぼゼロとして扱えるため。
        今回のデータでは t=0.2024s/0.2976s 付近に交差がある。位置は有限差分と一致しており
        信頼できるが、速度は瞬時値でノイズが乗るため、局所多項式フィット等での平滑化は
        励振の最高調波（29.6Hz、周期34ms）を潰してしまい有効な対策にならない点に注意）
11. [ ] 2ラン（`_1786559877`/`_1786560014`）とも投入する（初期状態の誤差は独立なので
        系統的な偏りになりにくい）
12. [ ] 切り出し点を複数パターン試し、同定されるパラメータが一致するか確認する
        （一致すれば初期状態の懸念は経験的に否定できる）
13. [ ] 最初は `armature`（反射慣性）1つだけを同定対象にする（複数同時は識別不能になりやすい）
14. [ ] `my_ak45/Mujoco/data/raw/` のCSVを `sysid.TimeSeries` に変換するアダプタスクリプトを書く
        （CSV列名 `output_angle`/`output_velocity` 等とXMLのセンサー名を対応づける。
        上記の区間分割もここで行う）
15. [ ] `ParameterDict` → `build_residual_fn` → `sysid.optimize` を実行し、収束（残差の低下）を確認する
16. [ ] 収束したら `damping` を追加するなど、対象パラメータを段階的に増やす

### フェーズ4: validation 【Windows PC、必要ならPiで追加データ取得】

17. [ ] sysidに使った軌道とは別の実機データ（例: PD制御での位置追従、
        `1_template_impedance.py` 等の既存インピーダンス制御ログ）で、同定後パラメータが
        実測とシミュレーションで一致するか確認する
18. [ ] パラメータが境界値に張り付いていないか、複数解の曖昧さがないかを確認する
19. [ ] 識別不能・不安定な場合は、条件を変えた追加データをPiで取得する
        （負荷変更・正逆両方向・振幅違いなど）→ フェーズ1に戻る

### フェーズ5: 反映

20. [ ] 同定結果をMuJoCoモデル（XML）に反映し、`my_ak45/Mujoco/` 配下に保存・コミットする
21. [ ] 変更履歴を `.ai/logs/` にCLAUDE.mdの規定フォーマットで記録する
        （このsysid作業自体はメイン package (`src/TMotorCANControl/`) に影響しないため、
        CLAUDE.mdの「テスト状況」項目は「実機検証」の代わりに「sysid収束・validation結果」
        を記載する形になる）

## 未確定事項

- `my_ak45/Mujoco/data/raw/` のCSVはサンプルレート次第で1ファイルあたり数百KB〜数MB程度になりうる
  （1kHz×10秒＝10,000行）。試行数が増えた場合にリポジトリサイズへの影響を再検討する
- ~~Windows PC側の `mujoco[sysid]` インストール方法~~ → `requirements.txt` を
  `mujoco[sysid]>=3.5.0` + `pandas` に更新して確定。ただし uv/pip どちらで管理するかは未決定。
  また `>=3.5.0` は下限のみの指定であり、チュートリアルのAPIで実際に動作確認が取れた時点で
  そのバージョンに固定すること（sysid は新しいAPIのため変更されている可能性がある）
- フェーズ2のXMLモデルをどこまで詳細化するか（単一関節のみか、将来的な脚機構を見据えるか）は
  `my_ak45/docs_mechanism/`・`quadruped_prep_ja.md` の計画次第で変わりうる
- フェーズ3の軌道分割（0.5〜1秒区間、`ModelSequences` への複数シーケンス投入）は方針決定のみで
  実装は未着手。区間長の最適値（4.5%〜8.8%の再現不能ずれのどこで手を打つか）も未確定
- `sysid_run_check.py` のしきい値は現時点の実機データ2セット（飽和域・非飽和域）からの
  経験値であり、統計的に厳密な根拠はない。データ蓄積に応じて見直しが必要になりうる
