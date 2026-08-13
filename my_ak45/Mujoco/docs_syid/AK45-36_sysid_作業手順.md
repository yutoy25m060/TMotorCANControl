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

6. [x] Windows PC に環境をインストール（2026-08-13、方針変更）
       - **`requirements.txt`（pip）ではなく、メインパッケージと同じ `uv` 環境に統合した。**
         `pyproject.toml` の `[dependency-groups]` に `mujoco-sysid = ["mujoco[sysid]>=3.5.0"]`
         を新設し、`uv sync --group mujoco-sysid` でインストール済み（mujoco 3.11.0）。
         「未確定事項」にあった uv/pip の選択は uv に確定。
       - `my_ak45/Mujoco/requirements.txt`（RL/JAX-MJX/PyQt6等を含む統合版、pip用）は
         sysid作業には過剰なため今回は使わず、変更もしていない。他用途で必要になった場合はそちらを使う。
       - 公式ノートブックにある `--pre -f https://py.mujoco.org/` は不要だった
         （sysid は通常のPyPI安定版に取り込み済み。3.5.0以降 provides_extra に `sysid` あり）
       - GPU関連パッケージはsysid最適化には不要（上記「前提・役割分担」の補足を参照）
7. [x] AK45-36 用の単一ヒンジ + トルクアクチュエータの最小XMLモデルを作成した
       （`my_ak45/Mujoco/models/ak45_36_joint.xml`。`docs_syid/sysid_mujoco_vscodeへの移設コード途中.py`
       のARM_XMLが雛形）。`exp_005_sysid_excitation.py` のCSV列（`output_angle`/`output_velocity`/
       `output_torque`/`desired_torque`）が `TMotorManager_mit_can` 側で `GEAR_RATIO`/`Kt_actual`
       換算済みの出力軸側の値であることに合わせ、モデルもギアを挟まず出力軸側で直接トルクを
       受け取る1関節構成（`gear="1"`）にした。実験がアーム等の外部負荷を持たない素の出力軸の
       空転挙動（純トルク開ループ）であることから、worldbodyの質量・慣性はプレースホルダーの
       極小値とし、実体の慣性は同定対象となる `joint armature` 側に持たせる設計にした
       （`damping`/`frictionloss` もフェーズ3方針どおりのプレースホルダー固定値）。
       `timestep=0.001` は実機サンプリング（`config.yaml` `dt`）に合わせた。
8. [x] 位置・速度センサーをXMLに定義した（`jointpos`/`jointvel`、センサー名をCSV列名
       `output_angle`/`output_velocity` と一致させ、`sysid.TimeSeries.from_names()` の
       名前ベース自動マッピングがフェーズ3でそのまま使えるようにした）
       - `uv run python` で `spec.compile()` → `mujoco.mj_step()` の単発実行、および
         励振と同じ multi-sine トルクでの `mujoco.rollout.rollout()` →
         `sysid.TimeSeries.from_names()` の疎通を確認済み（`signal_mapping` が
         `output_angle`/`output_velocity` を正しく解決）

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

9. [x] 実機CSVを**0.5〜1秒程度の区間に分割**し、`sysid.ModelSequences` に複数シーケンスとして
       渡す。`identification/csv_adapter.py` の `segment_starts()`/`build_sequences()` で実装。
       既定の区間長0.5sで1ランあたり16区間、採用3ランで計48区間になる
10. [x] 各区間の切り出し点は**速度ゼロ交差付近**を選ぶ（初期速度をほぼゼロとして扱えるため。
        採用3ランでは起動過渡を切り捨てた後に81個の交差があり、最初の2つは
        t=0.203s/0.295s 付近（旧2ランの t=0.2024s/0.2976s とほぼ同じ）。位置は有限差分と
        一致しており信頼できるが、速度は瞬時値でノイズが乗るため、局所多項式フィット等での
        平滑化は励振の最高調波（29.6Hz、周期34ms）を潰してしまい有効な対策にならない点に注意）
11. [x] 採用3ラン（`_1786575616`/`_1786575633`/`_1786575782`、コミット `6e7214b`）を全て投入する
        （初期状態の誤差は各ランで独立なので系統的な偏りになりにくい）。
        **旧記載の2ラン（`_1786559877`/`_1786560014`）は `wall_time` 列を持たないため使わない**
12. [x] 切り出し点を複数パターン試し、同定されるパラメータが一致することを確認した。
        ステージ2で区間長・切り出し点を変えると `armature` は
        0.012461〜0.012719（**振れ幅±1%**）に収まる:

        | 条件 | 区間数 | armature | frictionloss |
        |---|---|---|---|
        | 区間0.5s・オフセット0（基準） | 48 | 0.012497 | 0.159819 |
        | 区間1.0s | 27 | 0.012719 | 0.144432 |
        | 交差オフセット3 | 48 | 0.012468 | 0.157860 |
        | 交差オフセット7 | 45 | 0.012461 | 0.157913 |

        → **開ループの初期状態鋭敏性が同定結果を歪めている懸念は経験的に否定できた**
        （`frictionloss` は 0.144〜0.160 と±5%動くので、こちらは `armature` ほど堅くない）
13. [x] 最初は `armature` 1つだけを同定対象にした（下記ステージ1）
14. [x] `identification/csv_adapter.py` を作成。`wall_time` を時刻軸に使い、起動過渡0.15sを
        切り捨て、速度ゼロ交差で区間分割する。センサー名と列名が一致しているため
        `TimeSeries.from_names()` の自動マッピングがそのまま使える。
        `wall_time` 列のない旧CSVは公称時刻 `t` へフォールバックし警告する。
        **実測列のずらし量は2行**（`identification/identify.py` の `DEFAULT_SHIFT`）。
        リポジトリ内に「1行」「3行」「2行」と3つの数字があるが、これらは矛盾ではなく
        別々の量である:

        | 数字 | 出どころ | 何を指すか |
        |---|---|---|
        | 1行 | `exp_005_sysid_excitation.py` | **記録の帳簿上のずれ**。`update()` が「状態を読む→送信」の順なので、CSV行kの実測値は行kの指令を送る*前*の状態 |
        | 約1.9行 | `sysid_run_check.py` の周波数応答 `L=1.82〜1.87ms` | **電流ループの物理的なむだ時間**。1kHzなので約2サンプル |
        | 3行 | `sysid_run_check.py` の相互相関ピーク | 上2つの**合計**（≈2.9）。1ms刻みの粗い測定で、一次遅れT≈1.25msの寄与も混じる |

        MuJoCo の `rollout` は `sensor[i] = ctrl[0..i-1] への応答` という、実機のロギング規約
        （＝1行目）と**まったく同じ**規約を持つ。したがって補正すべきは物理的なむだ時間だけで、
        約2サンプルになる。実際に振ると最終コストは
        shift=0:0.665 / 1:0.408 / **2:0.334** / 3:0.344 で2が最小となり、
        独立に測定された L=1.82〜1.87ms と直接一致した
15. [x] `identification/identify.py` で `ParameterDict` → `build_residual_fn` → `sysid.optimize`
        を実行し、収束を確認した（下記の表）
16. [x] `frictionloss` → `damping` と段階的に増やした。採用3ラン48区間・指令トルク入力・
        区間0.5s・shift=2 での結果:

        | ステージ | 同定対象 | コスト | armature | frictionloss | damping |
        |---|---|---|---|---|---|
        | 1 | armature | 43.43 → 15.77 | 0.015775 | (0固定) | (0固定) |
        | 2 | +frictionloss | 23.72 → **0.334** | 0.012497 | 0.159819 | (0固定) |
        | 3 | +damping | 15.95 → **0.216** | 0.012750 | 0.097735 | 0.027016 |

        - **`frictionloss` は必須**。ステージ1→2でコストが15.77→0.334と47倍改善しており、
          クーロン摩擦を入れないモデルは実機を全く再現できていない。参照手法（RS02）が
          frictionloss 0.163 を得ているのとも整合する
        - `damping` の追加による改善は1.5倍程度と小さく、`frictionloss` が 0.160→0.098 へ
          大きく動く。**両者は部分的に識別不能なペア**であり、`damping` の値
          （0.027）は単独では信用できない。参照手法も「有意な粘性摩擦は見えない」としている
        - 一方 `armature` はステージ2→3で 0.0125→0.0128（2%）しか動かず**安定している**。
          反射慣性に換算するとロータ慣性 ≈1.0e-5 kg·m²（= 0.0128/36²）で物理的に妥当

> **決着: MuJoCoへの入力は指令トルク（`desired_torque`）を使う**
>
> `.ai/logs/2026-08-13_01_*` で「モーター内蔵電流ループの定常ゲイン K=0.817・一次遅れ
> T=1.24ms をどう扱うか（(a) 実測トルクを入力にする / (b) 遅れをモデル化する）はフェーズ3で
> 決める」としていた未決事項。同条件で `output_torque`（実測トルク）を入力にした比較を行った:
>
> | 入力 | ステージ | コスト | armature | frictionloss |
> |---|---|---|---|---|
> | `desired_torque` | 1 | 15.77 | 0.015775 | — |
> | `output_torque` | 1 | 33.04 | 0.015658 | — |
> | `desired_torque` | 2 | **0.334** | 0.012497 | 0.159819 |
> | `output_torque` | 2 | 0.938 | 0.009624 | 0.200863 |
>
> - **実測トルク入力の方がフィットが明確に悪い**（ステージ2でコスト2.8倍）。`desired_torque` は
>   解析的に生成された滑らかな multi-sine であるのに対し、`output_torque` は電流計測から
>   換算されたノイズを含む信号であり、開ループ積分でその誤差が増幅されるためと考えられる
> - `armature` の比は 0.009624/0.012497 = 0.770 で、理論予測（K=0.817。指令トルク入力では
>   `armature` が 1/K 倍に膨らむ）とおおむね一致し、K の吸収という解釈は裏付けられた。
>   ただし `frictionloss` の比は 1.257 と逆向きで、単純な比例関係では説明できない
> - 指令トルク入力は参照手法・`exp_005_sysid_excitation.py` の設計意図・フェーズ4の
>   閉ループvalidation（制御器が出すのは指令トルク）のいずれとも整合するため、こちらを採用する。
>   **その代わり `armature`=0.0128 は真の反射慣性より約1/K=1.22倍過大**であり、
>   物理量として引用する際は注意すること（モデルとしては自己整合しているので、
>   同じ `Kt_actual` を使う制御コードをシミュレートする用途では問題ない）

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
- ~~Windows PC側の `mujoco[sysid]` インストール方法~~ →
  ~~`requirements.txt` を `mujoco[sysid]>=3.5.0` + `pandas` に更新して確定~~ →
  **2026-08-13に再確定**: `pyproject.toml` `[dependency-groups]` の `mujoco-sysid` として
  メインパッケージと同じ `uv` 環境に統合（`uv sync --group mujoco-sysid`）。
  `my_ak45/Mujoco/requirements.txt`（pip、RL/JAX-MJX/PyQt6等含む統合版）は変更せず残置。
  `>=3.5.0` は下限のみの指定であり、チュートリアルのAPIで実際に動作確認が取れた時点で
  そのバージョンに固定すること（sysid は新しいAPIのため変更されている可能性がある）
- ~~フェーズ2のXMLモデルをどこまで詳細化するか~~ → 2026-08-13時点では単一ヒンジ関節のみ
  （`my_ak45/Mujoco/models/ak45_36_joint.xml`）。将来的な脚機構を見据えた詳細化は
  `my_ak45/docs_mechanism/`・`quadruped_prep_ja.md` の計画次第で別途検討
- **`frictionloss` と `damping` は部分的に識別不能**（ステージ3で `damping` を足すと
  `frictionloss` が 0.160→0.098 に大きく動く）。どちらの値を採るかはフェーズ4のvalidationで
  別軌道に対する再現性を見て決める必要がある。`armature` はこの影響をほぼ受けない
- **指令-実測トルクの定常ゲイン K=0.817 の物理的原因は未特定**（`.ai/logs/2026-08-13_01_*` から
  継続）。現状は `armature` に吸収させているため、モデルは指令トルク入力に対しては正しく
  振る舞うが、`armature` を物理的な反射慣性として引用することはできない
- `identification/results/` の `report.html` は1ファイル約750KBあり、コード＋データから
  再生成できるため `.gitignore` で追跡対象外にしている（`params.yaml`/`summary.txt` は追跡する）
- フェーズ3の軌道分割（0.5〜1秒区間、`ModelSequences` への複数シーケンス投入）は方針決定のみで
  実装は未着手。区間長の最適値（4.5%〜8.8%の再現不能ずれのどこで手を打つか）も未確定
- `sysid_run_check.py` のしきい値は現時点の実機データ2セット（飽和域・非飽和域）からの
  経験値であり、統計的に厳密な根拠はない。データ蓄積に応じて見直しが必要になりうる
