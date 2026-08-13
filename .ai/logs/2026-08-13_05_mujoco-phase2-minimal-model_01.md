# フェーズ2: AK45-36用MuJoCo最小モデルを作成し、sysid環境をuv dependency-groupsに統合

## 冒頭メタ情報

- 日時: 2026-08-13（時刻未記録）
- 対象ファイル:
  - `my_ak45/Mujoco/models/ak45_36_joint.xml`（新規）
  - `pyproject.toml`（`[dependency-groups]` に `mujoco-sysid` を新設）
  - `uv.lock`（`uv sync --group mujoco-sysid` による更新）
  - `my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`（フェーズ2チェックリスト・未確定事項を更新）
- 種別: 機能追加

## 設計判断と理由

### 1. sysid環境のインストール方法をuv dependency-groupsに変更

作業手順書はこれまで「Windows PC側で `pip install -r my_ak45/Mujoco/requirements.txt`」を
想定しており、`requirements.txt`（RL/JAX-MJX/PyQt6等を含む統合版）とメインパッケージの
`uv` 管理環境（`pyproject.toml`/`uv.lock`）が別々に存在する状態だった。

- **採用した対応**: `pyproject.toml` の `[dependency-groups]`（既存の `dev` グループと同じ形式）に
  `mujoco-sysid = ["mujoco[sysid]>=3.5.0"]` を新設し、`uv sync --group mujoco-sysid` で
  メインパッケージと同じ `uv` 環境に統合した（mujoco 3.11.0 がインストールされ、
  `uv.lock` に反映済み）。ユーザーからの明示的な指示（「mujoco[sysid]のみpyproject.tomlに追加、
  基本Mujocoも今まで使っていたuv環境で動かしたい」）に基づく。
- **`requirements.txt` は変更しなかった理由**: 同ファイルはRL(gymnasium等)/MJX-JAX/PyQt6等、
  sysid作業には不要な依存を多数含む統合ファイルであり、ユーザーも「mujoco[sysid]のみ」を
  明示的に要望していたため、sysid用の最小依存だけを別グループとして追加する方が既存方針を
  崩さない。`requirements.txt` は他用途（pip専用環境での作業等）にそのまま残置する。
- **却下案**: `requirements.txt` の `mujoco[sysid]>=3.5.0` 部分を `pyproject.toml` の
  メイン `dependencies` に直接追加する案は、公開パッケージ（PyPI配布物）の実行時依存を
  肥大化させてしまうため見送った。`dependency-groups`（PEP 735、`dev` と同じ仕組み）は
  ビルド済みwheel/sdistのメタデータに含まれないため、この懸念がない。

### 2. AK45-36用の単一ヒンジ最小モデル

`docs_syid/sysid_mujoco_vscodeへの移設コード途中.py` のARM_XML（5自由度アーム、motor
アクチュエータ + armature + jointpos センサー）を単一関節に簡略化した
`my_ak45/Mujoco/models/ak45_36_joint.xml` を新規作成した。

- **単位系の判断**: `exp_005_sysid_excitation.py` が記録するCSV列（`output_angle`/
  `output_velocity`/`output_torque`/`desired_torque`）は `TMotorManager_mit_can` 側で
  `GEAR_RATIO`/`Kt_actual` 変換済みの出力軸側（post-gearbox）の値である
  （`mit_can.py` の `get_output_torque_newton_meters()`/`set_output_torque_newton_meters()`
  経由）。そのためモデル側でギア比を再度挟むと二重変換になる。`<motor gear="1">` として
  出力軸側トルク[Nm]をそのまま受け取る1関節構成にした。
- **質量・慣性の配置**: この実験はアーム等の外部負荷を持たない、素の出力軸の空転を記録した
  純トルク開ループ実験である。したがって物理的に意味のある慣性は「リフレクトされたロータ慣性」
  （`joint armature`、フェーズ3で同定する未知パラメータ）であり、worldbody側の
  `inertial mass`/`diaginertia` は将来的な外部負荷追加に備えた最小限のプレースホルダー
  （0.05 kg、対角慣性1e-5オーダー）とした。`damping`/`frictionloss` も同様に固定の
  プレースホルダー値とし、作業手順書フェーズ3の方針（「最初は armature のみを同定対象にする」）
  に委ねた。
- **timestep**: 実機サンプリング周期（`config.yaml` `experiment.sysid_excitation.dt=0.001`）に
  合わせて `0.001` にした。`sysid.TimeSeries` はリサンプリングに対応するため必須ではないが、
  揃えておけばフェーズ3のCSVアダプタでの補間が不要になる。
- **センサー名**: `jointpos`/`jointvel` のセンサー名をCSV列名（`output_angle`/
  `output_velocity`）と一致させた。`sysid.TimeSeries.from_names()` はセンサー名から
  信号マッピングを自動生成するため、フェーズ3のCSVアダプタで列名とセンサー名を
  手動対応付けする必要がなくなる。

## 未対応・既知の課題

- モデルの `armature`/`damping`/`frictionloss` の初期値（0.01/0.05/0.0）は、実測に基づかない
  暫定プレースホルダーである。フェーズ3で `Parameter` の `nominal`/初期 `value` を設定する際に
  上書きされる前提であり、このXML上の値自体に根拠はない。
- `joint range="-3.14 3.14"` は `config.yaml` `safety.max_position` を流用した安全側の目安であり、
  実機のメカストップ実測値ではない（AK45-36は連続回転が可能な構造の可能性があり、そもそも
  ハードな可動範囲制限が存在しない場合もある。要確認）。
- worldbodyの `geom`（`cylinder`）は可視化用の適当な形状であり、実機のジオメトリを反映していない。
- `mujoco-sysid` グループの `mujoco[sysid]>=3.5.0` は下限指定のみ。動作確認が取れたバージョン
  （今回は3.11.0）への固定は未実施（作業手順書の未確定事項に既存の記載あり）。
- フェーズ3（`ParameterDict`・`ModelSequences`・CSVアダプタ）は未着手。

## テスト状況

- [ ] 単体テスト実行（このリポジトリに自動テストスイートは存在しない。本変更もpytest対象外）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `uv sync --group mujoco-sysid` が成功し、mujoco 3.11.0 が既存 `uv` 環境にインストール
    されることを確認
  - `uv run python` で `mujoco.MjSpec.from_file()` → `spec.compile()` → `mujoco.MjData()` →
    `mujoco.mj_step()` が例外なく実行できることを確認（`sensordata`/`qpos` が取得できる）
  - 励振実験と同じ multi-sine（4.0/13.6/29.6 Hz、振幅0.9/0.54/0.27 Nm相当）トルクで
    `mujoco.rollout.rollout()` を1秒分実行し、`sysid.TimeSeries.from_names()` が
    `signal_mapping` として `output_angle`/`output_velocity` を正しく解決することを確認
  - `ruff check .` を実行し、本変更（`pyproject.toml`/XML/Markdown）に起因する新規エラーが
    ないことを確認（既存349件はすべて無関係な既存ファイルのもの）
- [ ] リグレッションテスト: 該当なし（実機を伴わないPC側の新規モデル作成のため）
