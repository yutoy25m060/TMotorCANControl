---
date: 2026-08-07 14:30 JST
type: 機能追加
slug: phase-bc-visualization
files:
  - my_ak45/wire_mechanism/plotting.py (新規)
  - my_ak45/docs_mechanism/ワイヤー駆動関節の運動学と定滑車配置の検討.md (C-2 セクション更新)
  - pyproject.toml (matplotlib >= 3.8.0 追加、requires-python を 3.9+ に更新)
  - .claude/rules/change-log-format.md (参照のみ)
---

# フェーズB/C 可視化機能の実装

## 設計判断と理由

### 実装対象
- **JavaScript Interactive Dashboard** (Web artifact)
  - リアルタイムなパラメータ調整とプロット
  - ブラウザで即座に動作（追加インストール不要）
  - [Artifact URL](https://claude.ai/code/artifact/b9b9d957-8f93-4ec3-b0d9-7e74587a1b87)

- **Python/matplotlib Plotting Module** (CLI output)
  - 静的 PNG ファイル出力
  - ドキュメント化・報告書作成用
  - 数値重視の研究者向け

### 技術選択
1. **matplotlib 選択理由**：
   - NumPy と同じメモリレイアウトで効率的
   - wire_statics.py の計算結果を直接プロット可能
   - 出力形式の自由度（PNG, PDF, SVG 等）
   - 学術論文作成時の標準ツール

2. **Python 3.9+ への昇格**：
   - matplotlib 3.8+ は Python 3.9 以上必須
   - pyproject.toml の requires-python が >=3.8 だったが、実際のデバイス（Raspberry Pi 5 等）は概ね 3.10+
   - 実装の複雑度軽減（3.8/3.9 分岐の回避）

3. **dev 依存化**：
   - CLI での plotting は開発時のみ使用（実車制御には不要）
   - ユーザーが必要に応じてインストール可能

### 検討した代替案
| 案 | メリット | デメリット | 判定 |
|----|---------|-----------|------|
| PyScript (Pyodide) | ブラウザのみで動作、インストール不要 | 起動遅い、NumPy ロード時間 | ❌ Dashboard との役割重複 |
| Plotly (Python) | 対話的HTML生成 | ファイルサイズ大、静的出力に向かない | ❌ matplotlib で十分 |
| Seaborn + matplotlib | より高度なスタイリング | 追加依存、本用途では過度 | ❌ matplotlib で十分 |
| gnuplot | 軽量、従来的 | Python と疎結合、学習コスト | ❌ Python 統合推奨 |

### トレードオフ
- **matplotlib**: 13MiB + 依存パッケージ約 20MiB のサイズ増
  - 代わりに：完全な Python 統合、学術論文品質の図
  - 対象ユーザーが「解析担当者」なので許容可能

---

## 実装内容

### 1. `wire_mechanism/plotting.py`

```python
def plot_wire_geometry_phase_bc(
    x: float,
    z: float,
    l_anchor: float,
    theta_anchor_offset: float,
    mass: float,
    l_com: float,
    g: float = 9.8,
    theta_range: tuple = (-π/2, π/2),
    num_points: int = 200,
    l_moment_arm_min: float = 1e-4,
    output_file: str | None = None,
) -> tuple
```

**機能**:
- フェーズB: `l_wire(θ₀)`, `l_moment_arm(θ₀)` プロット
- フェーズC: `τ_gravity(θ₀)`, `T(θ₀)` プロット
- 可行性判定を色分け（紫=可行, 赤=不可行）
- スカラー/配列入力をサポート（NumPy ufunc パターン）

**実装細節**:
- `solve_wire_geometry()` と `solve_wire_tension()` を直接呼び出し
- matplotlib backend 自動切り替え（ヘッドレス環境対応）
- 4パネル subplot レイアウト
- 日本語ラベル対応（Unicode）

### 2. ドキュメント更新（C-2 セクション）

**追加内容**:
- 対話的ダッシュボード URL 埋め込み
- Python モジュール使用例コード
- パラメータ範囲説明
- CLI 実行例

---

## 未対応・既知の課題

### スコープ外
1. **Phase D 統合**
   - グリッド探索時の matplotlib バッチ出力は D-3 実装時に追加
   - 現在は単一パラメータセット向け

2. **3D 可視化**
   - ワイヤー経路の 3D 描画は要検討
   - 2D 平面（x-z）に限定（y=0 仮定）

3. **対話的調整**
   - Python CLI では静的出力のみ
   - リアルタム調整はダッシュボード使用

### 既知制限
- `l_wire` の完全退化点（l2=l3, θ3=0）では NaN 描画
  - 物理的に不可能な領域なので表示として問題ない
- matplotlib の日本語フォント未設定時は `□` 表示
  - ユーザー環境で matplotlib.rcParams['font.sans-serif'] 設定が必要

---

## テスト状況

### ✅ 実装テスト
- [x] モジュール import 確認
- [x] 標準パラメータでの plotting 実行
- [x] PNG ファイル出力確認
- [x] 3 パターンの異なるパラメータセットで動作確認
  - パターン1: 標準的な下側定滑車 (x=0.2, z=-0.15)
  - パターン2: 上側定滑車 (x=0.1, z=0.1)
  - パターン3: 広い可動域での符号反転 (x=0.3, z=-0.2, θ ∈ [-π, π])

### ✅ ドキュメント
- [x] C-2 セクションにダッシュボード URL 記載
- [x] Python 使用例を完全なコード例として提示
- [x] CLI 実行例を記載

### ✅ 依存関係
- [x] pyproject.toml に matplotlib 追加
- [x] requires-python を 3.9+ に更新（matplotlib 互換性）
- [x] uv sync で正常にインストール確認

### ⚠️ 未実施（スコープ外）
- ハードウェア検証（plotting は純計算、CAN 非依存）
- CI テスト（pytest 統合は D フェーズ）

---

## 関連する変更

- **コミット**: このファイル作成時点で git add なし（作業ツリーのみ）
- **リンク**: ダッシュボード Artifact の長期可用性は Claude インフラに依存
  - Local copy 必要な場合は `.ai/artifacts/phase_bc_dashboard_backup.html` 形式で保存推奨

---

## 次のステップ

1. **フェーズD**: グリッド探索時の matplotlib integration
   - ヒートマップ出力（最適値の可視化）
   - 制約違反領域のマスキング

2. **オプション機能**
   - Plotly による interactive HTML 出力（matplotlib の代替）
   - SVG 出力（ドキュメント埋め込み対応）

3. **日本語フォント対応**
   - `.ai/config/matplotlibrc` で font.sans-serif を設定
   - 'IPAGothic' 等の日本語フォント指定

