# ワイヤー駆動機構の2Dアニメーション表示（GIF生成関数）を追加

## 冒頭メタ情報

- 日時: 2026-08-16 11:16
- 対象ファイル:
  - `my_ak45/wire_mechanism/plotting.py`（`animate_wire_mechanism()` /
    `animate_antagonistic_mechanism()` を追加）
- 種別: 機能追加（解析結果の可視化）
- ブランチ: `wire-mechanism-docs`

## 設計判断と理由

ユーザーから「解析結果を視覚的にわかりやすく見れるようにしたい」「2次元グラフ内でワイヤ駆動が
動作している様子を表示させたい」という依頼を受けた（参考として外部の機構アニメーションGIFを提示）。

### GIF生成をリポジトリ機能として実装した

- **代替案として検討し却下**: インタラクティブなGUIアプリ（PyQt等）を作る案。
  却下理由: CLAUDE.md が明記する「headless Raspberry Pi/Linux 前提、GUI依存を持ち込まない」
  という方針に反する。
- 既存の `plotting.py` の3関数（`plot_wire_geometry_phase_bc` 等）と同じ
  「matplotlib、`try/except ImportError`、`output_file` に保存 or 返すのみ」という型を
  そのまま踏襲した。`matplotlib.animation.PillowWriter` でGIF保存する構成を採用——
  `pillow` は matplotlib の依存として `uv.lock` に既に解決済みで、**新規依存の追加は不要**
  だったため、これ以上シンプルな選択肢はなかった。

### 座標変換は `wire_kinematics.py` の規約をそのまま複製した（新規ロジックなし）

`theta_anchor = theta_joint - theta_anchor_offset`、
`x_anchor = l_anchor*cos(theta_anchor)`、`z_anchor = -l_anchor*sin(theta_anchor)`
（`pulley_xy_from_polar()` と同じ `z=-l・sinθ` 規約）をそのままアニメーション関数内で計算している。
`solve_wire_geometry()` を呼ばずに手で再計算しているのは、アニメーションは「アンカー点の
xy座標そのもの」が欲しいのに対し `solve_wire_geometry()` は `l_wire`/`l_moment_arm` 等の
スカラー量しか返さない（xy座標を返す設計になっていない）ため。将来 `solve_wire_geometry()`
がxy座標も返すよう拡張されたら、ここも書き換えて重複を無くすのが望ましい（未対応・既知の課題参照）。

### 往復揺動（ping-pong）でループさせた

`theta_range` の片道分の配列を `np.concatenate([theta, theta[::-1]])` で往復にしてから
アニメーションを組み立てている。GIFは無限ループ再生されるものなので、単純な片道掃引だと
終端から始端へ瞬間移動する不自然な切り替わりが毎周期起きる。往復にすることで
「揺動」（`drive_modes.MotionSpec` が実際にモデル化している動作）をそのまま表現できる。

### 拮抗2本はアンカー点を「原点を頂点とする三角形」として描いた

2本のワイヤーがそれぞれ独立した `theta_anchor_offset_a`/`_b` を持ちうる
（`search_antagonistic_placement()` の引数構成）。単純に2本の腕を別々に描くと
「2つの独立したリンクが偶然同じ関節にある」ように見えかねないため、
`[原点, アンカーA, アンカーB, 原点]` を1本のポリラインとして描き、**1つの剛体リンクが
2つの取り付け点を持つ**ことを視覚的に示した。`theta_anchor_offset_a == theta_anchor_offset_b`
の場合は退化して1点に重なる（今回のスモークテストがこのケース）。

### 張力の色分けは呼び出し側から配列を渡す設計にした（オプション引数）

`tension`/`tension_a`/`tension_b` を渡すと `viridis`（単方向）または `autumn`/`winter`
（拮抗A/B）でワイヤーの色を張力に応じて変える。渡さなければグレー/赤/青の固定色。
本モジュールは物理モデルに依存しない設計（`wire_statics.py`/`drive_modes.py` のどちらの
結果でも渡せる）という既存方針をアニメーションでも維持した。

## 未対応・既知の課題

- **`solve_wire_geometry()` を経由せず座標計算を複製している**（上記参照）。現状は
  数式が単純（3行）なので実害は小さいが、`wire_kinematics.py` の規約が将来変わった場合は
  ここも同時に直す必要がある。
- **プーリー半径・ドラム半径は模式的な表示のみ**（物理的な意味を持たない固定サイズの丸）。
  E-2節（プーリー半径とワイヤー繰り出し量の関係）の実装が今後入っても、この見た目には
  反映されない。
- pytestテストは追加していない（既存3つの静的プロット関数と同じく、本モジュールは
  目視・スモークスクリプトのみで検証する慣習を踏襲）。
- ユーザーとの会話内で、同じ物理モデルをJavaScriptで再実装したインタラクティブな
  Artifact（ブラウザプレビュー用HTML）も作成したが、**これはリポジトリにはコミットしていない**
  （チャット確認専用、Pythonコードの二重管理を避けるため）。

## テスト状況

- [x] 単体テスト実行: `uv run pytest -q my_ak45/wire_mechanism/tests` — 54 passed
      （新規テストなし、既存の回帰が無いことのみ確認）
- [x] Lint: `uv run ruff check my_ak45/wire_mechanism/` — All checks passed
- [x] フォーマット: `uv run ruff format --check my_ak45/wire_mechanism/` — 全ファイル整形済み
- [x] 手動確認:
  - `assumed_params.py` の仮値・実際のPhase D探索結果（0.5Hz単方向1本、1.0Hz拮抗2本）を使い
    両関数でGIFを生成し、`Read`ツールでフレームを目視確認
    （リンクがアンカー点と共に回転し、ワイヤーが定滑車とアンカー点を結び続けること、
    拮抗版で従動側の張力がT_min=5Nに固定されたまま色が変わらないことを確認）
  - GIF生成時間: 単方向・拮抗ともに約20秒（120フレーム、fps=20、往復揺動込み）
- [ ] 統合テスト実行（対象外：実機非依存の純粋可視化処理）
