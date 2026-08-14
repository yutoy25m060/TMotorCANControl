# 実測列のずらし量に関する説明の訂正（sysid_rollout の暗黙の時刻オフセットを見落としていた）

## 冒頭メタ情報

- 日時: 2026-08-13（時刻未記録）
- 対象ファイル:
  - `my_ak45/Mujoco/identification/identify.py`（`DEFAULT_SHIFT` のコメント）
  - `my_ak45/Mujoco/identification/csv_adapter.py`（`build_sequences()` の docstring）
  - `my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py`（コメント。ついでに F541 の
    無意味な `f""` プレフィックスも修正）
  - `my_ak45/Mujoco/data_collection/sysid_run_check.py`（コメントと出力メッセージ）
  - `my_ak45/Mujoco/docs_syid/AK45-36_sysid_作業手順.md`（項目14）
- 種別: バグ修正（コード動作は変わらず、`DEFAULT_SHIFT=2` という結論も不変。誤っていたのは
  「なぜ2なのか」の説明）

## 設計判断と理由

`.ai/logs/2026-08-13_06_sysid-phase3-adapter-and-optimize_01.md` で「MuJoCo の rollout は
`sensor[i] = ctrl[0..i-1] への応答` という実機のロギング規約と同じ規約を持つので、帳簿上の
1サンプルは自動的に合い、補正すべきは電流ループの物理的なむだ時間（約2サンプル）だけ」と
説明していたが、これは誤りだった。

`sysid.sysid_rollout` の出力を実データで直接確認したところ:

```
pred.sensordata.data[0]  = 初期状態そのもの（1ステップも進めていない値）
pred.sensordata.times[0] = dt                （なのに時刻は dt から始まる）
```

`sysid.model_residual` はこの `times` を使って実測を窓がけし予測を補間するため、予測は
自分の物理時刻より**約1サンプル後**の実測と突き合わされる。つまり暗黙の前詰めが1サンプル
入っており、「rollout の因果性の規約が実機の帳簿上のずれと一致するから自動的に合う」という
前回の説明はこの暗黙のオフセットを見落としていた。

正しい整理:

- 実機CSV上の指令-実測のずれ = 帳簿上の1サンプル + 物理的なむだ時間 約1.9サンプル ≈ **3サンプル**
  （`sysid_run_check.py` の相互相関が測る「3行」はこの合計そのものであり、これまでの
  「1ms刻みの粗い測定で過大に出る」という説明も不要だった）
- そのうち約1サンプルは `sysid_rollout` の時刻付けにより暗黙に補正される
- よって明示的に指定するのは 3 − 1 = **約2サンプル**

`DEFAULT_SHIFT = 2` という結論、および実測したコストのスイープ結果
（shift=0:0.665 / 1:0.408 / **2:0.334** / 3:0.344）は変わらない。前回は「独立に測定された
L=1.82〜1.87ms と直接一致するので引き算の産物ではなく物理的なむだ時間そのもの」と書いたが、
実際には引き算（3−1）が正しく、L≈1.9 と 3−1=2 が近い値になったのは偶然だった。

この暗黙の1サンプルは `mujoco.sysid` の実装詳細に依存するため、mujoco のバージョンを
上げた際は shift のスイープをやり直すべき旨を各ファイルに注記した。

## 未対応・既知の課題

- この暗黙のオフセットの根本原因（なぜ `sensordata.times[0]` が `0` ではなく `dt` から
  始まる設計になっているか）は `mujoco.sysid` 側の実装判断であり、こちらでは変更できない。
  将来 `mujoco.sysid` を更新した際、この挙動が変わらないかは都度確認が必要
- `mujoco.sysid._src.residual`（プライベートAPI）を読んで確認した内容であり、公開APIの
  ドキュメントには明記されていない。将来のバージョンで挙動が変わっても警告なしに
  ずれる可能性がある

## テスト状況

- [ ] 単体テスト実行（このリポジトリのpytestスイートは `my_ak45/wire_mechanism/` のみが対象で、
      今回の変更は対象外）
- [ ] 統合テスト実行（同上）
- [x] 手動確認:
  - `uv run ruff check` で対象4ファイルにエラーなし
  - `uv run python -m py_compile` で Pi側2スクリプトともOK（変更はコメント・出力文字列のみで
    判定ロジックには手を入れていない）
  - `sysid_rollout` の `sensordata.data[0]` が初期状態と厳密に一致し `times[0] == dt` で
    あることを実データ（`exp005_sysid_excitation_1786575616`）で確認
- [ ] リグレッションテスト: 該当なし（コード動作の変更なし）
