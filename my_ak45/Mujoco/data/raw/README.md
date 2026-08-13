# sysid 実機データ（生ログ）

`my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py` を実機（Raspberry Pi）で実行すると、
このディレクトリの下に `exp005_sysid_excitation_{タイムスタンプ}/` フォルダが自動作成され、
`log.csv`（指令トルク・実測位置/速度/電流/トルク/温度）と `console.log`（実行中のターミナル
出力の複製）が保存されます。

`my_ak45/control_mit_can/logs/` と異なり、このディレクトリは `.gitignore` の対象外です
（MuJoCo sysid の最適化を別PC・Windows/GPU側で行うため、実機データをgit経由でそのまま
共有できるようにする設計。詳細は
[`../../docs_syid/AK45-36_sysid_作業手順.md`](../../docs_syid/AK45-36_sysid_作業手順.md) 参照）。

不要な試行（パラメータ調整中の失敗ラン等）はコミット前に削除してください。

## 現在保持しているラン（2026-08-13 整理済み）

いずれも 基準周波数 4.0 Hz / 基準振幅 0.9 Nm / 10.25秒 / 1kHz。

| ラン | `sysid_run_check.py` 判定 | 用途 |
|---|---|---|
| `exp005_sysid_excitation_1786575616` | 条件付き合格（WARN 2件） | **同定・validationに使用**（採用トリオ） |
| `exp005_sysid_excitation_1786575633` | 条件付き合格（WARN 2件） | 同上 |
| `exp005_sysid_excitation_1786575782` | 条件付き合格（WARN 2件） | 同上 |
| `exp005_sysid_excitation_1786574251` | **不合格（FAIL 1件）** | 同定には使わない。`sysid_run_check.py` のFAIL判定が実際に発火した唯一の実例として、しきい値を見直す際の参照用に残置 |

2026-08-13 に、これ以外の7ラン（振幅1.5Nm版2本・10.0秒版2本・`wall_time` 列を追加する前の
版2本・未使用の合格ラン1本、計約8MB）を削除しました。`identification/` が参照するのは
採用トリオのみです。なお `identification/csv_adapter.py` は `wall_time` 列が無いCSVを
公称時刻 `t` で読むフォールバックを持っていますが、この経路を通るデータはもう残っていません
（将来 Pi 側の古いスクリプトで取得した場合に備えた防御的な実装として維持しています）。
