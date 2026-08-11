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
