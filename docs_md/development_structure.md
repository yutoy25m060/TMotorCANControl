# TMotorCANControl 開発構成（推奨）

## フォルダ構成
```
demos/my_ak45_control/
├── 0_template_basic.py      # 基本制御テンプレート
├── 1_template_impedance.py  # インピーダンス制御テンプレート  
├── 2_template_current.py    # 電流制御テンプレート
├── config.yaml              # 設定ファイル
├── experiments/             # 実験スクリプト
│   ├── exp_001_gain_tuning.py     # ゲイン調整実験
│   ├── exp_002_step_response.py   # ステップ応答実験
│   ├── exp_003_multi_motor.py     # 多モーター制御実験
│   └── exp_004_trajectory.py      # 軌跡追従実験
├── logs/                    # 実験ログ
│   └── README.md           # ログ分析ガイド
├── .gitignore              # Git 除外設定
└── README_ja.md            # 日本語ドキュメント
```

## テンプレート
- `0_template_basic.py`: with ブロック + update() ループの基本骨組み
- 各実験は exp_NN_description.py として、テンプレートをコピーして作成

## 設定管理
- `config.yaml` で CAN 設定、モーター ID、ゲイン上限値を一元管理
- 実験スクリプト側は config.yaml を読み込み、パラメータを上書き

## ロギング
- CSV 出力は自動的に logs/ に保存
- ファイル名に タイムスタンプ を含める
- README_ja.md で進捗・実験結果を記録

## 版管理
- logs/*.csv は .gitignore に
- 実験スクリプト、テンプレート、config.yaml は Git 追跡対象

## 運用
1. 新規実験 → テンプレートをコピー
2. パラメータを config.yaml で管理
3. 実行 → ログ自動保存
4. 結果を README_ja.md に記録
