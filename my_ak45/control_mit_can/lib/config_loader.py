"""control_mit_can/config.yaml の読み込み。

呼び出し元スクリプトが control_mit_can/ 直下（テンプレート）と
control_mit_can/experiments/ 配下（実験スクリプト）のどちらから実行されても
同じ config.yaml を読み込めるよう、パス解決はこのモジュール自身のファイル位置
（cwd ではなく）基準で行う。
"""

from pathlib import Path

import yaml

_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config() -> dict:
    """control_mit_can/config.yaml を読み込んで dict を返す。"""
    with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
