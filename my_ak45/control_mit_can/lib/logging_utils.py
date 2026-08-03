"""ログファイル命名・制御ループ生成の共通処理。"""

import time
from pathlib import Path

from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

_LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"


def make_log_path(prefix: str) -> str:
    """control_mit_can/logs/{prefix}_{timestamp}.csv のパスを返す（cwd非依存）。"""
    timestamp = int(time.time())
    return str(_LOGS_DIR / f"{prefix}_{timestamp}.csv")


def make_realtime_loop(**overrides) -> SoftRealtimeLoop:
    """SoftRealtimeLoop を既定値 (dt=0.01, report=True, fade=0) で構築する。

    config.yaml の control.realtime.* は現状どのスクリプトからも参照されていない
    ため、ここでも読み込まない（構造整理と挙動変更を分離するため意図的に見送り）。
    """
    params = {"dt": 0.01, "report": True, "fade": 0}
    params.update(overrides)
    return SoftRealtimeLoop(**params)
