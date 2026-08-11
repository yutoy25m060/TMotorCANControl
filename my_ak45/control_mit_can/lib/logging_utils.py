"""ログファイル命名・制御ループ生成・コンソール出力記録の共通処理。"""

import sys
import time
import traceback
from pathlib import Path

from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

_LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"


def make_run_dir(name: str) -> Path:
    """control_mit_can/logs/{name}_{timestamp}/ を作成して返す（cwd非依存）。

    1回のスクリプト実行につき1回呼び出す。CSV・コンソールログなど、その実行で生成する
    記録ファイルはすべてこのディレクトリの下にまとめる。
    """
    timestamp = int(time.time())
    run_dir = _LOGS_DIR / f"{name}_{timestamp}"
    run_dir.mkdir(parents=True)
    return run_dir


def make_log_path(run_dir: Path, filename: str) -> str:
    """run_dir 内の filename へのパスを返す。"""
    return str(run_dir / filename)


def make_realtime_loop(**overrides) -> SoftRealtimeLoop:
    """SoftRealtimeLoop を既定値 (dt=0.01, report=True, fade=0) で構築する。

    config.yaml の control.realtime.* は現状どのスクリプトからも参照されていない
    ため、ここでも読み込まない（構造整理と挙動変更を分離するため意図的に見送り）。
    """
    params = {"dt": 0.01, "report": True, "fade": 0}
    params.update(overrides)
    return SoftRealtimeLoop(**params)


class _Tee:
    """write()/flush() を複数のストリームに複製する（標準出力とログファイルへの同時書き込み用）。"""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for stream in self._streams:
            stream.write(data)

    def flush(self):
        for stream in self._streams:
            stream.flush()


class console_log:
    """CUI表示（標準出力・標準エラー出力）を run_dir/console.log にも複製するコンテキストマネージャ。

    ターミナルへの表示はそのまま残しつつ、進捗表示・警告・例外トレースバックなど実行中に
    画面に出た内容を実行ごとのフォルダに保存する。スクリプト全体をこの with 文で囲んで使う。

    未捕捉の例外は、with 文を抜けた後にPythonインタプリタ側でトレースバック出力される
    （＝標準エラー出力を復元した後）ため、そのままでは記録に残らない。そのため __exit__ 内で
    例外情報を先に console.log へ書き出してから、ストリームの復元・例外の再送出を行う。
    """

    def __init__(self, run_dir: Path, filename: str = "console.log"):
        self._log_path = run_dir / filename

    def __enter__(self):
        self._log_file = open(self._log_path, "w", encoding="utf-8")
        self._stdout_orig = sys.stdout
        self._stderr_orig = sys.stderr
        sys.stdout = _Tee(self._stdout_orig, self._log_file)
        sys.stderr = _Tee(self._stderr_orig, self._log_file)
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb, file=self._log_file)
        sys.stdout = self._stdout_orig
        sys.stderr = self._stderr_orig
        self._log_file.close()
        return False
