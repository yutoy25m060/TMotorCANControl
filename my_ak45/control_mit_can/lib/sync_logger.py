"""複数モーターの状態を共通タイムラインで1つのCSVに記録するロガー。

TMotorManager_mit_can はモーターごとに個別CSV（各モーターindependentな pi_time 起点）を
書き出す設計になっているため、複数モーターの歩容データを同じタイムスタンプで
突き合わせたい場合には向いていない。このモジュールは、外部（実験スクリプト側）の
制御ループが持つ共通の経過時間 t を使って、1行=1タイムステップ、列=各モーターの
指定変数、という形式で記録する。
"""

import csv


class SyncMultiMotorLogger:
    """複数モーターの状態を共通タイムラインで1行にまとめて記録するロガー。"""

    def __init__(self, csv_file, motors, motor_names, log_vars):
        """
        Args:
            csv_file: 出力先CSVファイルのパス。
            motors: TMotorManager_mit_can インスタンスのリスト（with ブロックで既に __enter__ 済みのもの）。
            motor_names: motors と同じ順序のモーター識別名リスト。
            log_vars: 各モーターについて記録する変数名のリスト（TMotorManager_mit_can.LOG_FUNCTIONS のキー）。
        """
        self.motors = motors
        self.log_vars = log_vars
        header = ["t"]
        for name in motor_names:
            header += [f"{name}_{var}" for var in log_vars]
        self._file = open(csv_file, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)

    def log(self, t):
        """時刻 t における全モーターの状態を1行として書き込む。"""
        row = [t]
        for motor in self.motors:
            row += [motor.LOG_FUNCTIONS[var]() for var in self.log_vars]
        self._writer.writerow(row)

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, etype, value, tb):
        self.close()
