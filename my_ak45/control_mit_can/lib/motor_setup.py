"""モーター初期化まわりの共通処理（単一モーター・複数モーター両対応）。"""

import time
from dataclasses import dataclass

from TMotorCANControl.mit_can import TMotorManager_mit_can


@dataclass
class MotorConfig:
    """config.yaml の motor: セクションを保持する軽量データ構造。"""

    type: str
    id: int
    max_temp: float


def get_motor_config(config: dict) -> MotorConfig:
    """config["motor"]（単一モーター設定）から MotorConfig を作る。"""
    motor = config["motor"]
    return MotorConfig(type=motor["type"], id=motor["id"], max_temp=motor["max_temp"])


def build_motor_manager(motor_config: MotorConfig, csv_file, log_vars) -> TMotorManager_mit_can:
    """単一モーター用の TMotorManager_mit_can を構築する。"""
    return TMotorManager_mit_can(
        motor_type=motor_config.type,
        motor_ID=motor_config.id,
        max_mosfett_temp=motor_config.max_temp,
        CSV_file=csv_file,
        log_vars=log_vars,
    )


def build_motor_managers(motors_config: list) -> list:
    """config["motors"]（複数モーター設定のリスト）から TMotorManager_mit_can を N台分構築する。

    個別CSVロギングは無効化する（複数モーターの記録は同期ロガー側が担当するため）。
    """
    return [
        TMotorManager_mit_can(
            motor_type=motor_config["type"],
            motor_ID=motor_config["id"],
            max_mosfett_temp=motor_config.get("max_temp", 50),
            CSV_file=None,
        )
        for motor_config in motors_config
    ]


def zero_position(motor, label=None, settle_time=1.5, verbose=True):
    """単一モーターの位置ゼロ化を行う。"""
    prefix = f"{label} " if label else ""
    if verbose:
        print(f"{prefix}位置ゼロ化を実行中...")
    motor.set_zero_position()
    time.sleep(settle_time)
    if verbose:
        print(f"{prefix}ゼロ化完了")


def zero_positions(motors, labels, settle_time=1.5):
    """複数モーターの位置ゼロ化を1台ずつ行う。

    以前は全台へゼロ化コードを送ってからまとめて1回だけ settle_time 待つ実装だったが、
    3台同時のゼロ化直後に実測位置がゼロになっていない（ゼロ化が完了しきっていない）事例が
    実機で確認されたため、zero_position()（単数版）と同じく1台ごとに settle_time 待つ
    方式に変更した。台数分だけ時間はかかるが、確実性を優先する。
    """
    print("全モーターの位置ゼロ化を実行中...")
    for label, motor in zip(labels, motors):
        print(f"  {label} ゼロ化中...")
        motor.set_zero_position()
        time.sleep(settle_time)
    print("全モーターゼロ化完了")
