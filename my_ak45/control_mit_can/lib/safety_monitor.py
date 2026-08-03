"""複数モーターの安全上限を監視し、超過時に全モーターを緊急停止するモニター。

TMotorManager_mit_can.update() はMOSFET温度チェックとMIT プロトコル生の範囲での
ラップアラウンド処理は行うが、config.yaml で運用者が設定する位置/速度/トルクの
ソフトウェア上限や、複数モーター横断の緊急停止（1台の異常で全台を止める）は
サポートしていない。ワイヤー駆動の脚機構は単体モーターより過張力・断線・詰まりの
リスクが高く、かつ複数の脚が同時に動くため、1台の異常が転倒や他の脚への連鎖的な
ダメージにつながりやすい。このモジュールはワークスペース層（my_ak45/）でその監視・
緊急停止を提供する。
"""


class SafetyMonitor:
    """複数モーターの位置・速度・トルクを監視し、上限超過時に全モーターへ緊急停止をかける。"""

    def __init__(self, motors, motor_names, max_position, max_velocity, max_torque, emergency_stop=True):
        """
        Args:
            motors: TMotorManager_mit_can インスタンスのリスト（with ブロックで既に __enter__ 済みのもの）。
            motor_names: motors と同じ順序のモーター識別名リスト。
            max_position: 許容する出力角度の絶対値上限 [rad]。
            max_velocity: 許容する出力速度の絶対値上限 [rad/s]。
            max_torque: 許容する出力トルクの絶対値上限 [Nm]。
            emergency_stop: True の場合、check() が異常を検知したら trigger_emergency_stop() を
                呼び出す側で実際に緊急停止処理を行う想定（このクラス自体はフラグを保持するのみ）。
        """
        self.motors = motors
        self.motor_names = motor_names
        self.max_position = max_position
        self.max_velocity = max_velocity
        self.max_torque = max_torque
        self.emergency_stop_enabled = emergency_stop

    def check(self):
        """
        全モーターの現在の状態を確認する。

        Returns:
            (True, メッセージ): いずれかのモーターがしきい値を超過している場合。
            (False, None): 全モーターが正常範囲内の場合。
        """
        for name, motor in zip(self.motor_names, self.motors):
            pos = motor.get_output_angle_radians()
            vel = motor.get_output_velocity_radians_per_second()
            torque = motor.get_output_torque_newton_meters()
            if abs(pos) > self.max_position:
                return True, f"{name}: 位置上限超過 ({pos:.3f} rad > {self.max_position} rad)"
            if abs(vel) > self.max_velocity:
                return True, f"{name}: 速度上限超過 ({vel:.3f} rad/s > {self.max_velocity} rad/s)"
            if abs(torque) > self.max_torque:
                return True, f"{name}: トルク上限超過 ({torque:.3f} Nm > {self.max_torque} Nm)"
        return False, None

    def trigger_emergency_stop(self, message):
        """全モーターを電源オフし、異常メッセージを表示する。"""
        print(f"!!! 緊急停止: {message} !!!")
        for name, motor in zip(self.motor_names, self.motors):
            try:
                motor.power_off()
            except Exception as e:
                print(f"  {name} の電源オフ中にエラー: {e}")
