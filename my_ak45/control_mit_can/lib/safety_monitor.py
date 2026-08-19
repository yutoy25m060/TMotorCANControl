"""複数モーターの安全上限を監視し、超過時に全モーターを緊急停止するモニター。

TMotorManager_mit_can.update() は各モーター自身の max_temp（config.yaml の
motor(s).max_temp から構築時に設定される、motor.max_temp 属性で参照可能）を超えると
即座に RuntimeError を送出するが、config.yaml で運用者が設定する位置/速度/トルクの
ソフトウェア上限や、複数モーター横断の緊急停止（1台の異常で全台を止める）は
サポートしていない。ワイヤー駆動の脚機構は単体モーターより過張力・断線・詰まりの
リスクが高く、かつ複数の脚が同時に動くため、1台の異常が転倒や他の脚への連鎖的な
ダメージにつながりやすい。このモジュールはワークスペース層（my_ak45/）でその監視・
緊急停止を提供する。

check() は温度（motor.max_temp との比較）も監視対象に含む。ただし update() 自身が
同一しきい値で先に RuntimeError を送出するため、通常の呼び出し順序（全モーターを
update() した後で check() を呼ぶパターン）では、温度超過時に実際に効くのは
update() 呼び出し側の try/except（exp_003_multi_motor.py・
my_ak45/Mujoco/data_collection/exp_005_sysid_excitation.py 参照）であり、check() の
温度分岐はその防御線がない呼び出し順序向けの保険的な位置づけ。
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
            temp = motor.get_temperature_celsius()
            if temp > motor.max_temp:
                return True, f"{name}: 温度上限超過 ({temp:.1f}C > {motor.max_temp}C)"
        return False, None

    def trigger_emergency_stop(self, message):
        """全モーターを電源オフし、異常メッセージを表示する。"""
        print(f"!!! 緊急停止: {message} !!!")
        for name, motor in zip(self.motor_names, self.motors):
            try:
                motor.power_off()
            except Exception as e:
                print(f"  {name} の電源オフ中にエラー: {e}")

    def update_and_check(self):
        """全モーターの update() を呼び、続けて check() で安全上限を確認する。

        exp_003_multi_motor.py で確立された「update() を try/except RuntimeError で囲み、
        検知したら trigger_emergency_stop() へ合流させる（update() 自身が先に温度超過を
        検知するケースの防御線）→ check() で位置/速度/トルク/温度の上限超過を確認」という
        パターンを、呼び出し側での重複実装を避けるためにこのクラスへ集約したもの。単一
        モーター（motors=[motor] の1要素リスト）でも複数モーターでも同じ流れで使える。

        制御コマンド（set_output_angle_radians() 等）はこのメソッドの前後どちらでも
        呼び出し側の責任で行う（制御モードごとに異なるため、ここでは扱わない）。

        Returns:
            True: 異常を検知し緊急停止した場合（呼び出し側はループを break する想定）。
            False: 異常なし、または emergency_stop_enabled=False で警告のみ出した場合。
        """
        try:
            for motor in self.motors:
                motor.update()
        except RuntimeError as e:
            self.trigger_emergency_stop(str(e))
            return True

        exceeded, message = self.check()
        if not exceeded:
            return False
        if self.emergency_stop_enabled:
            self.trigger_emergency_stop(message)
            return True
        print(f"警告（緊急停止は無効）: {message}")
        return False
