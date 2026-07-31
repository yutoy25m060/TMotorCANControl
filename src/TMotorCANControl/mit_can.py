import can
import time
import csv
import traceback
import os
from collections import namedtuple
from enum import Enum
from math import isfinite
import numpy as np
import warnings

# このライブラリで制御可能な各モーター固有のパラメータ辞書
# 閾値は cubemars.com のモーターデータシートに基づいています

MIT_Params = {
    "ERROR_CODES": {
        0: "エラーなし",
        1: "過温度故障",
        2: "過電流故障",
        3: "過電圧故障",
        4: "低電圧故障",
        5: "エンコーダー故障",
        6: "相電流不平衡故障（ハードウェアが損傷している可能性があります）",
    },
    "AK80-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -18.0,
        "T_max": 18.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # q軸電流を補正するため
        "Kt_actual": 0.115,  # Need to use the right constant -- 0.115 by our calcs, 0.091 by theirs. At output leads to 1.31 by them and 1.42 by us.
        "GEAR_RATIO": 9.0,  # hence the 9 in the name
        "Use_derived_torque_constants": True,  # true if you have a better model
        "a_hat": [0.0, 1.15605006e00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02],
        #'a_hat' : [0.0,  8.23741648e-01, 4.57963164e-04,     2.96032614e-01, 9.31279510e-02]# [7.35415941e-02, 6.26896231e-01, 2.65240487e-04,     2.96032614e-01,  7.08736309e-02]# [-5.86860385e-02,6.50840079e-01,3.47461078e-04,8.58635580e-01,2.93809281e-01]
    },
    "AK10-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -65.0,
        "T_max": 65.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.16,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # 未テスト定数！
        "Kt_actual": 0.206,  # 未テスト定数！
        "GEAR_RATIO": 9.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK60-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -15.0,
        "T_max": 15.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.068,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # 未テスト定数！
        "Kt_actual": 0.087,  # 未テスト定数！
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK70-10": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -25.0,
        "T_max": 25.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.095,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # 未テスト定数！
        "Kt_actual": 0.122,  # 未テスト定数！
        "GEAR_RATIO": 10.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -76.0,
        "V_max": 76.0,
        "T_min": -12.0,
        "T_max": 12.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # 未テスト定数！
        "Kt_actual": 0.017,  # 未テスト定数！
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-64": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -8.0,
        "V_max": 8.0,
        "T_min": -144.0,
        "T_max": 144.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.119,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # 未テスト定数！
        "Kt_actual": 0.153,  # 未テスト定数！
        "GEAR_RATIO": 80.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK45-36": {
        "P_min": -12.5,  # MITプロトコル標準
        "P_max": 12.5,  # MITプロトコル標準
        "V_min": -30.0,  # 50000 ERPM / (14極対 * 36減速比) から算出される実用域
        "V_max": 30.0,
        "T_min": -32.0,  # 最大電流 35A 時の理論最大トルク付近
        "T_max": 32.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.1206,  # foc_current_kp の値と一致
        "Current_Factor": 0.59,
        "Kt_actual": 0.1206,  # 実測パラメータがないためKp値を暫定採用
        "GEAR_RATIO": 36.0,  # p_pid_ang_div より確定
        "Use_derived_torque_constants": False,
    },
}
"""
AKシリーズ TMotor アクチュエータの各タイプのモーターのパラメータとエラーコード定義を含む辞書。
利用可能であれば、摩擦損失を考慮したオプションのトルクモデルを使用できます。
現時点では、このようなモデルは AK80-9 でのみ利用可能です。

このモデルは以下の定数を持つ線形回帰から得られます：
    - a_hat[0] = バイアス
    - a_hat[1] = 標準トルク定数乗数
    - a_hat[2] = 非線形トルク定数乗数
    - a_hat[3] = クーロン摩擦
    - a_hat[4] = ギアボックス摩擦

モデルは以下の形式を持ちます：
τ = a_hat[0] + gr*(a_hat[1]*kt - a_hat[2]*abs(i))*i - (v/(ϵ + np.abs(v)) )*(a_hat[3] + a_hat[4]*np.abs(i))

以下の値を使用：
    - τ = 近似トルク
    - gr = ギア比
    - kt = 公称トルク定数
    - i = 電流
    - v = 速度
    - ϵ = 符号速度閾値
"""


class motor_state:
    """モーター状態を保存・更新するためのデータ構造"""

    def __init__(self, position, velocity, current, temperature, error, acceleration):
        """
        モーター状態を入力値に設定します。

        Args:
            position: 位置 [rad]
            velocity: 速度 [rad/s]
            current: 電流 [A]
            temperature: 温度 [℃]
            error: エラーコード、0 はエラーなしを意味します
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        """
        モーター状態を入力値に設定します。

        Args:
            position: 位置 [rad]
            velocity: 速度 [rad/s]
            current: 電流 [A]
            temperature: 温度 [℃]
            error: エラーコード、0 はエラーなしを意味します
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        """
        このモーター状態オブジェクトの値を別のモーター状態オブジェクトの値に設定します。

        Args:
            other_motor_state: このモーター状態オブジェクトの値を設定する値を持つ別のモーター状態オブジェクト。
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration


# update 時に送信される MIT_command を保存するためのデータ構造
class MIT_command:
    """update 時に送信される MIT_command を保存するためのデータ構造"""

    def __init__(self, position, velocity, kp, kd, current):
        """
        コマンドを入力値に設定します。

        Args:
            position: 位置 [rad]
            velocity: 速度 [rad/s]
            kp: 位置ゲイン
            kd: 速度ゲイン
            current: 電流 [A]
        """
        self.position = position
        self.velocity = velocity
        self.kp = kp
        self.kd = kd
        self.current = current


# コントローラーからのモーター状態、編集不可の名前付きタプル
MIT_motor_state = namedtuple("motor_state", "position velocity current temperature error")
"""
Motor state from the controller, uneditable named tuple
"""


# python-can listener object, with handler to be called upon reception of a message on the CAN bus
# CANバスでメッセージが受信されたときに呼び出されるハンドラーを持つpython-canリスナーオブジェクト
class motorListener(can.Listener):
    """
    Python-can listener object, with handler to be called upon reception of a message on the CAN bus
        CANバスでメッセージが受信されたときに呼び出されるハンドラーを持つpython-canリスナーオブジェクト
    """

    def __init__(self, canman, motor):
        """
        CANマネージャーとモーターオブジェクトの参照を保存します。

        Args:
            canman: メッセージを取得する CanManager オブジェクト
            motor: 更新する TMotorCANManager オブジェクト
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        """
        このリスナーのモーターを、メッセージがこのモーター向けの場合、msg に含まれる情報で更新します。

        Args:
            msg: python-can の CAN メッセージ
        """
        data = bytes(msg.data)
        ID = data[0]
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_MIT_message(data, self.motor.type))


# A class to manage the low level CAN communication protocols
# CAN通信プロトコルの低レベルを管理するクラス
class CAN_Manager(object):
    """A class to manage the low level CAN communication protocols
    CAN通信プロトコルの低レベルを管理するクラス
    """

    debug = False
    """
    Set to true to display every message sent and recieved for debugging.
    """
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    """
    Used to keep track of one instantation of the class to make a singleton object
    """

    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        シングルトン パターンで CAN バスを管理するオブジェクトを作成します。
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager")
            # CAN インターフェースをリセット（クラッシュや前回の実行のため）
            os.system("sudo /sbin/ip link set can0 down")
            # CAN インターフェースを起動（ビットレート 1Mbps）
            os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
            # python-can の Bus オブジェクトを作成（SocketCAN ドライバを使用）
            cls._instance.bus = can.interface.Bus(channel="can0", bustype="socketcan")  # bustype='socketcan_native')
            # CAN バスからのメッセージをリスナーに分配する Notifier を作成
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
        """
        ALl initialization happens in __new__
        """
        pass

    def __del__(self):
        """
        # shut down the CAN bus when the object is deleted
            CANバスをシャットダウンするため、オブジェクトが削除されたとき
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
            #これは呼び出されない可能性があるため、これが重要な場合は参照を保持し、明示的に削除してください。
        """
        os.system("sudo /sbin/ip link set can0 down")

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception
            CANバスでメッセージが受信されたときに更新されるモーターオブジェクトをサブスクライブします

        Args:
            motor: The TMotorManager object to be subscribed to the notifier
        """
        self.notifier.add_listener(motorListener(self, motor))

    # Locks value between min and max
    # 最小値と最大値の間で値をロック
    @staticmethod
    def limit_value(value, min, max):
        """
        値を最小値と最大値の間に制限します。

        Args:
            value: 制限される値。
            min: 値に許可される最小値（含む）。
            max: 値に許可される最大値（含む）。
        """
        if value >= max:
            return max
        elif value <= min:
            return min
        else:
            return value

    # interpolates a floating point number to fill some amount of the max size of unsigned int,
    # as specified with the num_bits
    @staticmethod
    def float_to_uint(x, x_min, x_max, num_bits):
        """
        浮動小数点数を num_bits 長の符号なし整数に補間します。
        x_max の数は num_bits の最大整数になり、x_min は 0 になります。

        Args:
            x: 変換する浮動小数点数
            x_min: 浮動小数点数の最小値
            x_max: 浮動小数点数の最大値
            num_bits: 符号なし整数のビット数
        """
        span = x_max - x_min
        bitratio = float((1 << num_bits) / span)
        x = CAN_Manager.limit_value(x, x_min, x_max - (2 / bitratio))
        # (x - x_min)*(2^num_bits)/span

        return CAN_Manager.limit_value(int((x - x_min) * (bitratio)), 0, int((x_max - x_min) * bitratio))

    # undoes the above method
    @staticmethod
    def uint_to_float(x, x_min, x_max, num_bits):
        """
        num_bits 長の符号なし整数を x_min と x_max の間の浮動小数点数に補間します。

        Args:
            x: 変換する符号なし整数
            x_min: 浮動小数点数の最小値
            x_max: 浮動小数点数の最大値
            num_bits: 符号なし整数のビット数
        """
        span = x_max - x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x * span / ((1 << num_bits) - 1) + x_min)

    # sends a message to the motor (when the motor is in MIT mode)
    def send_MIT_message(self, motor_id, data):
        """
        モーターに MIT モードメッセージを送信します。ヘッダーに motor_id とデータ配列 data を使用します。

        Args:
            motor_id: 送信するモーターの CAN ID。
            data: 送信する整数またはバイトのデータ配列。
        """
        DLC = len(data)
        assert DLC <= 8, "Data too long in message for motor " + str(motor_id)

        if self.debug:
            print("ID: " + str(hex(motor_id)) + "   Data: " + "[{}]".format(", ".join(hex(d) for d in data)))

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info))
        except can.CanError:
            if self.debug:
                print("    Message NOT sent")

    # send the power on code
    def power_on(self, motor_id):
        """
        motor_id に電源オンコードを送信します。

        Args:
            motor_id: メッセージを送信するモーターの CAN ID。
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

    # send the power off code
    def power_off(self, motor_id):
        """
        motor_id に電源オフコードを送信します。

        Args:
            motor_id: メッセージを送信するモーターの CAN ID。
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

    # ゼロ化コードを送信。スケールのように、位置をゼロにするのに約1秒かかります
    def zero(self, motor_id):
        """
        motor_id にゼロ化コードを送信します。このコードはモーターとの通信を約1秒間停止します。

        Args:
            motor_id: メッセージを送信するモーターの CAN ID。
        """
        self.send_MIT_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    # send an MIT control signal, consisting of desired position, velocity, and current, and gains for position and velocity control
    # basically an impedance controller
    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I):
        """
        モーターに MIT スタイルの制御信号を送信します。この信号は、モーター制御チップ上のフィールド指向コントローラー用の電流を生成するために使用され、次の式で与えられます：

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: メッセージを送信するモーターの CAN ID
            motor_type: モーターの種類を示す文字列、例 'AK80-9'
            position: 希望する位置 [rad]
            velocity: 希望する速度 [rad/s]
            Kp: 位置ゲイン
            Kd: 速度ゲイン
            I: 追加電流
        """
        position_uint16 = CAN_Manager.float_to_uint(
            position, MIT_Params[motor_type]["P_min"], MIT_Params[motor_type]["P_max"], 16
        )
        velocity_uint12 = CAN_Manager.float_to_uint(
            velocity, MIT_Params[motor_type]["V_min"], MIT_Params[motor_type]["V_max"], 12
        )
        Kp_uint12 = CAN_Manager.float_to_uint(Kp, MIT_Params[motor_type]["Kp_min"], MIT_Params[motor_type]["Kp_max"], 12)
        Kd_uint12 = CAN_Manager.float_to_uint(Kd, MIT_Params[motor_type]["Kd_min"], MIT_Params[motor_type]["Kd_max"], 12)
        I_uint12 = CAN_Manager.float_to_uint(I, MIT_Params[motor_type]["T_min"], MIT_Params[motor_type]["T_max"], 12)

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12 & 0x00F) << 4) | (Kp_uint12) >> 8,
            (Kp_uint12 & 0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12 & 0x00F) << 4) | (I_uint12) >> 8,
            (I_uint12 & 0x0FF),
        ]
        # print(data)
        self.send_MIT_message(motor_id, data)

    # convert data recieved from motor in byte format back into floating point numbers in real units
    def parse_MIT_message(self, data, motor_type):
        """
        RAW MIT メッセージを受け取り、読み取り可能な浮動小数点数にフォーマットします。

        Args:
            data: 解析する python-can メッセージオブジェクトのデータバイト
            motor_type: モーターの種類を示す文字列、例 'AK80-9'

        Returns:
            位置、速度、電流、温度、エラーを rad、rad/s、A、℃ で含む浮動小数点値を持つ MIT_Motor_State 名前付きタプル。
            0 はエラーなしを意味します。

            注目すべきことに、電流は報告された 'トルク' 値から A に変換され、これは i*Kt です。これにより、摩擦損失を考慮しない推定トルクではなく、実際の q 軸電流に基づく制御が可能になります。
        """
        assert len(data) == 8 or len(data) == 6, "Tried to parse a CAN message that was not Motor State in MIT Mode"
        temp = None
        error = None
        position_uint = data[1] << 8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4] >> 4) << 4) >> 4
        current_uint = (data[4] & 0x0F) << 8 | data[5]

        if len(data) == 8:
            temp = int(data[6])
            error = int(data[7])

        position = CAN_Manager.uint_to_float(
            position_uint, MIT_Params[motor_type]["P_min"], MIT_Params[motor_type]["P_max"], 16
        )
        velocity = CAN_Manager.uint_to_float(
            velocity_uint, MIT_Params[motor_type]["V_min"], MIT_Params[motor_type]["V_max"], 12
        )
        current = CAN_Manager.uint_to_float(current_uint, MIT_Params[motor_type]["T_min"], MIT_Params[motor_type]["T_max"], 12)

        if self.debug:
            print("  Position: " + str(position))
            print("  Velocity: " + str(velocity))
            print("  Current: " + str(current))
            if (temp is not None) and (error is not None):
                print("  Temp: " + str(temp))
                print("  Error: " + str(error))

        # returns the Tmotor "current" which is really a torque estimate
        return MIT_motor_state(position, velocity, current, temp, error)


# デフォルトでログに記録される変数
LOG_VARIABLES = ["output_angle", "output_velocity", "output_acceleration", "current", "output_torque"]


# コントローラーの可能な状態
class _TMotorManState(Enum):
    """
    異なる制御状態を追跡するための Enum
    """

    IDLE = 0
    IMPEDANCE = 1
    CURRENT = 2
    FULL_STATE = 3
    SPEED = 4


# the user-facing class that manages the motor.
class TMotorManager_mit_can:
    """
    モーターを管理するユーザー向けクラス。このクラスは、モーターの制御を安全に開始/終了するために、with as ブロック内で使用する必要があります。
    """

    def __init__(self, motor_type="AK80-9", motor_ID=1, max_mosfett_temp=80, CSV_file=None, log_vars=LOG_VARIABLES):
        """
        モーターマネージャーをセットアップします。このメソッドではデバイスが電源オンになりません！モーターの制御を試みる前に、__enter__ を呼び出す必要があります。通常は with ブロックを使用してください。

        Args:
            motor_type: 制御するモーターの種類、例 AK80-9。
            motor_ID: モーターの CAN ID。
            max_mosfett_temp: エラーをスローする MOSFET 温度の上限（摂氏度）。
            CSV_file: ログ情報を出力する CSV ファイル。None の場合、ログは記録されません。
            log_vars: ログする変数の Python リスト。可能な完全なリストは以下の通りです:
                - "output_angle"
                - "output_velocity"
                - "output_acceleration"
                - "current"
                - "output_torque"
                - "motor_angle"
                - "motor_velocity"
                - "motor_acceleration"
                - "motor_torque"
                - "mosfet_temperature"
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        print("デバイスを初期化: " + self.device_info_string())

        self._motor_state = motor_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._motor_state_async = motor_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._command = MIT_command(0.0, 0.0, 0.0, 0.0, 0.0)
        self._control_state = _TMotorManState.IDLE
        self._times_past_position_limit = 0
        self._times_past_current_limit = 0
        self._times_past_velocity_limit = 0
        self._angle_threshold = (
            MIT_Params[self.type]["P_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast
        self._current_threshold = (
            self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) - 3.0
        )  # A, only really matters if the current changes quick
        self._velocity_threshold = (
            MIT_Params[self.type]["V_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast
        self._old_pos = None
        self._old_curr = 0.0
        self._old_vel = 0.0
        self._old_current_zone = 0
        self.max_temp = max_mosfett_temp  # max temp in deg C, can update later

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        self.SF = 1.0

        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "output_angle": self.get_output_angle_radians,
            "output_velocity": self.get_output_velocity_radians_per_second,
            "output_acceleration": self.get_output_acceleration_radians_per_second_squared,
            "current": self.get_current_qaxis_amps,
            "output_torque": self.get_output_torque_newton_meters,
            "motor_angle": self.get_motor_angle_radians,
            "motor_velocity": self.get_motor_velocity_radians_per_second,
            "motor_acceleration": self.get_motor_acceleration_radians_per_second_squared,
            "motor_torque": self.get_motor_torque_newton_meters,
            "mosfet_temperature": self.get_temperature_celsius,
        }

        self._canman = CAN_Manager()
        self._canman.add_motor(self)

    def __enter__(self):
        """
        モーターを安全に電源オンし、ログファイル（指定されている場合）を開始するために使用されます。
        """
        print("デバイス制御を開始: " + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name, "w") as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"] + self.log_vars)
            self.csv_file = open(self.csv_file_name, "a").__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError("Device not connected: " + str(self.device_info_string()))
        return self

    def __exit__(self, etype, value, tb):
        """
        モーターを安全に電源オフし、ログファイル（指定されている場合）を閉じるために使用されます。
        """
        print("デバイス制御を終了: " + self.device_info_string())
        self.power_off()

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def TMotor_current_to_qaxis_current(self, iTM):
        """
        TMotor が報告するトルクを q 軸電流に変換しようとする。
        """
        return (
            MIT_Params[self.type]["Current_Factor"]
            * iTM
            / (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
        )

    def qaxis_current_to_TMotor_current(self, iq):
        """
        q 軸電流を TMotor が報告するトルクに変換しようとする。
        """
        return (
            iq
            * (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
            / MIT_Params[self.type]["Current_Factor"]
        )

    # CAN バスからモーターの状態メッセージを受け取るたびにハンドラーから呼ばれ、
    # 最新の状態情報を保存する
    def _update_state_async(self, MIT_state):
        """
        CAN バスからこのモーターの最新状態メッセージを受け取るたびにハンドラーから呼ばれ、最新の状態情報を保存します。

        Args:
            MIT_state: 最新のモーター状態を含む MIT_Motor_State 名前付きタプル。

        Raises:
            RuntimeError: デバイスが 0 以外のエラーコードを返した場合（0 はエラーなしを意味します）。
        """
        # エラーチェック：モーターがエラーを返している場合は例外を発生
        if MIT_state.error != 0:
            raise RuntimeError(
                "Driver board error for device: "
                + self.device_info_string()
                + ": "
                + MIT_Params["ERROR_CODES"][MIT_state.error]
            )

        # タイムスタンプを記録し、前回の更新からの経過時間を計算
        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        # 加速度を速度の変化から計算
        acceleration = (MIT_state.velocity - self._motor_state_async.velocity) / dt

        # モーターから返される "Current" は実は current*Kt（トルク相当）なので、実際の q 軸電流に変換
        self._motor_state_async.set_state(
            MIT_state.position,
            MIT_state.velocity,
            self.TMotor_current_to_qaxis_current(MIT_state.current),
            MIT_state.temperature,
            MIT_state.error,
            acceleration,
        )

        # 新しい非同期状態データが利用可能であることを示す
        self._updated = True

    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message recieved
    def update(self):
        """
        このメソッドは、コントローラーが使用する現在の状態を最新の受信メッセージと同期させるとともに、現在のコマンドを送信するためにユーザーが呼び出します。
        """

        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError(
                "モーター制御を安全に電源オンする前にモーター状態を更新しようとしました。デバイス: "
                + self.device_info_string()
            )

        if self.get_temperature_celsius() > self.max_temp:
            raise RuntimeError("温度が {}℃を超えています。デバイス: {}".format(self.max_temp, self.device_info_string()))

        # check that the motor data is recent
        # print(self._command_sent)
        now = time.time()
        if (now - self._last_command_time) < 0.25 and ((now - self._last_update_time) > 0.1):
            # print("状態の更新が要求されましたが、モーターからのデータがありません。ゼロ調整後の遅延時間を長くするか、周波数を下げるか、接続を確認してください。")
            warnings.warn(
                "状態の更新が要求されましたが、モーターからのデータがありません。ゼロ調整後の遅延時間を長くするか、周波数を下げるか、接続を確認してください。"
                + self.device_info_string(),
                RuntimeWarning,
            )
        else:
            self._command_sent = False

        # artificially extending the range of the position, current, and velocity that we track
        P_max = MIT_Params[self.type]["P_max"] + 0.01
        I_max = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) + 1.0
        V_max = MIT_Params[self.type]["V_max"] + 0.01

        if self._old_pos is None:
            self._old_pos = self._motor_state_async.position
        old_pos = self._old_pos
        old_curr = self._old_curr
        old_vel = self._old_vel

        new_pos = self._motor_state_async.position
        new_curr = self._motor_state_async.current
        new_vel = self._motor_state_async.velocity

        thresh_pos = self._angle_threshold
        thresh_curr = self._current_threshold
        thresh_vel = self._velocity_threshold

        curr_command = self._command.current

        actual_current = new_curr

        # TMotor はすべての返却値で限界を超えると -max にラップアラウンドします！！これを考慮に入れる
        if (thresh_pos <= new_pos and new_pos <= P_max) and (-P_max <= old_pos and old_pos <= -thresh_pos):
            self._times_past_position_limit -= 1
        elif (thresh_pos <= old_pos and old_pos <= P_max) and (-P_max <= new_pos and new_pos <= -thresh_pos):
            self._times_past_position_limit += 1

        # 電流は基本的に位置と同じですが、瞬時にコマンドを切り替えると実際に十分に速く変化してこれを狂わせる可能性があるため、それも考慮されています。電流ジッターの問題を解決するために電流にハードリミットを設けています。
        if (thresh_curr <= new_curr and new_curr <= I_max) and (-I_max <= old_curr and old_curr <= -thresh_curr):
            # self._old_current_zone = -1
            # if (thresh_curr <= curr_command and curr_command <= I_max):
            #     self._times_past_current_limit -= 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            else:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            new_curr = actual_current
        elif (thresh_curr <= old_curr and old_curr <= I_max) and (-I_max <= new_curr and new_curr <= -thresh_curr):
            # self._old_current_zone = 1
            # if not (-I_max <= curr_command and curr_command <= -thresh_curr):
            #     self._times_past_current_limit += 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            else:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            new_curr = actual_current

        # 速度は位置と同じように動作するはず
        if (thresh_vel <= new_vel and new_vel <= V_max) and (-V_max <= old_vel and old_vel <= -thresh_vel):
            self._times_past_velocity_limit -= 1
        elif (thresh_vel <= old_vel and old_vel <= V_max) and (-V_max <= new_vel and new_vel <= -thresh_vel):
            self._times_past_velocity_limit += 1

        # 拡張状態変数を更新
        self._old_pos = new_pos
        self._old_curr = new_curr
        self._old_vel = new_vel

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += self._times_past_position_limit * 2 * MIT_Params[self.type]["P_max"]
        self._motor_state.current = actual_current
        self._motor_state.velocity += self._times_past_velocity_limit * 2 * MIT_Params[self.type]["V_max"]

        # send current motor command
        self._send_command()

        # writing to log file
        if self.csv_file_name is not None:
            self.csv_writer.writerow(
                [self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars]
            )

        self._updated = False

    # 制御モードに応じてモーターにコマンドを送信
    def _send_command(self):
        """
        制御モードに応じて、適切なコマンドを CAN バス経由でモーターに送信します。
        update() から呼ばれますが、状態更新なしでコマンドを送りたい場合は単独で呼び出してもよいです。
        """
        # 制御モードをチェックし、それぞれのモードに対応するコマンドを送信
        if self._control_state == _TMotorManState.FULL_STATE:
            # フル状態フィードバック：位置+速度+電流制御
            self._canman.MIT_controller(
                self.ID,
                self.type,
                self._command.position,
                self._command.velocity,
                self._command.kp,
                self._command.kd,
                self.qaxis_current_to_TMotor_current(self._command.current),
            )
        elif self._control_state == _TMotorManState.IMPEDANCE:
            # インピーダンス制御：位置+速度ゲインのみ（フィードフォワード電流なし）
            self._canman.MIT_controller(
                self.ID, self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, 0.0
            )
        elif self._control_state == _TMotorManState.CURRENT:
            # 電流制御モード：電流コマンドのみ
            self._canman.MIT_controller(
                self.ID, self.type, 0.0, 0.0, 0.0, 0.0, self.qaxis_current_to_TMotor_current(self._command.current)
            )
        elif self._control_state == _TMotorManState.IDLE:
            # アイドルモード：すべてゼロコマンド
            self._canman.MIT_controller(self.ID, self.type, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif self._control_state == _TMotorManState.SPEED:
            # 速度制御モード：速度コマンドのみ
            self._canman.MIT_controller(self.ID, self.type, 0.0, self._command.velocity, 0.0, self._command.kd, 0.0)
        else:
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())
        # 最後にコマンドを送った時刻を記録（接続確認などで使用）
        self._last_command_time = time.time()

    # モーターの基本的なユーティリティコマンド
    def power_on(self):
        """モーターの電源をオンにします。かすかなヒス音が聞こえるかもしれません。"""
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        """モーターの電源をオフにします。"""
        self._canman.power_off(self.ID)

    # 位置をゼロにする（スケールと同様に、ゼロ化後は約 1 秒待つ必要がある）
    # ゼロ化後の待機時間はユーザーの責任
    def set_zero_position(self):
        """位置をゼロにします。スケールのように、約1秒待つ必要があります。"""
        self._canman.zero(self.ID)
        self._last_command_time = time.time()

    # モーター状態を取得するゲッター関数
    def get_temperature_celsius(self):
        """
        戻り値:
        最新のモーター温度（摂氏度）。
        """
        return self._motor_state.temperature

    def get_motor_error_code(self):
        """
        戻り値:
        最新のモーターエラーコード。
        注意: この値が 0 以外の場合、プログラムはランタイムエラーをスローします。

        コード:
        - 0 : 'エラーなし'
        - 1 : '過温度故障'
        - 2 : '過電流故障'
        - 3 : '過電圧故障'
        - 4 : '低電圧故障'
        - 5 : 'エンコーダー故障'
        - 6 : '相電流不平衡故障（ハードウェアが損傷している可能性があります）'
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self):
        """
        戻り値:
        最新の q 軸電流（アンペア）。
        """
        return self._motor_state.current

    def get_output_angle_radians(self):
        """
        戻り値:
        最新の出力角度（ラジアン）。
        """
        return self._motor_state.position

    def get_output_velocity_radians_per_second(self):
        """
        戻り値:
            最新の出力速度（ラジアン/秒）。
        """
        return self._motor_state.velocity

    def get_output_acceleration_radians_per_second_squared(self):
        """
        戻り値:
            最新の出力加速度（ラジアン/秒²）。
        """
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self):
        """
        戻り値:
            最新の出力トルク（Nm）。
        """
        return self.get_current_qaxis_amps() * MIT_Params[self.type]["Kt_actual"] * MIT_Params[self.type]["GEAR_RATIO"]

    # プレーンインピーダンスモードを使用し、電流コマンドに 0.0 を送信します。
    def set_impedance_gains_real_unit(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """
        プレーンインピーダンスモードを使用し、位置リクエストに加えて電流コマンドに 0.0 を送信します。

        Args:
            kp: Dephy ライブラリとの後方互換性のためのダミー引数。
            ki: Dephy ライブラリとの後方互換性のためのダミー引数。
            K: 剛性 [Nm/rad]。
            B: 減衰 [Nm/(rad/s)]。
            ff: Dephy ライブラリとの後方互換性のためのダミー引数。
        """
        assert isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"]
        assert isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"]
        self._command.kp = K
        self._command.kd = B
        self._command.velocity = 0.0
        self._control_state = _TMotorManState.IMPEDANCE

    # フル MIT モードを使用し、設定された電流コマンドを送信します。
    def set_impedance_gains_real_unit_full_state_feedback(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """
        フル状態フィードバックモードを使用し、位置リクエストに加えて設定された電流コマンドを送信します。

        Args:
            kp: Dephy ライブラリとの後方互換性のためのダミー引数。
            ki: Dephy ライブラリとの後方互換性のためのダミー引数。
            K: 剛性 [Nm/rad]。
            B: 減衰 [Nm/(rad/s)]。
            ff: Dephy ライブラリとの後方互換性のためのダミー引数。
        """
        assert isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"]
        assert isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"]
        self._command.kp = K
        self._command.kd = B
        self._control_state = _TMotorManState.FULL_STATE

    # プレーン電流モードを使用し、位置ゲインに 0.0 を送信します。
    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        """
        プレーン電流モードを使用し、位置ゲインに加えて要求された電流を送信します。

        Args:
            kp: Dephy ライブラリとの後方互換性のためのダミー引数。
            ki: Dephy ライブラリとの後方互換性のためのダミー引数。
            ff: Dephy ライブラリとの後方互換性のためのダミー引数。
            spoof: Dephy ライブラリとの後方互換性のためのダミー引数。
        """
        self._control_state = _TMotorManState.CURRENT

    def set_speed_gains(self, kd=1.0):
        """
        プレーン速度モードを使用し、位置ゲインとフィードフォワード電流に 0.0 を送信します。

        Args:
            kd: 速度コントローラーのゲイン。制御法は (v_des - v_actual)*kd = iq です。
        """
        self._command.kd = kd
        self._control_state = _TMotorManState.SPEED

    # インピーダンス制御またはフル状態フィードバック（MIT）制御の位置目標値を設定
    def set_output_angle_radians(self, pos):
        """
        インピーダンスまたはフル状態フィードバックモードで出力角度コマンドを設定します。
        注意: このメソッドはコマンドを送信せず、TMotorManager の保存されたコマンドを更新します。
        update() が呼び出されたときに送信されます。

        Args:
            pos: 希望する出力位置 [rad]。
        """
        # position commands must be within a certain range :/
        # pos = (np.abs(pos) % MIT_Params[self.type]["P_max"])*np.sign(pos) # this doesn't work because it will unwind itself!
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if np.abs(pos) >= MIT_Params[self.type]["P_max"]:
            raise RuntimeError(
                "Cannot control using impedance mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["P_max"])
                + "rad!"
            )

        if self._control_state not in [_TMotorManState.IMPEDANCE, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send position command without gains for device " + self.device_info_string())
        self._command.position = pos

    def set_output_velocity_radians_per_second(self, vel):
        """
        速度またはフル状態フィードバックモードで出力速度コマンドを設定します。
        注意: このメソッドはコマンドを送信せず、TMotorManager の保存されたコマンドを更新します。
        update() が呼び出されたときに送信されます。

        Args:
            vel: 希望する出力速度 [rad/s]。
        """
        if np.abs(vel) >= MIT_Params[self.type]["V_max"]:
            raise RuntimeError(
                "Cannot control using speed mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["V_max"])
                + "rad/s!"
            )

        if self._control_state not in [_TMotorManState.SPEED, _TMotorManState.FULL_STATE]:
            raise RuntimeError("Attempted to send speed command without gains for device " + self.device_info_string())
        self._command.velocity = vel

    # 電流制御またはフル状態フィードバック制御の電流目標値を設定
    def set_motor_current_qaxis_amps(self, current):
        """
        電流またはフル状態フィードバックモードで電流コマンドを設定します。
        注意: このメソッドはコマンドを送信せず、TMotorManager の保存されたコマンドを更新します。
        update() が呼び出されたときに送信されます。

        Args:
            current: 希望する電流 [A]。
        """
        if self._control_state not in [_TMotorManState.CURRENT, _TMotorManState.FULL_STATE]:
            raise RuntimeError(
                "Attempted to send current command before entering current mode for device " + self.device_info_string()
            )
        self._command.current = current

    # 電流制御またはフル状態フィードバック制御で、希望トルクから電流を計算して設定
    def set_output_torque_newton_meters(self, torque):
        """
        電流または MIT モードで、希望するトルクに基づいて電流を設定します。
        モーターに複雑なトルクモデルが利用可能な場合はそれを使用します。
        それ以外の場合は、モーターのトルク定数を使用します。

        Args:
            torque: 希望する出力トルク [Nm]。
        """
        self.set_motor_current_qaxis_amps((torque / MIT_Params[self.type]["Kt_actual"] / MIT_Params[self.type]["GEAR_RATIO"]))

    # 減速比を考慮したモーター側の関数（モーター軸側のトルク制御）
    def set_motor_torque_newton_meters(self, torque):
        """
        減速比を考慮してモーター側のトルクを制御する set_output_torque のバージョン。

        Args:
            torque: 希望するモーター側のトルク [Nm]。
        """
        self.set_output_torque_newton_meters(torque * MIT_Params[self.type]["Kt_actual"])

    def set_motor_angle_radians(self, pos):
        """
        減速比を考慮してモーター側の角度を制御する set_output_angle のラッパー。

        Args:
            pos: 希望するモーター側の位置 [rad]。
        """
        self.set_output_angle_radians(pos / (MIT_Params[self.type]["GEAR_RATIO"]))

    def set_motor_velocity_radians_per_second(self, vel):
        """
        減速比を考慮してモーター側の速度を制御する set_output_velocity のラッパー。

        Args:
            vel: 希望するモーター側の速度 [rad/s]。
        """
        self.set_output_velocity_radians_per_second(vel / (MIT_Params[self.type]["GEAR_RATIO"]))

    def get_motor_angle_radians(self):
        """
        減速比を考慮してモーター側の角度を取得する get_output_angle のラッパー。

        Returns:
            最新のモーター側の角度 [rad]。
        """
        return self._motor_state.position * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        """
        減速比を考慮してモーター側の速度を取得する get_output_velocity のラッパー。

        Returns:
            最新のモーター側の速度 [rad/s]。
        """
        return self._motor_state.velocity * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        """
        減速比を考慮してモーター側の加速度を取得する get_output_acceleration のラッパー。

        Returns:
            最新のモーター側の加速度 [rad/s/s]。
        """
        return self._motor_state.acceleration * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        """
        減速比を考慮してモーター側のトルクを取得する get_output_torque のラッパー。

        Returns:
            最新のモーター側のトルク [Nm]。
        """
        return self.get_output_torque_newton_meters() * MIT_Params[self.type]["GEAR_RATIO"]

    # デバッグ表示用：モーターの情報を見やすく表示
    def __str__(self):
        """モーターのデバイス情報と電流を表示します。"""
        return (
            self.device_info_string()
            + " | Position: "
            + "{: 1f}".format(round(self.θ, 3))
            + " rad | Velocity: "
            + "{: 1f}".format(round(self.θd, 3))
            + " rad/s | current: "
            + "{: 1f}".format(round(self.i, 3))
            + " A | torque: "
            + "{: 1f}".format(round(self.τ, 3))
            + " Nm"
        )

    def device_info_string(self):
        """モーターの ID とデバイスタイプを表示します。"""
        return str(self.type) + "  ID: " + str(self.ID)

    # CAN 接続確認：10 回のコマンドを送り、10 回の応答があるかで接続を判定
    def check_can_connection(self):
        """
        起動メッセージを 10 回送信することでモーターの接続を確認します。
        10 回の応答が返された場合、接続が確立されていると判断します。

        __enter__() によってモーター制御が有効になった後にのみ呼び出し可能です。

        Returns:
            接続が確立されている場合は True、それ以外の場合は False。
        """
        # モーター制御が有効になっていることを確認
        if not self._entered:
            raise RuntimeError(
                "モーター制御を開始する前に check_can_connection を呼び出そうとしました！__enter__ メソッドを使用して制御を開始するか、with ブロック内で TMotorManager をインスタンス化してください。"
            )

        # 一時的な CAN リスナーを作成（接続確認中のメッセージを受け取るため）
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)

        # モーターに対して 10 回のパワーオン メッセージを送信（強制的に応答させるため）
        for i in range(10):
            self.power_on()
            time.sleep(0.001)

        # モーターからの応答を受け取る試行
        success = True
        time.sleep(0.1)  # モーターが応答するまで待機
        for i in range(10):
            flag = Listener.get_message(timeout=0.1)  # 100ms のタイムアウト
            # モーターから応答がない或いは別のデバイスからの応答の場合、失敗と判定
            if flag is None or (flag.arbitration_id & 0xFF) != self.ID:
                success = False

        # 一時的なリスナーをクリーンアップして結果を返す
        self._canman.notifier.remove_listener(Listener)
        return success

    temperature = property(get_temperature_celsius, doc="温度（摂氏度）")
    """温度（摂氏度）"""

    error = property(get_motor_error_code, doc="温度（摂氏度）")
    """モーターエラーコード。0 はエラーなしを意味します。"""

    # 電気量に関連する変数（プロパティー）
    current_qaxis = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps_current_only")
    """Q軸電流（アンペア）"""

    # 出力側（ギアボックス後）の変数（プロパティー）
    position = property(get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians_impedance_only")
    """出力角度（ラジアン）"""

    velocity = property(
        get_output_velocity_radians_per_second,
        set_output_velocity_radians_per_second,
        doc="output_velocity_radians_per_second",
    )
    """出力速度（ラジアン/秒）"""

    acceleration = property(
        get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared"
    )
    """出力加速度（ラジアン/秒²）"""

    torque = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")
    """出力トルク（Nm）"""

    # モーター側（ギアボックス前）の変数（プロパティー）
    position_motorside = property(get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians_impedance_only")
    """モーター側角度（ラジアン）"""

    velocity_motorside = property(
        get_motor_velocity_radians_per_second, set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second"
    )
    """モーター側速度（ラジアン/秒）"""

    acceleration_motorside = property(
        get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared"
    )
    """モーター側加速度（ラジアン/秒²）"""

    torque_motorside = property(
        get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters"
    )
    """モーター側トルク（Nm）"""
