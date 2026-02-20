# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターがCAN通信で正しく接続されているかを確認するためのデモプログラムです。
# モーターの機種名とCAN IDを設定し、TMotorManager_servo_canクラスを使用してモーターを初期化します。
# check_can_connectionメソッドを呼び出して、モーターとの通信が確立しているかを確認し、結果をコンソールに出力します。


# TMotorCANControlライブラリからサーボモーター用のTMotorManager_servo_canをインポートします。
from TMotorCANControl.servo_can import TMotorManager_servo_can

# 使用するモーターに合わせて、以下の設定を変更してください！
# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID = 1

# TMotorManager_servo_canを使用してモーターデバイスを初期化します。
# 'with'ステートメントを使うことで、プログラム終了時に安全にモーターの電源がオフになります。
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    # check_can_connectionメソッドでモーターとのCAN通信が確立しているか確認します。
    if dev.check_can_connection():
        # 接続が確認できた場合のメッセージ
        print("\nモーターは正常に接続されています！\n")
    else:
        # 接続が確認できなかった場合のメッセージ
        print("\nモーターが接続されていません。モーターの電源、ネットワーク配線、CANバスの接続を確認してください。\n")
