# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターに速度のステップ入力を行うデモプログラムです。
# モーターを初期化し、速度制御モードに設定します。
# 制御ループ内で、指令速度を1.0ラジアン/秒に設定し、モーターの状態を更新してコンソールに出力します。
# デモはCtrl+Cで終了することができます。


# リアルタイムループ制御用のSoftRealtimeLoopをインポート
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# TMotorCANControlライブラリがシステムパスにない場合にパスを追加するためのコード
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:

# システムパスを追加してライブラリをインポート可能にする
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time

# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID = 1

# モーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    
    # 制御ループを0.01秒間隔で設定
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    # モーターを速度制御モードに設定
    dev.enter_velocity_control()
    # 制御ループを開始
    for t in loop:
        # 指令速度を1.0ラジアン/秒に設定（ステップ入力）
        dev.velocity = 1.0 # rad/s
        # モーターの状態を更新
        dev.update()
        # モーターの状態をコンソールに出力
        print("\r" + str(dev),end='')
