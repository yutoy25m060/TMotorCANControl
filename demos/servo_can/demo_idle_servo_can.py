# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターに電流のステップ入力を行うデモプログラムです。
# モーターを初期化し、電流制御モードに設定します。
# 制御ループ内で、指令電流を0.4Aに設定し、モーターの状態を更新してコンソールに出力します。
# デモはCtrl+Cで終了することができます。



# リアルタイムループ制御用のSoftRealtimeLoopをインポート
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# TMotorCANControlライブラリがシステムパスにない場合にパスを追加するためのコード
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
# サーボモーター用のTMotorManager_servo_canをインポート
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time

# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID = 1

# モーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    
    # 制御ループを0.005秒間隔で設定
    loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
    # モーターをアイドルモード（無抵抗状態）に設定
    dev.enter_idle_mode()
    # 現在の位置をゼロ点として設定
    dev.set_zero_position()
    # 制御ループを開始
    for t in loop:
        # モーターの状態を更新（位置などを読み取る）
        dev.update()
        # モーターの状態をコンソールに出力
        print("\r" + str(dev),end='')