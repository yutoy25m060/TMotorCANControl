# 要約
# このコードは、TMotorCANControlライブラリを使用して、2自由度のサーボモーターをCAN通信で制御するためのデモプログラムです。
# 2つのモーターを初期化し、1つ目のモーターを位置制御モードに設定し、2つ目のモーターをアイドルモードに設定します。
# 制御ループ内で、2つ目のモーターの位置を読み取り、1つ目のモーターの目標位置として設定することで、1つ目のモーターが2つ目のモーターの動きを追従するように制御します。
# 両方のモーターの状態をコンソールに出力し、リアルタイムで動きを確認できるようにしています。



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
    
    # 制御ループを0.01秒間隔で設定
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    # モーターを電流制御モードに設定
    dev.enter_current_control()
    # 制御ループを開始
    for t in loop:
        # 指令電流を0.4Aに設定（ステップ入力）
        dev.current_qaxis = 0.4
        # モーターの状態を更新
        dev.update()
        # モーターの状態をコンソールに出力
        print(f" {dev}", end='')
