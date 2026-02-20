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
import numpy as np

# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID1 = 1
ID2 = 2

# 1つ目のモーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID1) as dev1:
    # 2つ目のモーター(ID=1)を初期化
    with TMotorManager_servo_can(motor_type=Type, motor_ID=ID2) as dev2:
        # 制御ループを0.01秒間隔で設定
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        #両方のモーターの現在位置をゼロ点として設定
        dev1.set_zero_position()
        dev2.set_zero_position()

        # dev1を位置制御モードに設定
        dev1.enter_position_control()
        # dev2をアイドルモード（無抵抗状態）に設定
        dev2.enter_idle_mode()

        # 制御ループを開始
        for t in loop:
            # dev2の位置を読み取り、dev1の目標位置として設定（dev1がdev2の動きを追従する）
            dev1.position = dev2.position
            # dev1の状態を更新
            dev1.update()
            # dev2の状態を更新
            dev2.update()
            # 両方のモーターの状態をコンソールに出力
            print(f" {dev1} {dev2}", end='')
