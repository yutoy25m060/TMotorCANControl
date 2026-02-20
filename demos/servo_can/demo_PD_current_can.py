# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターに電流のステップ入力を行うデモプログラムです。
# モーターを初期化し、電流制御モードに設定します。
# 制御ループ内で、指令電流を0.4Aに設定し、モーターの状態を更新してコンソールに出力します。
# デモはCtrl+Cで終了することができます。



# システムパスを追加してライブラリをインポート可能にする
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
# サーボモーター用のライブラリを全てインポート
from TMotorCANControl.servo_can import *
# リアルタイムループ制御用のSoftRealtimeLoopをインポート
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time

# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID = 1

# 目標位置(Pdes)と目標速度(Vdes)の初期化
Pdes = 0
Vdes = 0

# PD制御のゲイン設定
P = 2  # 比例ゲイン
D = 0.3 # 微分ゲイン

# モーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    # 制御ループを0.002秒間隔で設定
    loop = SoftRealtimeLoop(dt=0.002, report=True, fade=0.0)
    # 現在位置をゼロ点として設定
    dev.set_zero_position()
    
    # モーターの状態を更新
    dev.update()
    # 電流制御モードに設定
    dev.enter_current_control()
    # 1秒待機
    time.sleep(1)
    
    # 制御ループを開始
    for t in loop:
        # 目標位置をsin波で生成
        Pdes = 3*np.sin(t)
        # PD制御則に基づいて指令電流(cmd)を計算
        # cmd = P * (目標位置 - 現在位置) + D * (目標速度 - 現在速度)
        # この例では目標速度が0なので、速度偏差は単純に -現在速度 となる
        cmd =  P*(Pdes - dev.position) + D*(Vdes - dev.velocity)
        # 計算した指令電流をモーターに設定
        dev.current_qaxis = cmd
        # モーターの状態を更新
        dev.update()
        # モーターの状態をコンソールに出力
        print(f" {dev}", end='')
        
