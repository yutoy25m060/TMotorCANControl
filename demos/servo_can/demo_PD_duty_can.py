# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターにPD制御則に基づくデューティサイクルの指令を送るデモプログラムです。
# モーターを初期化し、デューティサイクル制御モードに設定します。
# 制御ループ内で、目標位置をsin波で生成し、PD制御則に基づいて指令デューティサイクルを計算してモーターに設定します。
# モーターの状態を更新し、コンソールに出力することで、モーターの応答を観察できます。



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
P = 0.1 # 比例ゲイン
D = 0.0 # 微分ゲイン

# モーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    # 制御ループを0.001秒間隔で設定
    loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0.0)
    # 現在位置をゼロ点として設定
    dev.set_zero_position()
    
    # モーターの状態を更新
    dev.update()
    # デューティサイクル制御モードに設定
    dev.enter_duty_cycle_control()
    # 1秒待機
    time.sleep(1)
    
    # 制御ループを開始
    for t in loop:
        # 目標位置をsin波で生成
        Pdes = np.sin(t)
        # PD制御則に基づいて指令デューティサイクルを計算し、モーターに設定
        # 指令値 = -P * (目標位置 - 現在位置) + D * (目標速度 - 現在速度)
        # 指令値が負になっているのは、モーターの回転方向と位置の符号を合わせるためだと思われる
        dev.set_duty_cycle_percent(-P*(Pdes - dev.position) + D*(Vdes - dev.velocity))
        # モーターの状態を更新
        dev.update()
        # モーターの状態をコンソールに出力
        print(f" {dev}", end='')

