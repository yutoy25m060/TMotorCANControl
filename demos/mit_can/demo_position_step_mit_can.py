from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説
# これは、MIT CANモーターの位置ステップ応答を示すデモです。
# モーターは最初にゼロ点に設定され、1秒後に90度（π/2ラジアン）に位置ステップされます。
# 位置制御のゲインは、K=10、B=0.5に設定されています。これらの値は、モーターの特性や負荷に応じて調整する必要があるかもしれません。
# デモは、モーターの位置と速度をリアルタイムで表示し、ユーザーがctrl+Cを押すまで続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID

def position_step(dev):
    dev.set_zero_position() # 遅延があります！
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)
    
    print("位置ステップのデモを開始します。終了するには ctrl+C を押してください。")

    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.position = 0.0
        else:
            dev.position = np.pi/2.0

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        position_step(dev)