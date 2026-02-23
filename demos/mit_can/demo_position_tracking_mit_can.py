from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can
# 解説
# これは、MIT CANモーターの位置追跡デモです。
# モーターは最初にゼロ点に設定され、1秒後から0.5*sin(pi*t)の位置を追跡します。
# 位置制御のゲインは、K=10、B=0.5に設定されています。これらの値は、モーターの特性や負荷に応じて調整する必要があるかもしれません。
# デモは、モーターの位置と速度をリアルタイムで表示し、ユーザーがctrl+Cを押すまで続きます。

# 0.5は振幅で、np.sin(np.pi*t)は時間に対する位置の変化を表しています。これにより、モーターは0.5ラジアンの振幅で1秒周期の正弦波を追跡します。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID

def position_tracking(dev):
    dev.set_zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)

    print("位置追跡デモを開始します。終了するにはCtrl+Cを押してください。")

    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.position = 0.0
        else:
            dev.position = 0.5*np.sin(np.pi*t)
    
    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        position_tracking(dev)