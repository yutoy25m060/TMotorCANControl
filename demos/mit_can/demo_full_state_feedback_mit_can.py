from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説
# これは、MIT CANモーターのフル状態フィードバック制御のデモです。
# モーターは最初にゼロ点に設定され、1秒後から、振幅1.0のチャープ信号をトルク指令として追跡しながら、位置も0とπ/2の間でステップします。
# 位置制御のゲインは、K=10、B=1に設定されています。これらの値は、モーターの特性や負荷に応じて調整する必要があるかもしれません。
# デモは、モーターのトルクと位置をリアルタイムで更新し、ユーザーがctrl+Cを押すまで続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID

def full_state_feedback(dev):
    dev.set_zero_position() # has a delay!
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit_full_state_feedback(K=10,B=1)
    chirp = Chirp(250, 200, 0.5)

    print("Starting full state feedback demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    amp = 1.0
  
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.torque = 0.0
            dev.position = 0.0
        else:
            des_τ = loop.fade*amp*chirp.next(t)*3/3.7
            dev.torque = des_τ
            dev.position = (np.pi/2)*int(t)

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        full_state_feedback(dev)