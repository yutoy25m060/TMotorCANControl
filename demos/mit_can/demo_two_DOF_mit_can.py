from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説 
# これは、MIT CANモーターの2自由度位置制御のデモです。
# 2台のモーターが同時に制御され、両方とも最初にゼロ点に設定され、1秒後から、振幅π/2のステップ信号を追跡します。
# 位置制御のゲインは、K=10、B=0.5に設定されています。これらの値は、モーターの特性や負荷に応じて調整する必要があるかもしれません。
# デモは、モーターの位置をリアルタイムで更新し、ユーザーがctrl+Cを押すまで続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
ID_1 = 1 # モーターのID
ID_2 = 2 # モーターのID

Type_1 = 'AK45-36'  # モーターの種類
Type_2 = 'AK45-36'  # モーターの種類


def two_DOF(dev1,dev2):
    dev1.set_zero_position()
    dev2.set_zero_position()
    time.sleep(1.5) # wait for the motors to zero (~1 second)
    dev1.set_impedance_gains_real_unit(K=10.0,B=0.5)
    dev2.set_impedance_gains_real_unit(K=10.0,B=0.5)
    
    print("Starting 2 DOF demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.005, report=True, fade=0)
    for t in loop:
        dev1.update()
        dev2.update()
        if t < 1.0:
            dev1.position = 0.0
            dev2.position = 0.0
        else:
            dev1.position = (np.pi/2)*int(t)
            dev2.position = (np.pi/2)*int(t)

    del loop

if __name__ == '__main__':
    # to use additional motors, simply add another with block
    # remember to give each motor a different log name!
    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_1) as dev1:
        with TMotorManager_mit_can(motor_type=Type_2, motor_ID=ID_2) as dev2:
            two_DOF(dev1,dev2)