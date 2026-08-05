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
    chirp = Chirp(250, 200, 0.5) # 250Hzから200Hzまでのチャープ信号を生成します。必要に応じてこの範囲を調整してください。

    print("Starting full state feedback demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.001, report=True, fade=0)
    amp = 1.0
  
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.torque = 0.0
            dev.position = 0.0
        else:
            # チャープ信号に基づいてトルク指令を生成します。必要に応じてこの式を調整してください。
            des_τ = loop.fade*amp*chirp.next(t)*3/3.7 # 3.7はAK45-36の最大トルクです。必要に応じてこの値を変更してください。
            # ⚠️ 未突合・実機未検証: この3.7という値は、src/TMotorCANControl/mit_can.py の
            # MIT_Params["AK45-36"]["T_max"]=32.0 や my_ak45/control_mit_can/docs/ の仕様書に
            # 記載の18.0とも一致しない。3箇所とも実測で検証されていないため、正確なスケーリングが
            # 必要な場合はKt_actual/GEAR_RATIOから再計算するか、実機で確認すること。
            dev.torque = des_τ # トルク指令をモーターに送ります。
            dev.position = (np.pi/2)*int(t) # 位置を0とπ/2の間でステップさせます。必要に応じてこの式を調整してください。

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        full_state_feedback(dev)