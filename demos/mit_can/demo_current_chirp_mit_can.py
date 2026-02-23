from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from NeuroLocoMiddleware.SysID import Chirp
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説
# これは、MIT CANモーターの電流チャープデモです。
# モーターは、1秒間で25Hzから250Hzまでの周波数で、振幅1.0のチャープ信号をトルク指令として追跡します。
# デモは、モーターのトルクをリアルタイムで更新し、ユーザーがctrl+Cを押すまで続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID


def chirp_demo(dev, amp=1.0, dt=0.001):
    print("Chirping ActPackA. Press CTRL-C to finish.")
    chirp = Chirp(250, 25, 1)
    dev.set_set_current_gains()
    
    print("Starting current chirp demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = dt, report=True, fade=0.1)
    for t in loop:
        dev.update()
        des_τ = loop.fade*amp*chirp.next(t)*3/3.7
        dev.torque = des_τ # a barely audible note

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        chirp_demo(dev, amp=1.0)
    print("done with chirp_demo()")