from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説
# これは、MIT CANモーターのトルクステップ応答を示すデモです。
# モーターは最初にゼロ点に設定され、1秒後に1.0Nmのトルクステップが加えられます。
# デモは、モーターのトルクをリアルタイムで表示し、ユーザーがctrl+Cを押すまで続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID


def torque_step(dev):
    dev.set_zero_position()
    time.sleep(1.5) # モーターがゼロ点設定を完了するまで待ちます（約1秒）
    dev.set_current_gains()
    
    print("トルクステップのデモを開始します。終了するには ctrl+C を押してください。")
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.torque = 0.0
        else:
            dev.torque = 1.0

    del loop


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        torque_step(dev)