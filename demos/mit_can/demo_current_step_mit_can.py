from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK80-9'
ID = 1

def current_step(dev):
    dev.set_zero_position()
    time.sleep(1.5) # モーターがゼロ点設定を完了するまで待ちます（約1秒）
    dev.set_current_gains()
    
    print("電流ステップのデモを開始します。終了するには ctrl+C を押してください。")

    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
    for t in loop:
        dev.update()
        if t < 1.0:
            dev.current_qaxis = 0.0
        else:
            dev.current_qaxis = 0.5

    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        current_step(dev)