from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# 解説
# これは、MIT CANモーターの読み取り専用デモです。
# モーターは最初にゼロ点に設定され、1秒後からモーターの位置と速度をリアルタイムで表示します。
# ユーザーがctrl+Cを押すまでデモは続きます。

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK45-36'  # モーターの種類
ID = 2 # モーターのID


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        dev.set_zero_position()
        time.sleep(1.5) # wait for the motor to zero (~1 second)
    
        print("Starting read only demo. Press ctrl+C to quit.")
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    
        for t in loop:
            dev.update()
            print(f"\rPosition: {dev.position:.3f} rad | Velocity: {dev.velocity:.3f} rad/s", end='')
    
    print("\nDemo finished.")