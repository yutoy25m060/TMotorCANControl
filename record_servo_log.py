import time
import os
import sys
import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

# 1. ライブラリへのパスを追加 (環境に合わせて調整してください)
sys.path.append("/home/pi/Research/y25m060_20260212/TMotorCANControl/src")

from TMotorCANControl.servo_can import TMotorManager_servo_can

# 設定
# ==========================================
# 設定項目
# ==========================================
# 保存先のディレクトリ（例: '/home/pi/Research/logs'）
# . (ドット) は現在のディレクトリを指します
SAVE_PATH = "./demos_logs" 
# ==========================================
Type = 'AK45-36' # モーターの機種名
ID = 1

log_filename = f"servo_log_{Type}_ID{ID}_{int(time.time())}.csv"
log_full_path = os.path.join(SAVE_PATH, log_filename)

print(f"ID:{ID} のデータをログファイル {log_filename} に記録します...")

# 3. CSV_file 引数を指定して初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID, CSV_file=log_full_path) as dev:
    # 100Hz (0.01秒間隔) でループ
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    
    # 制御モードに入る (記録を開始するためには update を呼ぶ必要がある)
    dev.enter_duty_cycle_control()
    
    try:
        for t in loop:
            # 常に最新の状態をCANから取得
            # update() が呼ばれるたびに、内部で CSV ファイルへ一行書き込まれます
            dev.update()
            
            # Duty 0% (安全な保持状態) を継続
            dev.set_duty_cycle_percent(0.0)
            
            # コンソールにも現在の数値を簡易表示
            print(f"\rTime: {t:.2f}s | Pos: {dev.position:.4f} rad | Vel: {dev.velocity:.4f} rad/s", end='')
            
    except KeyboardInterrupt:
        print("\n記録を終了しました。")

# 終了後、同じディレクトリに CSV ファイルが生成されます