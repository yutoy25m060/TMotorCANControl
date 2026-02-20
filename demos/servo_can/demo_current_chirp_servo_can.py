# 要約
# このコードは、TMotorCANControlライブラリを使用して、サーボモーターに電流チャープ信号を入力するデモプログラムです。
# 周波数が300Hzから始まり200秒かけて線形に増加するチャープ信号を生成し、モーターに電流制御モードで入力します。
# 制御ループ内で、モーターの状態を更新し、チャープ信号に振幅を掛けた指令電流をモーターに設定することで、モーターの応答を観察できます。
# デモはCtrl+Cで終了することができます。


# リアルタイムループ制御用のSoftRealtimeLoopをインポート
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
# システム同定用のチャープ信号を生成するChirpをインポート
from NeuroLocoMiddleware.SysID import Chirp
# TMotorCANControlライブラリがシステムパスにない場合にパスを追加するためのコード
# try:
#      from TMotorCANControl.TMotorManager import TMotorManager
# except ModuleNotFoundError:
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
# サーボモーター用のTMotorManager_servo_canをインポート
from TMotorCANControl.servo_can import TMotorManager_servo_can


# モーターの機種名
Type = 'AK45-36'
# モーターのCAN ID
ID = 1

# チャープ信号による電流入力デモを実行する関数
def chirp_demo(dev, amp=1.0, dt=0.001):
    print("電流チャープ信号のデモを開始します。終了するにはCtrl+Cを押してください。")
    # 周波数が300Hzから始まり200秒かけて線形に増加するチャープ信号を生成
    chirp = Chirp(300, 200, True)
    # モーターを電流制御モードに設定
    dev.enter_current_control()

    # 制御ループを指定された時間間隔(dt)で設定
    loop = SoftRealtimeLoop(dt = dt, report=True)
    # 制御ループを開始
    for t in loop:
        # モーターの状態を更新
        dev.update()
        # チャープ信号に振幅(amp)を掛けて指令電流を生成し、モーターに設定
        # （コメントには「かろうじて聞こえる音」とある）
        dev.current_qaxis = amp*chirp.next(t) 

# モーター(ID=0)を初期化
with TMotorManager_servo_can(motor_type=Type, motor_ID=ID) as dev:
    # チャープ信号のデモを実行（振幅3.0A）
    chirp_demo(dev, amp=3.0)
print("チャープ信号のデモが完了しました。")
