from TMotorCANControl.mit_can import TMotorManager_mit_can

# ご自身のデバイスに合わせてこれらの値を変更してください！
Type = 'AK80-9'
ID = 1

with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
    if dev.check_can_connection():
        print("\nモーターは正常に接続されています！\n")
    else:
        print("\nモーターが接続されていません。デバイスの電源、ネットワーク配線、およびCANバスの接続を確認してください。\n")
    