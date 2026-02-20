import can
import time
import os
from datetime import datetime

# ==========================================
# 設定項目
# ==========================================
# 保存先のディレクトリ（例: '/home/pi/Research/logs'）
# . (ドット) は現在のディレクトリを指します
SAVE_PATH = "./can1_dump_logs" 

# ファイル名のプレフィックス（接頭辞）
FILE_PREFIX = "can1_dump"
# ==========================================

def start_logging():
    # 1. 保存先ディレクトリの作成（存在しない場合）
    if not os.path.exists(SAVE_PATH):
        os.makedirs(SAVE_PATH)
        print(f"ディレクトリを作成しました: {SAVE_PATH}")

    # 2. 開始時刻を取得してファイル名を生成 (例: can1_dump_20260221_031520.txt)
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{FILE_PREFIX}_{now}.txt"
    full_log_path = os.path.join(SAVE_PATH, filename)

    try:
        # can1 インターフェースの初期化
        bus = can.interface.Bus(channel='can1', interface='socketcan')
        print(f"can1 の監視を開始しました。")
        print(f"保存先: {full_log_path}")
        print("停止するには Ctrl+C を押してください。")
        
        with open(full_log_path, "w") as f:
            f.write(f"--- CAN1 Logging Start: {datetime.now()} ---\n")
            
            while True:
                msg = bus.recv(1.0) # 1秒タイムアウト
                
                if msg is not None:
                    # タイムスタンプとデータをフォーマット
                    timestamp = msg.timestamp
                    can_id = hex(msg.arbitration_id).upper().replace('0X', '').zfill(8)
                    dlc = msg.dlc
                    data = ' '.join([hex(b).upper().replace('0X', '').zfill(2) for b in msg.data])
                    
                    line = f"({timestamp:.6f}) can1 {can_id} [{dlc}] {data}"
                    
                    # コンソールに表示
                    print(line)
                    
                    # ファイルに書き込み
                    f.write(line + "\n")
                    f.flush() # 書き込みを即座に確定

    except KeyboardInterrupt:
        print(f"\n記録を終了しました。ファイル: {full_log_path}")
    except Exception as e:
        print(f"\nエラーが発生しました: {e}")

if __name__ == "__main__":
    start_logging()