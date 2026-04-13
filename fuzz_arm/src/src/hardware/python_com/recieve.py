import socket

# --- 設定 ---
# 送信先マイコンのIPアドレス, ポート
TARGET_IP = "192.168.4.40"
TARGET_PORT = 8888
# 送信する角度司令（ディジタル値）[前半：Slave, 後半：Master] (-1800 ~ 1800)
MESSAGE = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"

# 受信待機するIPアドレス, ポート
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 8886

# 受信バッファサイズ
BUFFER_SIZE = 4096
# ----------------

def start_udp_server():
    """UDPサーバーを起動してデータを受信する関数"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((LISTEN_IP, LISTEN_PORT))
            print(f"UDPサーバーが起動しました。ポート {LISTEN_PORT} で待機中...")

            s.sendto(MESSAGE.encode('utf-8'), (TARGET_IP, TARGET_PORT)) # 初回にメッセージを送信
            while True:
                data, addr = s.recvfrom(BUFFER_SIZE)
                if addr[0] == TARGET_IP:
                    try:
                        # 受信データをUTF-8で文字列に変換
                        message = data.decode('utf-8')
                        print(f"マイコン({addr[0]}:{addr[1]})から受信: {message}")
                    except UnicodeDecodeError:
                        print(f"マイコン({addr[0]}:{addr[1]})からデコード不可能なデータを受信: {data}")
                else:
                    print(f"不明な送信元({addr[0]}:{addr[1]})からデータを受信しました。")


    except OSError as e:
        print(f"エラー: ポート {LISTEN_PORT} は既に使用されている可能性があります。({e})")
    except KeyboardInterrupt:
        print("\nサーバーの待機を終了します。")

if __name__ == '__main__':
    start_udp_server()