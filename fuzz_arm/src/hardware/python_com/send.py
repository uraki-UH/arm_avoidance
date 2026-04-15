import socket

# --- 設定 ---
# 送信先マイコンのIPアドレス, ポート
TARGET_IP = "192.168.4.40"
TARGET_PORT = 8888

# 送信する角度司令（ディジタル値）[前半：Slave, 後半：Master] (-1800 ~ 1800)
MESSAGE = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
# ----------------

def send_udp_message():
    """指定したIPアドレスとポートにUDPメッセージを送信する"""
    
    print(f"送信先: {TARGET_IP}:{TARGET_PORT}")
    print(f"送信メッセージ: {MESSAGE}")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            # 文字列をUTF-8のバイト列にエンコードして送信
            s.sendto(MESSAGE.encode('utf-8'), (TARGET_IP, TARGET_PORT))
            print("メッセージの送信に成功しました。")
        except socket.error as e:
            print(f"エラー: メッセージの送信に失敗しました。 - {e}")
        except Exception as e:
            print(f"予期せぬエラーが発生しました。 - {e}")

if __name__ == '__main__':
    send_udp_message()