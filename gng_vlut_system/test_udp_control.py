import socket
import time

def send_udp(msg, port=12345):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg.encode(), ('127.0.0.1', port))
    print(f"Sent: {msg}")

# Test 1: Neutral position, half-open gripper
send_udp("0,0,0,0,0,0,500,0,0,0,0,0,0,0,0")
time.sleep(1)

# Test 2: Bend arm
send_udp("450,-300,600,0,0,0,500,0,0,0,0,0,0,0,0")
time.sleep(1)

# Test 3: Close gripper
send_udp("450,-300,600,0,0,0,0,0,0,0,0,0,0,0,0")
