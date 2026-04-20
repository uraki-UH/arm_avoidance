import socket
import time
import math

def send_udp_status(angles, port=8886):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Format: agl,v1,v2,...,v14,
    # Values: integers representing degrees * 10
    val_strs = [str(int(a * 10)) for a in angles]
    msg = "agl," + ",".join(val_strs) + ","
    sock.sendto(msg.encode(), ('127.0.0.1', port))
    # print(f"Sent: {msg}")

def main():
    print("Starting Dummy UDP Status Sender (Robot Simulation)...")
    print("Target: 127.0.0.1:8886")
    
    t = 0.0
    try:
        while True:
            # Create dummy angles (6 Slave + 1 Gripper + 6 Master + 1 Gripper = 14)
            # Vary slave_j1 and slave_j2 using sine waves
            s1 = 45.0 * math.sin(t)
            s2 = 30.0 * math.cos(t * 0.5)
            
            angles = [0.0] * 14
            angles[0] = s1
            angles[1] = s2
            
            send_udp_status(angles)
            
            if int(t * 10) % 10 == 0:
                print(f"Time: {t:.1f}s | Current J1: {s1:.1f} deg")
                
            t += 0.05 # 20Hz
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
