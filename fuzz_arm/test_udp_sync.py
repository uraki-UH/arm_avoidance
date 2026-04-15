import socket
import time
import math

def send_udp_command(joint_degrees, gripper_val):
    # Format: d1*10, d2*10, ..., d6*10, gripper_raw
    csv_data = ",".join([str(int(d * 10)) for d in joint_degrees])
    csv_data += f",{int(gripper_val)}"
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(csv_data.encode(), ("127.0.0.1", 12345))
    print(f"Sent: {csv_data}")

if __name__ == "__main__":
    # Example: Move joints slightly
    joints = [0.0, 10.0, -10.0, 5.0, -5.0, 0.0]
    gripper = -500 # Slightly closed
    
    print("Starting UDP Sync test...")
    for i in range(10):
        # Sine wave move for joint 1
        joints[0] = 20.0 * math.sin(i * 0.5)
        send_udp_command(joints, gripper)
        time.sleep(0.1)
