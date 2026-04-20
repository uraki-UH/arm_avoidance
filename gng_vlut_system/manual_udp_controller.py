import socket
import curses
import time

# --- Configuration ---
TARGET_IP = "192.168.4.40"
TARGET_PORT = 8888
NUM_SERVOS = 14
ANGLE_DELTA = 25  # degree * 10 delta

# --- State ---
current_angles = [0] * NUM_SERVOS
inc_mode = True  # True: Increment, False: Decrement

def generate_msg(angles):
    """Format: j1,j2,j3,...,j14,"""
    return ",".join(map(str, angles)) + ","

def main(stdscr):
    global inc_mode
    
    # Setup Curses
    curses.curs_set(0) # Hide cursor
    stdscr.nodelay(True) # Non-blocking input
    stdscr.timeout(50)   # 50ms loop

    # Setup Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    msg_y, msg_x = 10, 2
    
    while True:
        stdscr.clear()
        stdscr.addstr(1, 2, "=== ToPoArm Python Manual Controller ===", curses.A_BOLD)
        stdscr.addstr(2, 2, f"Target: {TARGET_IP}:{TARGET_PORT}")
        stdscr.addstr(4, 2, "Controls:")
        stdscr.addstr(5, 4, "1-7 : Adjust Slave Joints (8-9,0: Master)")
        stdscr.addstr(6, 4, "q   : Mode -> INCREMENT (+)")
        stdscr.addstr(7, 4, "w   : Mode -> DECREMENT (-)")
        stdscr.addstr(8, 4, "r/0 : Reset all angles to 0")
        stdscr.addstr(9, 4, "ESC : Quit")
        
        mode_str = "INCREMENT (+)" if inc_mode else "DECREMENT (-)"
        stdscr.addstr(11, 2, f"CURRENT MODE: {mode_str}", curses.A_REVERSE)
        
        # Display Angles
        stdscr.addstr(13, 2, "Angles (deg*10):")
        half = NUM_SERVOS // 2
        stdscr.addstr(14, 4, "Slave:  " + " ".join(f"{v:4}" for v in current_angles[:half]))
        stdscr.addstr(15, 4, "Master: " + " ".join(f"{v:4}" for v in current_angles[half:]))

        # Handle Keyboard
        try:
            key = stdscr.getch()
        except:
            key = -1

        if key == 27: # ESC
            break
        elif key == ord('q'):
            inc_mode = True
        elif key == ord('w'):
            inc_mode = False
        elif key == ord('r') or key == ord('0'):
            current_angles[:] = [0] * NUM_SERVOS
        elif ord('1') <= key <= ord('9'):
            idx = key - ord('1')
            if idx < NUM_SERVOS:
                current_angles[idx] += ANGLE_DELTA if inc_mode else -ANGLE_DELTA
        
        # Send Packet periodically
        msg = generate_msg(current_angles)
        sock.sendto(msg.encode('utf-8'), (TARGET_IP, TARGET_PORT))
        
        stdscr.addstr(17, 2, f"Last Sent: {msg}")
        stdscr.refresh()

    sock.close()

if __name__ == "__main__":
    curses.wrapper(main)
