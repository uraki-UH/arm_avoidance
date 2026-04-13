#include "data_utils.hpp"
#include "hardware/udp_comm.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

// For raw terminal mode on Unix-like systems
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

// Settings
const std::string TARGET_IP = "192.168.4.40";
const int TARGET_PORT = 8888;
const int LISTEN_PORT = 8886;
const int NUM_SERVOS = 14;
const int ANGLE_DELTA = 25; // Smaller delta for more fine-grained control

// Shared State
std::vector<int> current_angles(NUM_SERVOS, 0);
std::mutex angles_mutex;
std::atomic<bool> running(true);
std::atomic<bool> auto_mode(false); // false = Manual, true = Auto
std::atomic<bool> is_increment_mode(true); // true = increment (q), false = decrement (w)


// --- Terminal Raw Mode Management ---
struct termios orig_termios;

void disableRawMode() {
  // Restore original terminal settings
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
  // Save original terminal settings
  tcgetattr(STDIN_FILENO, &orig_termios);
  // Register disableRawMode to be called on exit
  atexit(disableRawMode);

  struct termios raw = orig_termios;
  // ICANON: disable canonical mode (line-by-line)
  // ECHO: disable echoing of input characters
  raw.c_lflag &= ~(ECHO | ICANON);
  // Apply the new settings
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}
// ------------------------------------


// Loop to send data at a fixed rate
void senderLoop(UdpSender &sender) {
  while (running) {
    std::vector<int> angles_to_send;
    {
      std::lock_guard<std::mutex> lock(angles_mutex);
      angles_to_send = current_angles;
    }

    std::string msg = generateSendData(angles_to_send);
    sender.send(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz send rate
  }
}

void receiverLoop(UdpReceiver &receiver) {
  while (running) {
    std::string msg = receiver.receive();
    if (!msg.empty()) {
      // Optional: process received data
    }
  }
}

// Function to print the status line
void printStatus() {
    printf("\r[MODE:%s | ADJ:%s] Angles: ",
           (auto_mode ? "AUTO" : "MANUAL"),
           (is_increment_mode ? "INC (q)" : "DEC (w)"));

    // Print the first 7 joint angles
    {
        std::lock_guard<std::mutex> lock(angles_mutex);
        for(int i = 0; i < 7; ++i) {
            printf("%d ", current_angles[i]);
        }
    }
    printf("     "); // Padding to clear previous longer outputs
    fflush(stdout);
}

// Loop to handle user keyboard input
void inputLoop() {
  std::cout << "=== V-Sido UDP Controller (Press & Hold) ===\n";
  std::cout << "Hold 1-7 to control joints. Press other keys to change modes.\n";
  std::cout << "  a   : Toggle Auto/Manual Mode\n";
  std::cout << "  q   : Set mode to INCREMENT angle\n";
  std::cout << "  w   : Set mode to DECREMENT angle\n";
  std::cout << "  1-7 : Adjust angle for the corresponding joint (HOLD to repeat)\n";
  std::cout << "  0/r : Reset all joints to 0\n";
  std::cout << "  ESC : Quit\n";
  std::cout << "----------------------------------\n" << std::endl;

  printStatus();

  char c;
  // Read one character at a time
  while (running && read(STDIN_FILENO, &c, 1) == 1) {
    // Check for ESC key to quit (ESC is \x1b)
    if (c == '\x1b') {
      running = false;
      break;
    }

    if (auto_mode && (c >= '1' && c <= '7')) {
        // Ignore joint commands in auto mode, but allow other commands
    } else {
        if (c == 'a') {
            auto_mode = !auto_mode;
        } else if (c == 'q') {
            is_increment_mode = true;
        } else if (c == 'w') {
            is_increment_mode = false;
        } else if (c >= '1' && c <= '7') {
            int joint_to_change = c - '1';
            std::lock_guard<std::mutex> lock(angles_mutex);
            if (is_increment_mode) {
                current_angles[joint_to_change] += ANGLE_DELTA;
            } else {
                current_angles[joint_to_change] -= ANGLE_DELTA;
            }
        } else if (c == '0' || c == 'r') {
            std::lock_guard<std::mutex> lock(angles_mutex);
            std::fill(current_angles.begin(), current_angles.end(), 0);
        }
    }
    
    // Refresh the status line after every key press
    printStatus();
  }
}

int main() {
  enableRawMode(); // Switch to raw terminal mode

  try {
    UdpSender sender(TARGET_IP, TARGET_PORT);
    UdpReceiver receiver(LISTEN_PORT);

    // Start background threads
    std::thread t_send(senderLoop, std::ref(sender));
    std::thread t_recv(receiverLoop, std::ref(receiver));

    // The main thread will handle user input
    inputLoop();

    // Cleanup
    running = false; // Signal threads to stop
    t_send.join();
    t_recv.detach();

  } catch (const std::exception &e) {
    // disableRawMode() is called automatically via atexit()
    std::cerr << "\nException: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\nExiting controller." << std::endl;
  return 0;
}
