#!/usr/bin/env python3
"""
Real-time Joint Angle Visualization Tool
Receives robot joint angle data via UDP and displays it in real-time graphs.
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import argparse


class JointAngleVisualizer:
    def __init__(self, port=12346, num_joints=6, history_length=10000):
        """
        Initialize the joint angle visualizer.
        
        Args:
            port: UDP port to listen on
            num_joints: Number of robot joints
            history_length: Number of data points to keep in history
        """
        self.port = port
        self.num_joints = num_joints
        self.history_length = history_length
        
        # Data storage
        self.time_data = deque(maxlen=history_length)
        self.joint_data = [deque(maxlen=history_length) for _ in range(num_joints)]
        self.current_time = 0
        
        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.01)  # Non-blocking with short timeout
        
        # Joint names
        self.joint_names = [f'Joint {i+1}' for i in range(num_joints)]
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup matplotlib figure and subplots."""
        self.fig, self.axes = plt.subplots(self.num_joints, 1, figsize=(12, 10))
        self.fig.suptitle('Robot Joint Angles (Real-time)', fontsize=16, fontweight='bold')
        
        if self.num_joints == 1:
            self.axes = [self.axes]
        
        self.lines = []
        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], 'b-', linewidth=2)
            self.lines.append(line)
            
            ax.set_ylabel(f'{self.joint_names[i]}\n(rad)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, self.history_length * 0.01)  # Assuming 100Hz update
            ax.set_ylim(-3.5, 3.5)  # Typical joint range
            
            # Add horizontal line at 0
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        self.axes[-1].set_xlabel('Time (s)', fontsize=12)
        
        plt.tight_layout()
        
    def receive_data(self):
        """Receive joint angle data from UDP socket."""
        try:
            data, addr = self.sock.recvfrom(1024)
            
            # Expected format: timestamp (double) + num_joints * joint_angles (float)
            expected_size = 8 + 4 * self.num_joints
            if len(data) == expected_size:
                # Unpack: 1 double (timestamp) + num_joints floats (joint angles)
                format_str = 'd' + 'f' * self.num_joints
                values = struct.unpack(format_str, data)
                
                timestamp = values[0]
                joint_angles = values[1:]
                
                return timestamp, joint_angles
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Error receiving data: {e}")
        
        return None, None
    
    def update(self, frame):
        """Update function for animation."""
        timestamp, joint_angles = self.receive_data()
        
        if joint_angles is not None:
            # Update data
            self.time_data.append(timestamp)
            for i, angle in enumerate(joint_angles):
                self.joint_data[i].append(angle)
            
            # Update plots
            if len(self.time_data) > 1:
                time_array = np.array(self.time_data)
                time_array = time_array - time_array[0]  # Normalize to start at 0
                
                for i, line in enumerate(self.lines):
                    line.set_data(time_array, np.array(self.joint_data[i]))
                
                # Update x-axis limits to show recent data
                if len(time_array) > 0:
                    max_time = time_array[-1]
                    for ax in self.axes:
                        ax.set_xlim(max(0, max_time - 100.0), max_time + 0.5)
        
        return self.lines
    
    def run(self):
        """Start the visualization."""
        print(f"Listening for joint angle data on UDP port {self.port}...")
        print("Press Ctrl+C to stop.")
        
        ani = FuncAnimation(self.fig, self.update, interval=10, blit=True, cache_frame_data=False)
        plt.show()
        
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description='Real-time robot joint angle visualization')
    parser.add_argument('--port', type=int, default=12346, help='UDP port to listen on')
    parser.add_argument('--joints', type=int, default=6, help='Number of robot joints')
    parser.add_argument('--history', type=int, default=10000, help='Number of data points to display')
    
    args = parser.parse_args()
    
    visualizer = JointAngleVisualizer(
        port=args.port,
        num_joints=args.joints,
        history_length=args.history
    )
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nVisualization stopped.")


if __name__ == '__main__':
    main()
