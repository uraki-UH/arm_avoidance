#!/usr/bin/env python3
"""
Full History Joint Angle Recorder and Visualizer
Records all joint angle data to a CSV file and provides playback visualization.
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse
import csv
from datetime import datetime
import os


class JointAngleRecorder:
    def __init__(self, port=12346, num_joints=6, output_dir="joint_data"):
        """
        Initialize the joint angle recorder.
        
        Args:
            port: UDP port to listen on
            num_joints: Number of robot joints
            output_dir: Directory to save recorded data
        """
        self.port = port
        self.num_joints = num_joints
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(output_dir, f"joint_angles_{timestamp}.csv")
        
        # Data storage (all history)
        self.time_data = []
        self.joint_data = [[] for _ in range(num_joints)]
        
        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.01)
        
        # Joint names
        self.joint_names = [f'Joint {i+1}' for i in range(num_joints)]
        
        # CSV file setup
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp'] + self.joint_names)
        
        # Setup plot
        self.setup_plot()
        
        print(f"Recording to: {self.csv_filename}")
        
    def setup_plot(self):
        """Setup matplotlib figure and subplots."""
        self.fig, self.axes = plt.subplots(self.num_joints, 1, figsize=(14, 10))
        self.fig.suptitle('Robot Joint Angles (Full History Recording)', fontsize=16, fontweight='bold')
        
        if self.num_joints == 1:
            self.axes = [self.axes]
        
        self.lines = []
        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.8)
            self.lines.append(line)
            
            ax.set_ylabel(f'{self.joint_names[i]}\n(rad)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-3.5, 3.5)
            
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
            # Save to CSV
            self.csv_writer.writerow([timestamp] + list(joint_angles))
            self.csv_file.flush()
            
            # Update data
            self.time_data.append(timestamp)
            for i, angle in enumerate(joint_angles):
                self.joint_data[i].append(angle)
            
            # Update plots (show all data)
            if len(self.time_data) > 1:
                time_array = np.array(self.time_data)
                time_array = time_array - time_array[0]  # Normalize to start at 0
                
                for i, line in enumerate(self.lines):
                    line.set_data(time_array, np.array(self.joint_data[i]))
                
                # Update x-axis limits to show all data
                if len(time_array) > 0:
                    max_time = time_array[-1]
                    for ax in self.axes:
                        ax.set_xlim(0, max(10, max_time + 1))
        
        return self.lines
    
    def run(self):
        """Start the recording and visualization."""
        print(f"Listening for joint angle data on UDP port {self.port}...")
        print("Press Ctrl+C to stop recording.")
        
        ani = FuncAnimation(self.fig, self.update, interval=10, blit=True, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nRecording stopped.")
        finally:
            self.csv_file.close()
            self.sock.close()
            print(f"Data saved to: {self.csv_filename}")
            print(f"Total data points: {len(self.time_data)}")


def playback_data(csv_file):
    """Playback recorded joint angle data from CSV file."""
    print(f"Loading data from: {csv_file}")
    
    # Load data
    data = np.loadtxt(csv_file, delimiter=',', skiprows=1)
    time_data = data[:, 0]
    joint_data = data[:, 1:]
    
    # Normalize time to start at 0
    time_data = time_data - time_data[0]
    
    num_joints = joint_data.shape[1]
    joint_names = [f'Joint {i+1}' for i in range(num_joints)]
    
    # Create plot
    fig, axes = plt.subplots(num_joints, 1, figsize=(14, 10))
    fig.suptitle(f'Joint Angles Playback: {os.path.basename(csv_file)}', fontsize=16, fontweight='bold')
    
    if num_joints == 1:
        axes = [axes]
    
    for i, ax in enumerate(axes):
        ax.plot(time_data, joint_data[:, i], 'b-', linewidth=1.5, alpha=0.8)
        ax.set_ylabel(f'{joint_names[i]}\n(rad)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-3.5, 3.5)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    plt.show()
    
    print(f"Total duration: {time_data[-1]:.2f} seconds")
    print(f"Total data points: {len(time_data)}")


def main():
    parser = argparse.ArgumentParser(description='Record and visualize robot joint angles')
    parser.add_argument('--port', type=int, default=12346, help='UDP port to listen on')
    parser.add_argument('--joints', type=int, default=6, help='Number of robot joints')
    parser.add_argument('--output-dir', type=str, default='joint_data', help='Output directory for CSV files')
    parser.add_argument('--playback', type=str, help='Playback a recorded CSV file')
    
    args = parser.parse_args()
    
    if args.playback:
        playback_data(args.playback)
    else:
        recorder = JointAngleRecorder(
            port=args.port,
            num_joints=args.joints,
            output_dir=args.output_dir
        )
        
        try:
            recorder.run()
        except KeyboardInterrupt:
            print("\nRecording stopped.")


if __name__ == '__main__':
    main()
