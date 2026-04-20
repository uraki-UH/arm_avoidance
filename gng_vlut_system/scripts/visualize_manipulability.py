#!/usr/bin/env python3
"""
Real-time Manipulability Monitoring Tool
Monitors manipulability metrics from CSV file and displays them in real-time graphs.
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import argparse


class ManipulabilityVisualizer:
    def __init__(self, csv_file="manipulability_data.csv", history_length=10000):
        """
        Initialize the manipulability visualizer.
        
        Args:
            csv_file: Path to CSV file containing manipulability data
            history_length: Number of data points to keep in history
        """
        self.csv_file = csv_file
        self.history_length = history_length
        
        # Data storage
        self.time_data = deque(maxlen=history_length)
        self.manipulability_data = deque(maxlen=history_length)
        self.condition_number_data = deque(maxlen=history_length)
        self.min_singular_data = deque(maxlen=history_length)
        self.manip_x_data = deque(maxlen=history_length)
        self.manip_y_data = deque(maxlen=history_length)
        self.manip_z_data = deque(maxlen=history_length)
        
        # File monitoring
        self.last_position = 0
        self.file_exists = False
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup matplotlib figure and subplots."""
        self.fig, self.axes = plt.subplots(4, 1, figsize=(14, 10))
        self.fig.suptitle('Robot Manipulability Metrics (Real-time)', fontsize=16, fontweight='bold')
        
        # Metric names and colors
        metrics = [
            ("Manipulability (μ)", "blue", 0.01),
            ("Condition Number (κ)", "orange", 100),
            ("Min Singular Value (σ_min)", "green", 0.001),
            ("Directional Manipulability", None, None),
        ]
        
        self.lines = []
        self.warning_lines = []
        
        for i, (name, color, threshold) in enumerate(metrics):
            ax = self.axes[i]
            
            # Main data line(s)
            if name == "Directional Manipulability":
                line_x, = ax.plot([], [], color="red", linewidth=1.5, label="X")
                line_y, = ax.plot([], [], color="green", linewidth=1.5, label="Y")
                line_z, = ax.plot([], [], color="blue", linewidth=1.5, label="Z")
                self.lines.extend([line_x, line_y, line_z])
            else:
                line, = ax.plot([], [], color=color, linewidth=2, label=name)
                self.lines.append(line)
            
            ax.set_ylabel(name, fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')
            
            # Add warning threshold line
            if threshold is not None:
                warning_line = ax.axhline(y=threshold, color='red', linestyle='--', 
                                         alpha=0.5, linewidth=1.5, label=f'Warning: {threshold}')
                self.warning_lines.append(warning_line)
            else:
                self.warning_lines.append(None)
            
            # Set y-axis limits (will be adjusted dynamically)
            if i == 0:  # Manipulability
                ax.set_ylim(0, 0.1)
            elif i == 1:  # Condition number
                ax.set_ylim(1, 10)  # Condition number is always >= 1
            elif i == 2:  # Min singular value
                ax.set_ylim(0, 0.1)
            else:  # Directional
                ax.set_ylim(0, 0.5)
        
        self.axes[-1].set_xlabel('Time (s)', fontsize=12)
        
        plt.tight_layout()
        
    def read_new_data(self):
        """Read new data from CSV file (tail -f style)."""
        if not os.path.exists(self.csv_file):
            if not self.file_exists:
                # Only print once
                self.file_exists = False
            return False
        
        if not self.file_exists:
            print(f"Found CSV file: {self.csv_file}")
            self.file_exists = True
        
        try:
            with open(self.csv_file, 'r') as f:
                # Seek to last known position
                f.seek(self.last_position)
                
                # Read new lines
                new_lines = f.readlines()
                self.last_position = f.tell()
                
                # Process new data
                for line in new_lines:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    try:
                        parts = line.split(',')
                        if len(parts) >= 5:
                            timestamp = float(parts[0])
                            manipulability = float(parts[1])
                            condition_number = float(parts[2])
                            min_singular = float(parts[3])
                            # parts[4] is ellipsoid_volume
                            manip_x = float(parts[5])
                            manip_y = float(parts[6])
                            manip_z = float(parts[7])
                            
                            self.time_data.append(timestamp)
                            self.manipulability_data.append(manipulability)
                            self.condition_number_data.append(condition_number)
                            self.min_singular_data.append(min_singular)
                            self.manip_x_data.append(manip_x)
                            self.manip_y_data.append(manip_y)
                            self.manip_z_data.append(manip_z)
                    except (ValueError, IndexError) as e:
                        # Skip malformed lines
                        continue
                
                return len(new_lines) > 0
        except Exception as e:
            print(f"Error reading file: {e}")
            return False
    
    def update(self, frame):
        """Update function for animation."""
        self.read_new_data()
        
        if len(self.time_data) > 1:
            time_array = np.array(self.time_data)
            time_array = time_array - time_array[0]  # Normalize to start at 0
            
            # Update data for each metric
            data_arrays = [
                np.array(self.manipulability_data),
                np.array(self.condition_number_data),
                np.array(self.min_singular_data),
            ]
            
            for i, (line, data) in enumerate(zip(self.lines, data_arrays)):
                line.set_data(time_array, data)
                
                # Auto-adjust y-axis limits with better margin handling
                if len(data) > 0:
                    # Filter out extreme outliers for better scaling (especially for CondNum 1e10)
                    valid_data = data[data < 1e5]
                    if len(valid_data) == 0:
                        valid_data = data
                        
                    data_min, data_max = np.min(valid_data), np.max(valid_data)
                    data_range = data_max - data_min
                    
                    # Use at least 15% margin
                    margin = max(data_range * 0.15, 0.1)
                    
                    # Condition number must be >= 1 by definition
                    if i == 1:  # Condition number
                        y_min = 1.0
                        # Cap the max display range for condition number to focus on useful range
                        y_max = min(100.0, data_max + margin)
                        if y_max < data_max: # If we are already above 100, show the data
                             y_max = data_max + margin
                    else:
                        y_min = max(0, data_min - margin)
                        y_max = data_max + margin
                    
                    # Ensure minimum range for visibility
                    if y_max - y_min < 0.01:
                        y_max = y_min + 0.01
                    
                    self.axes[i].set_ylim(y_min, y_max)
            
            # Special handling for 4th subplot (Directional)
            if len(self.manip_x_data) > 0:
                y_data_x = np.array(self.manip_x_data)
                y_data_y = np.array(self.manip_y_data)
                y_data_z = np.array(self.manip_z_data)
                
                # Update lines 3, 4, 5 (X, Y, Z)
                self.lines[3].set_data(time_array, y_data_x)
                self.lines[4].set_data(time_array, y_data_y)
                self.lines[5].set_data(time_array, y_data_z)
                
                # Auto-scale directional plot
                all_directional = np.concatenate([y_data_x, y_data_y, y_data_z])
                d_min, d_max = np.min(all_directional), np.max(all_directional)
                d_margin = max((d_max - d_min) * 0.15, 0.05)
                self.axes[3].set_ylim(max(0, d_min - d_margin), d_max + d_margin)
            
            # Update x-axis limits to show recent 100 seconds
            if len(time_array) > 0:
                max_time = time_array[-1]
                for ax in self.axes:
                    ax.set_xlim(max(0, max_time - 100.0), max_time + 0.5)
        
        return self.lines
    
    def run(self):
        """Start the visualization."""
        print(f"Monitoring manipulability data from: {self.csv_file}")
        print("Waiting for data...")
        print("Press Ctrl+C to stop.")
        
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=True, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nVisualization stopped.")


def main():
    parser = argparse.ArgumentParser(description='Real-time manipulability monitoring')
    parser.add_argument('--file', type=str, default='manipulability_data.csv', 
                       help='CSV file to monitor')
    parser.add_argument('--history', type=int, default=10000, 
                       help='Number of data points to display')
    
    args = parser.parse_args()
    
    visualizer = ManipulabilityVisualizer(
        csv_file=args.file,
        history_length=args.history
    )
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nVisualization stopped.")


if __name__ == '__main__':
    main()
