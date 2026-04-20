#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse

def main():
    parser = argparse.ArgumentParser(description='Plot GNG Distance Statistics')
    parser.add_argument('--file', type=str, default='gng_distance_stats.dat', help='Stats file path')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: {args.file} not found.")
        return

    # Column format:
    # 0: n_learning
    # 1: dist_s1_sq
    # 2: dist_s2_sq
    # 3: ema1_s1_sq
    # 4: ema1_s2_sq
    # 5: ema2_s1_sq
    # 6: ema2_s2_sq
    
    data = np.loadtxt(args.file)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    
    steps = data[:, 0]
    
    plt.figure(figsize=(12, 8))
    
    # Apply sqrt to all data columns except the first one (steps)
    plot_data = np.sqrt(data[:, 1:])
    
    # Plot raw data with transparency
    plt.plot(steps, plot_data[:, 0], color='blue', alpha=0.05, label='Raw S1 Dist')
    plt.plot(steps, plot_data[:, 1], color='green', alpha=0.05, label='Raw S2 Dist')
    
    # Plot 1st-order EMA (Thin lines)
    plt.plot(steps, plot_data[:, 2], color='blue', alpha=0.4, linewidth=1, label='EMA1 S1 Dist')
    plt.plot(steps, plot_data[:, 3], color='green', alpha=0.4, linewidth=1, label='EMA1 S2 Dist')
    
    # Plot 2nd-order EMA (Thick lines)
    plt.plot(steps, plot_data[:, 4], color='blue', linewidth=2.5, label='EMA2 S1 Dist (Final)')
    plt.plot(steps, plot_data[:, 5], color='green', linewidth=2.5, label='EMA2 S2 Dist (Final)')
    
    plt.title('GNG Convergence: RMS Distance to 1st & 2nd Winners', fontsize=16)
    plt.xlabel('Learning Iterations', fontsize=12)
    # plt.ylabel('Distance (Root Mean Square Error)', fontsize=12)
    plt.ylabel('Distance (RMS Error)', fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.ylim(0, 1.5) # Set Y-axis limit as requested
    plt.legend()
    # plt.yscale('log') # Removed log scale as requested
    
    # Save the plot
    output_filename = "gng_convergence_stats.png"
    plt.savefig(output_filename)
    print(f"Plot saved to: {output_filename}")
    
    plt.show()

if __name__ == '__main__':
    main()
