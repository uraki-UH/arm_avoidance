#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import glob
import os
import sys

def get_latest_log():
    log_dirs = glob.glob("experiment_logs/run_*")
    if not log_dirs:
        print("Error: No experiment logs found in experiment_logs/")
        return None
    latest_run = max(log_dirs, key=os.path.getmtime)
    csv_path = os.path.join(latest_run, "log_data.csv")
    if not os.path.exists(csv_path):
        print(f"Error: log_data.csv not found in {latest_run}")
        return None
    return csv_path

def analyze_performance(csv_path):
    print(f"Analyzing: {csv_path}")
    df = pd.read_csv(csv_path)
    
    # 1. Basic Stats
    print("\n--- Basic Statistics ---")
    metrics = ['env_update_ms', 'safety_update_ms', 'node_count', 'touched_node_count']
    print(df[metrics].describe().to_string())
    
    # 2. Plots
    output_dir = os.path.dirname(csv_path)
    
    # Plot 1: Update Times over Time
    plt.figure(figsize=(12, 6))
    plt.plot(df['timestamp'], df['env_update_ms'], label='Env Update (ms)', alpha=0.7)
    plt.plot(df['timestamp'], df['safety_update_ms'], label='Safety Update (ms)', alpha=0.7)
    plt.title('Update Computation Times')
    plt.xlabel('Simulation Time (s)')
    plt.ylabel('Computation Time (ms)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'update_times.png'))
    plt.close()
    
    # Plot 2: Node Counts over Time
    plt.figure(figsize=(12, 6))
    plt.plot(df['timestamp'], df['node_count'], label='Active Nodes', alpha=0.7)
    plt.plot(df['timestamp'], df['touched_node_count'], label='Touched Nodes (this frame)', alpha=0.7)
    plt.title('GNG Node Activity')
    plt.xlabel('Simulation Time (s)')
    plt.ylabel('Node Count')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'node_activity.png'))
    plt.close()
    
    # Plot 3: Correlation (Touched Nodes vs Safety Time)
    plt.figure(figsize=(10, 6))
    sns.scatterplot(data=df, x='touched_node_count', y='safety_update_ms', alpha=0.5)
    plt.title('Correlation: Touched Nodes vs Safety Update Time')
    plt.xlabel('Touched Node Count')
    plt.ylabel('Safety Update Time (ms)')
    plt.grid(True, alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'correlation_touched_vs_time.png'))
    plt.close()
    
    print(f"\nPlots saved to: {output_dir}")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else get_latest_log()
    if path:
        analyze_performance(path)
