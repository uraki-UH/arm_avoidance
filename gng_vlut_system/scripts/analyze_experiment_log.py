#!/usr/bin/env python3
import csv
import sys
import glob
import os
from datetime import datetime
import statistics

def parse_args():
    if len(sys.argv) > 1:
        return sys.argv[1]
    
    # Default to finding the latest log in experiment_logs (checking current and parent dirs)
    search_patterns = [
        "experiment_logs/*.csv",
        "../experiment_logs/*.csv",
        "experiment_log.csv",
        "../experiment_log.csv"
    ]
    
    log_files = []
    for pattern in search_patterns:
        log_files.extend(glob.glob(pattern))
    
    if not log_files:
        print("Error: No experiment log files found.")
        sys.exit(1)
        
    # Sort by modification time
    latest_log = max(log_files, key=os.path.getmtime)
    print(f"[Info] Analyzing latest log: {latest_log}")
    return latest_log

def calculate_metrics(csv_file):
    events = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                events.append(row)
    except FileNotFoundError:
        print(f"Error: File {csv_file} not found.")
        sys.exit(1)
    
    if not events:
        print("No events found in log.")
        return

    # 1. Group by Stage
    stages = {}
    experiment_start_time = None
    experiment_end_time = None
    
    # Helper to parse time
    def parse_time(t_str):
        try:
            return datetime.strptime(t_str, "%Y-%m-%d %H:%M:%S")
        except ValueError:
            return None

    # Track transition times
    last_stage_end_time = experiment_start_time
    
    for row in events:
        stage_name = row['Stage']
        event_type = row['Event']
        timestamp = parse_time(row['Timestamp'])
        
        if timestamp is None:
            continue

        if event_type == "EXPERIMENT_START":
            experiment_start_time = timestamp
            last_stage_end_time = timestamp
        elif event_type == "EXPERIMENT_COMPLETE":
            experiment_end_time = timestamp
        
        # Initialize stage metrics container if new
        if stage_name not in stages:
            stages[stage_name] = {
                'start_time': last_stage_end_time, # Default start is end of last stage
                'end_time': None,
                'reached': False,
                'replanning_count': 0,
                'planning_times': [],
                'collision_checks': [],
                'total_check_times': []
            }
        
        # If this is the very first event for this stage and start_time is None/old, update it?
        # Actually, best logic: Stage N start = Stage N-1 end.
        # But if we parallelize or have gaps, we might want the first log timestamp of that stage.
        # Let's stick to "Stage N starts when N-1 ended or Exp started".
        
        if event_type == "TARGET_REACHED":
            stages[stage_name]['end_time'] = timestamp
            stages[stage_name]['reached'] = True
            last_stage_end_time = timestamp # Update for next stage
            
        elif event_type == "PLAN_SUCCESS":
            stages[stage_name]['replanning_count'] += 1
            try:
                stages[stage_name]['planning_times'].append(float(row['PlanTimeMs']))
                stages[stage_name]['collision_checks'].append(int(row['CollChecks']))
                stages[stage_name]['total_check_times'].append(float(row['TotalCheckTimeUs']))
            except (ValueError, KeyError):
                pass
    
    # 4. Save Report to File
    output_dir = "evaluate/experiment_result"
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
        except OSError as e:
            # Fallback for when running from build dir or elsewhere
            if os.path.exists("../evaluate"):
                output_dir = "../evaluate/experiment_result"
                os.makedirs(output_dir, exist_ok=True)
            else:
                # Last resort: create in current dir
                output_dir = "experiment_result"
                os.makedirs(output_dir, exist_ok=True)

    base_name = os.path.basename(csv_file).replace('.csv', '')
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_filename = f"{output_dir}/report_{base_name}_{timestamp_str}.txt"
    # 1. Calculate Totals
    total_stages = 0
    success_stages = 0
    total_time_diffs = []
    
    for stage, data in stages.items():
        if stage == "UNKNOWN" or stage == "COMPLETED": continue
        total_stages += 1
        if data['reached']:
            success_stages += 1
            if data['start_time'] and data['end_time']:
                 total_time_diffs.append((data['end_time'] - data['start_time']).total_seconds())

    success_rate = (success_stages / total_stages * 100) if total_stages > 0 else 0.0
    avg_completion_time = statistics.mean(total_time_diffs) if total_time_diffs else 0.0
    
    start_str = experiment_start_time.strftime('%Y-%m-%d %H:%M:%S') if experiment_start_time else "N/A"
    end_str = experiment_end_time.strftime('%Y-%m-%d %H:%M:%S') if experiment_end_time else "N/A"
    total_duration = "N/A"
    if experiment_start_time and experiment_end_time:
        total_duration = f"{(experiment_end_time - experiment_start_time).total_seconds():.2f} s"

    # 2. Write to File
    with open(report_filename, 'w') as f:
        def write_line(line=""):
            print(line)
            f.write(line + "\n")
            
        write_line("\n" + "="*80)
        write_line(f" EXPERIMENT ANALYSIS REPORT: {csv_file}")
        write_line("="*80)
        
        write_line(f"\n{'STAGE':<25} | {'RESULT':<10} | {'TIME(s)':<8} | {'REPLANS':<8} | {'AVG PLAN(ms)':<12} | {'CHECKS':<8}")
        write_line("-" * 85)
        
        for stage, data in stages.items():
            if stage == "UNKNOWN" or stage == "COMPLETED": continue
            
            result_str = "SUCCESS" if data['reached'] else "TIMEOUT/FAIL"
            
            time_str = "-"
            if data['start_time'] and data['end_time']:
                delta = (data['end_time'] - data['start_time']).total_seconds()
                time_str = f"{delta:.2f}"
                
            avg_plan = 0.0
            if data['planning_times']:
                avg_plan = statistics.mean(data['planning_times'])
                
            avg_checks = 0
            if data['collision_checks']:
                avg_checks = int(statistics.mean(data['collision_checks']))
                
            write_line(f"{stage:<25} | {result_str:<10} | {time_str:<8} | {data['replanning_count']:<8} | {avg_plan:<12.2f} | {avg_checks:<8}")
            
        write_line("-" * 85)
        write_line(f"OVERALL SUMMARY")
        write_line(f"  Success Rate:        {success_rate:.1f}% ({success_stages}/{total_stages})")
        write_line(f"  Avg Completion Time: {avg_completion_time:.2f} s")
        write_line(f"  Experiment Duration: {total_duration} ({start_str} -> {end_str})")
        write_line("="*80 + "\n")
        
    print(f"\n[Info] Report saved to: {report_filename}")

if __name__ == "__main__":
    csv_file = parse_args()
    calculate_metrics(csv_file)
