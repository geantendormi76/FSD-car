import os
import glob
import csv
import math
import numpy as np

def purify_and_hindsight():
    raw_dir = "/home/zhz/fsd-car/dataset"
    purified_dir = os.path.join(raw_dir, "purified")
    os.makedirs(purified_dir, exist_ok=True)
    
    csv_pattern = os.path.join(raw_dir, "spice_run_*.csv")
    csv_files = sorted(glob.glob(csv_pattern))
    
    if not csv_files:
        print("Error: No raw dataset files found.")
        return

    print("=====================================================================================")
    print("🛰️  NEXUS - Hindsight Goal Alignment & Purification (SOTA 2026)")
    print("=====================================================================================")
    print(f"{'File Name':<28} | {'Raw':<5} | {'Pauses':<6} | {'Purified':<8} | {'Max Vel':<8} | {'Align'}")
    print("-" * 82)

    for file_path in csv_files:
        file_name = os.path.basename(file_path)
        
        # Read all rows into memory
        rows = []
        with open(file_path, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames
            for row in reader:
                rows.append(row)
                
        if len(rows) < 50:
            # Skip corrupted/too short runs
            print(f"{file_name:<28} | {len(rows):<5} | {'-':<6} | {'0':<8} | {'-':<8} | SKIP ❌ (Too short)")
            continue
            
        # 1. Hindsight: Extract the absolute last frame as the Dynamic Goal
        last_row = rows[-1]
        goal_x = float(last_row['odom_x'])
        goal_y = float(last_row['odom_y'])
        
        purified_rows = []
        pause_count = 0
        max_vel = 0.0
        
        # 2. Recalculate homogeneous relative goals for every frame
        for row in rows:
            v = float(row['current_v'])
            cmd_v = float(row['cmd_v'])
            odom_x = float(row['odom_x'])
            odom_y = float(row['odom_y'])
            odom_yaw = float(row['odom_yaw'])
            
            dx = goal_x - odom_x
            dy = goal_y - odom_y
            
            # Recalculate ego-centric coordinates based on final pose of this run
            local_goal_x = dx * math.cos(odom_yaw) + dy * math.sin(odom_yaw)
            local_goal_y = -dx * math.sin(odom_yaw) + dy * math.cos(odom_yaw)
            local_goal_dist = math.sqrt(local_goal_x**2 + local_goal_y**2)
            
            if v > max_vel:
                max_vel = v
                
            # Filter out intermediate idle pauses
            if v < 0.05 and local_goal_dist > 0.35 and abs(cmd_v) < 0.01:
                pause_count += 1
                continue
                
            # Update row values
            row['local_goal_x'] = f"{local_goal_x:.6f}"
            row['local_goal_y'] = f"{local_goal_y:.6f}"
            row['local_goal_dist'] = f"{local_goal_dist:.6f}"
            purified_rows.append(row)
            
        if len(purified_rows) < 20:
            print(f"{file_name:<28} | {len(rows):<5} | {pause_count:<6} | {len(purified_rows):<8} | {max_vel:<8.3f} | EMPTY ❌")
            continue
            
        # 3. Write purified, hindsight-aligned file back to disk
        out_path = os.path.join(purified_dir, file_name)
        with open(out_path, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(purified_rows)
            
        print(f"{file_name:<28} | {len(rows):<5} | {pause_count:<6} | {len(purified_rows):<8} | {max_vel:<8.3f} | SUCCESS 🏆 (Aligned)")

    print("=====================================================================================")
    print("Hindsight dataset pre-processing completed. Stored at dataset/purified/")

if __name__ == "__main__":
    purify_and_hindsight()
