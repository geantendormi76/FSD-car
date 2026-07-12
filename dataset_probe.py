#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import glob
import csv
import math

def audit_dataset():
    dataset_dir = "/home/zhz/fsd-car/dataset"
    csv_files = sorted(glob.glob(os.path.join(dataset_dir, "spice_run_*.csv")))
    
    if not csv_files:
        print("❌ 致命错误：未在目录 /home/zhz/fsd-car/dataset 下找到任何 spice_run_*.csv 黄金数据集！")
        return

    print("=====================================================================================")
    print("🛰️  NEXUS - 10维黄金数据集时空自洽性深度审计探针 (SOTA 2026)")
    print("=====================================================================================")
    
    total_frames = 0
    total_fighting_frames = 0
    total_static_blind_frames = 0
    total_teleport_jumps = 0
    
    file_reports = []
    
    for file_path in csv_files:
        file_name = os.path.basename(file_path)
        rows = []
        with open(file_path, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
                
        if len(rows) < 10:
            continue
            
        # 审计单文件特征
        file_frames = len(rows)
        total_frames += file_frames
        
        fighting_count = 0
        static_blind_count = 0
        jump_count = 0
        
        prev_x, prev_y = None, None
        max_v = 0.0
        
        for i, row in enumerate(rows):
            v = float(row['current_v'])
            cmd_w = float(row['cmd_w'])
            f_y = float(row['frog_eye_fy'])
            x = float(row['odom_x'])
            y = float(row['odom_y'])
            g_dist = float(row['local_goal_dist'])
            
            if v > max_v:
                max_v = v
                
            # 1. 审计：力量对抗 (Fighting Force)
            # 如果青蛙眼有明显的侧向避障斥力 (abs(f_y) > 0.05)，但人类方向盘打舵方向 (cmd_w) 与斥力方向相反！
            # 这会导致神经网络在训练时发生因果混淆！
            if abs(f_y) > 0.05 and abs(cmd_w) > 0.05:
                if (f_y > 0 and cmd_w < 0) or (f_y < 0 and cmd_w > 0):
                    fighting_count += 1
                    total_fighting_frames += 1
            
            # 2. 审计：静态失盲 (Static Blindness)
            # 小车静止 (v < 0.01) 且距离目标还很远 (g_dist > 0.5)，此时青蛙眼避障力由于帧间静止全部丢失为 0
            if v < 0.01 and g_dist > 0.5 and abs(f_y) < 0.01:
                static_blind_count += 1
                total_static_blind_frames += 1
                
            # 3. 审计：未分断的瞬移跳变 (Teleportation Jumps)
            if prev_x is not None and prev_y is not None:
                dist_step = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                if dist_step > 2.0: # 单帧位移超过 2 米判定为未正常分割的重置跳变
                    jump_count += 1
                    total_teleport_jumps += 1
                    
            prev_x, prev_y = x, y
            
        fighting_ratio = (fighting_count / file_frames) * 100.0 if file_frames > 0 else 0.0
        static_blind_ratio = (static_blind_count / file_frames) * 100.0 if file_frames > 0 else 0.0
        
        file_reports.append({
            'name': file_name,
            'frames': file_frames,
            'max_v': max_v,
            'fight_ratio': fighting_ratio,
            'blind_ratio': static_blind_ratio,
            'jumps': jump_count
        })
        
    # 打印全局和单文件审计面板
    print(f"{'CSV 文件名':<36} | {'帧数':<5} | {'最大速度':<6} | {'力量对抗率':<10} | {'静态失盲率':<10} | {'瞬移跳变'}")
    print("-" * 92)
    for r in file_reports:
        print(f"{r['name']:<36} | {r['frames']:<5} | {r['max_v']:<6.2f} | {r['fight_ratio']:<9.1f}% | {r['blind_ratio']:<9.1f}% | {r['jumps']:<5}")
        
    print("=====================================================================================")
    print("📊 综合自洽性审计诊断报告:")
    print("=====================================================================================")
    print(f" 1. 扫描总帧数        : {total_frames} 帧")
    print(f" 2. 力量冲突帧数比例  : {total_fighting_frames} 帧 (占总数据集 {(total_fighting_frames/total_frames*100.0) if total_frames>0 else 0:.1f}%)")
    print(f"    -> 诊断：若该比例 > 15%，说明人类手动指令经常与青蛙眼冲突，模仿学习训练后极易“左右互搏”画圈！")
    print(f" 3. 静态失盲帧数比例  : {total_static_blind_frames} 帧 (占总数据集 {(total_static_blind_frames/total_frames*100.0) if total_frames>0 else 0:.1f}%)")
    print(f"    -> 诊断：若该比例 > 10%，模型在静止时将失去避障力，恢复前进时必然撞墙！")
    print(f" 4. 未切割瞬移异常点  : {total_teleport_jumps} 处")
    print(f"    -> 诊断：若存在跳变，说明重置小车时没有正常切分文件，这会导致训练轨迹发生剧烈的“拉扯致幻”！")
    print("=====================================================================================")

if __name__ == "__main__":
    audit_dataset()
