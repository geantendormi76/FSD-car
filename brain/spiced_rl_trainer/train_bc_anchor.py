import os
import glob
import csv
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

class SpicedDataset(Dataset):
    def __init__(self, dataset_dir):
        self.inputs = []
        self.actions = []
        csv_pattern = os.path.join(dataset_dir, "spice_run_*.csv")
        csv_files = sorted(glob.glob(csv_pattern))
        if not csv_files:
            raise FileNotFoundError(f"No purified CSV files found in {dataset_dir}")
        print(f"Loading and rebuilding {len(csv_files)} files with 5-frame sliding window...")
        
        v_max = 0.8
        kappa_max = 1.25 # rad/m (w_max / v_max = 1.0 / 0.8)

        for file_path in csv_files:
            rows = []
            with open(file_path, mode='r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    rows.append(row)
            if len(rows) < 300:
                continue
            max_vel = max([float(r['current_v']) for r in rows])
            if max_vel < 0.20:
                continue
            start_idx = 0
            for idx, r in enumerate(rows):
                if float(r['current_v']) > 0.05 or abs(float(r['cmd_v'])) > 0.01:
                    start_idx = idx
                    break
            end_idx = len(rows) - 1
            for idx in range(len(rows) - 1, -1, -1):
                r = rows[idx]
                if float(r['current_v']) > 0.05 or float(r['local_goal_dist']) > 0.20:
                    end_idx = idx
                    break
            if end_idx - start_idx < 150:
                continue
            valid_rows = rows[start_idx:end_idx + 1]
            
            for t in range(4, len(valid_rows)):
                obs_25 = []
                for d in range(5):
                    r_hist = valid_rows[t - (4 - d)]
                    l_goal_x = float(r_hist['local_goal_x']) * 0.20
                    l_goal_y = float(r_hist['local_goal_y']) * 0.20
                    l_goal_dist = float(r_hist['local_goal_dist']) * 0.20
                    f_x = float(r_hist['frog_eye_fx'])
                    f_y = float(r_hist['frog_eye_fy'])
                    obs_25.extend([l_goal_x, l_goal_y, l_goal_dist, f_x, f_y])
                
                cmd_v = float(valid_rows[t]['cmd_v'])
                cmd_w = float(valid_rows[t]['cmd_w'])
                
                # Apply Physics-Aware Action Mapping (PAM) normalization
                norm_v = np.clip(cmd_v / v_max, 0.0, 1.0)
                denom = max(cmd_v, 0.01)
                raw_kappa = cmd_w / denom
                norm_kappa = np.clip(raw_kappa / kappa_max, -1.0, 1.0)
                
                act = np.array([norm_v, norm_kappa], dtype=np.float32)
                self.inputs.append(np.array(obs_25, dtype=np.float32))
                self.actions.append(act)
                
        self.inputs = np.array(self.inputs)
        self.actions = np.array(self.actions)
        print(f"Sliding window dataset built. Total training samples: {len(self.inputs)}")

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        return torch.from_numpy(self.inputs[idx]), torch.from_numpy(self.actions[idx])

class GaussianPolicy(nn.Module):
    def __init__(self, input_dim=25, action_dim=2):
        super(GaussianPolicy, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh()
        )
        self.mean_head = nn.Linear(128, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, x):
        features = self.fc(x)
        raw_mean = self.mean_head(features)
        
        # Output 0: Platform-agnostic velocity scale [0, 1.0]
        mean_v = torch.sigmoid(raw_mean[:, 0:1])
        # Output 1: Platform-agnostic normalized curvature [-1.0, 1.0]
        mean_w = torch.tanh(raw_mean[:, 1:2])
        
        mean = torch.cat([mean_v, mean_w], dim=1)
        return mean, self.log_std.expand_as(mean)

class SpicedBrainInference(nn.Module):
    def __init__(self, policy):
        super(SpicedBrainInference, self).__init__()
        self.policy = policy

    def forward(self, x):
        mean, _ = self.policy(x)
        return mean

def train_bc():
    dataset_dir = "/home/zhz/fsd-car/dataset/purified"
    model_dir = "/home/zhz/fsd-car/model"
    os.makedirs(model_dir, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training on device: {device}")
    
    full_dataset = SpicedDataset(dataset_dir)
    train_size = int(0.9 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(full_dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=2048, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=2048, shuffle=False)
    
    policy = GaussianPolicy().to(device)
    optimizer = optim.Adam(policy.parameters(), lr=3e-4)
    best_val_loss = float('inf')
    epochs = 300
    
    print("Beginning epoch-loop training (25D Sliding Window & Steering Balanced)...")
    for epoch in range(1, epochs + 1):
        policy.train()
        train_loss = 0.0
        for obs, act in train_loader:
            obs, act = obs.to(device), act.to(device)
            optimizer.zero_grad()
            mean, log_std = policy(obs)
            
            # Penalize steering errors heavily based on curvature channel
            steer_weight = torch.where(torch.abs(act[:, 1]) > 0.02, 5.0, 1.0)
            var = torch.exp(2 * log_std)
            nll_elementwise = 0.5 * (((act - mean) ** 2) / var + 2 * log_std + np.log(2 * np.pi)).sum(dim=1)
            nll = (nll_elementwise * steer_weight).mean()
            nll.backward()
            optimizer.step()
            train_loss += nll.item() * obs.size(0)
        train_loss /= len(train_loader.dataset)
        
        policy.eval()
        val_loss = 0.0
        with torch.no_grad():
            for obs, act in val_loader:
                obs, act = obs.to(device), act.to(device)
                mean, log_std = policy(obs)
                steer_weight = torch.where(torch.abs(act[:, 1]) > 0.02, 5.0, 1.0)
                var = torch.exp(2 * log_std)
                nll_elementwise = 0.5 * (((act - mean) ** 2) / var + 2 * log_std + np.log(2 * np.pi)).sum(dim=1)
                nll = (nll_elementwise * steer_weight).mean()
                val_loss += nll.item() * obs.size(0)
        val_loss /= len(val_loader.dataset)
        
        if epoch % 30 == 0 or epoch == 1:
            print(f"Epoch {epoch:03d} | Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f}")
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(policy.state_dict(), "bc_anchor.pth")
            
    print("Optimization completed. Training 25D BC Anchor Coach succeeded.")
    print("Compiling Spiced Brain model for ONNX export...")
    
    policy.load_state_dict(torch.load("bc_anchor.pth"))
    policy.eval()
    inference_model = SpicedBrainInference(policy).to("cpu")
    inference_model.eval()
    
    dummy_input = torch.zeros(1, 25, dtype=torch.float32)
    onnx_path = os.path.join(model_dir, "spiced_brain.onnx")
    torch.onnx.export(
        inference_model,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=15,
        do_constant_folding=True,
        input_names=['input_state'],
        output_names=['action_output'],
        dynamic_axes={'input_state': {0: 'batch_size'}, 'action_output': {0: 'batch_size'}}
    )
    print(f"SUCCESS: 25D Spiced Brain exported to {onnx_path}")

if __name__ == "__main__":
    train_bc()
