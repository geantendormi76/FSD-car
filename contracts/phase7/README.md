# Phase 7 Hardware Sizing And Deployment Profile

Phase 7 is the final planned stage. It profiles the deployment stack without
Isaac Sim: warehouse_nav14 semantic inference, metric depth-to-BEV, acados
NMPC and the 2 Hz XFeat localization slow path. PIDNet and the abandoned
Spiced BC/PPO model are intentionally excluded.

Reproduce the frozen host profile with Isaac Sim Python, which already carries
the project's CUDA ONNX Runtime and acados dependencies:

```bash
powerprofilesctl set performance
/home/zhz/isaacsim/python.sh contracts/phase7/run_phase7_profile.py \
  --output artifacts/phase7_profile/final_20260716_174458
python3 contracts/phase7/validate_phase7.py
cd contracts/phase7 && sha256sum -c SHA256SUMS
powerprofilesctl set balanced
```

The RTX 3060 reference run passed. The 20 Hz control pipeline measured 17.07
ms p95, acados NMPC measured 1.20 ms p95, and the native 640x480 to 640x640
XFeat path measured 3.85 ms p95. Process RSS increased by 1,211.97 MiB and
desktop GPU memory increased by 253 MiB while both CUDA and CPU fallback ONNX
sessions were resident.

The selected deployment target is Jetson Orin NX 16GB Super. Orin Nano Super
8GB remains a prototype-only minimum; AGX Orin 32GB is the high-headroom
option. Desktop latency and power are not Jetson predictions. The exact same
profile must pass on purchased target hardware before physical actuation or
real-vehicle control is considered.

See `deployment_profile_zh-CN.md` for the measured evidence, sizing rationale,
recommended supporting hardware and remaining physical integration gates.
