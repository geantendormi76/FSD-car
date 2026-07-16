# Phase 5-A Oracle NMPC White-box Loop

Phase 5-A isolates planning and control from learned perception. The warehouse
map comes only from the frozen Phase 4 USD semantic overlay.

The grid planner uses a conservative circumscribed-radius inflation. Closed-loop
collision acceptance uses the robot's yaw-aware rectangular footprint against
the raw oracle occupancy, so the final gate does not treat the robot as a point.

Build the oracle map with Isaac Sim:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/build_oracle_map.py
```

Run the deterministic 20 Hz white-box closed loop against the generated acados
C solver:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/oracle_nmpc_closed_loop.py
```

The gate covers a straight aisle, a turn with initial heading error, and a
pallet detour. Every run emits per-step CSV telemetry, a trajectory image and a
machine-readable summary. Acceptance requires all three goals, zero yaw-aware
footprint collisions, zero solver failures, bounded commands and p95 solver
latency no greater than one 20 Hz control period.

The candidate PIDNet and depth-lift BEV are forbidden from controlling the
Phase 5-A run. They remain shadow evidence until later phases.

Validate the frozen result with:

```bash
python3 contracts/phase5/validate_phase5a.py
cd contracts/phase5 && sha256sum -c SHA256SUMS
```

## Phase 5-B Shadow Comparison

Phase 5-B replays the three frozen Oracle trajectories through the warehouse
camera without granting any shadow output control authority. It compares the
Oracle occupancy against GT semantic depth-lift, deployed PIDNet flat IPM and
PIDNet depth-lift on exactly synchronized frames:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/phase5b_shadow_replay.py
python3 contracts/phase5/validate_phase5b.py
```

The current Cityscapes PIDNet is deliberately retained as a negative control.
Meeting the latency budget does not open the metric or control gate. Emergency
stop recall also remains unclaimed until a later suite supplies Oracle-positive
dynamic-obstacle frames.

## Phase 5-C Dynamic Upper Bound

Before spending GPU time on warehouse model adaptation, Phase 5-C checks
whether perfect warehouse semantic labels plus metric depth can satisfy the
same BEV gate. The counterfactual suite contains 400 center-corridor STOP
frames and 600 GO frames; none of its outputs can affect vehicle control:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/phase5c_dynamic_upper_bound.py
python3 contracts/phase5/validate_phase5c.py
```

The dynamic STOP/GO decision upper bound passes, but the global pixel/BEV
upper bound does not. Warehouse model training is therefore blocked until the
camera/depth-to-BEV geometry is repaired; training below a failed teacher upper
bound would not create a promotable candidate.

## Phase 5-C2 Depth-to-BEV Geometry Repair

Phase 5-C2 preserves the conservative Phase 5-A control map and creates a
separate perception-scoring Oracle from exact USD mesh faces clipped to the
robot collision-height band. The corrected depth lift rejects overhead shelf
surfaces, accepts only near-ground free evidence and rasterizes dynamic object
footprints by cell intersection rather than cell-center inclusion:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/build_perception_oracle_map.py
/home/zhz/isaacsim/python.sh contracts/phase5/phase5c2_geometry_upper_bound.py \
  --oracle-manifest artifacts/phase5c2_oracle/20260715_180806/manifest.json
python3 contracts/phase5/validate_phase5c2.py
```

The same 1000-frame dynamic suite now passes the frozen perception, latency and
STOP/GO gates. This opens warehouse semantic model training only. Oracle NMPC
retains sole control authority until a learned candidate passes a later shadow
gate.

## Phase 5-C3 warehouse_nav14 Model Adaptation

Phase 5-C3 trains an independent 14-class LRASPP-MobileNetV3 candidate. It does
not overwrite the deployed 19-class Cityscapes PIDNet contract. The dataset
contains 1200 training and 300 validation frames in disjoint 2 m world-space
blocks, excludes the frozen Phase 5 gate trajectory and covers all 14 classes:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/generate_phase5c3_dataset.py
/home/zhz/isaacsim/python.sh contracts/phase5/train_phase5c3_model.py \
  --dataset artifacts/phase5c3_dataset/20260715_191641
/home/zhz/isaacsim/python.sh contracts/phase5/phase5c3_candidate_shadow.py
python3 contracts/phase5/validate_phase5c3.py
```

The selected CUDA-trained candidate reaches validation mIoU 0.6631. On the
frozen 1000-frame shadow suite it reaches occupied IoU 0.5887, free IoU 0.9434,
400/400 STOP recall and 599/600 GO specificity with 14.39 ms p95 end-to-end
latency. The shadow gate passes, but this phase deliberately does not connect
the candidate to Dora or vehicle control. Oracle NMPC remains the sole control
authority pending a separate promotion phase.

## Phase 5-D Multi-seed Robustness and Runtime Gate

Phase 5-D applies frozen RGB, JPEG, depth and camera-mount perturbations to
three independent 1000-frame suites. Camera mount variation is paired with its
known depth-lift extrinsics, as required by the metric camera-to-body contract;
residual real-camera calibration error remains a separate deployment gate.
Each seed must pass independently:

```bash
/home/zhz/isaacsim/python.sh contracts/phase5/phase5d_multiseed_robustness.py
```

The dedicated Dora runtime topology replays 100 synchronized JPEG and metric
depth pairs at 20 Hz. It requires exact `source_frame_id` pairing, byte-exact
occupancy parity, 100% valid outputs and p95 latency below 50 ms:

```bash
dora up
PHASE5D_FIXTURE=artifacts/phase5d_robustness/20260715_211454/runtime_fixture \
PHASE5D_RUNTIME_OUTPUT=artifacts/phase5d_runtime/20260715_212658 \
  dora start contracts/phase5/dora_dataflow_phase5d_runtime.yaml --attach
python3 contracts/phase5/validate_phase5d.py
```

The runtime graph publishes only `shadow_bev_grid` and `shadow_health`; it has
no `control_cmd` output or NMPC consumer. Passing Phase 5-D therefore proves
robust shadow execution, not control promotion.

## Phase 5-E Real Calibration and Live Metric-depth Shadow

Phase 5-E requires two independent gates. The real-camera gate requires a
serial-bound 640x480 intrinsic calibration, target-based `T_body_camera`,
registered metric depth and frozen error evidence. It must not reuse the zero
distortion Isaac baseline:

The evidence file must conform to `phase5e_real_calibration.schema.json`.

```bash
python3 contracts/phase5/phase5e_real_calibration_audit.py \
  --calibration path/to/real_camera_calibration.json
```

The live Isaac gate renders RGB and `distance_to_image_plane` on demand, sends
both through Dora and scores the warehouse candidate against the USD Oracle on
the frozen Phase 5-A trajectory. It does not read the Phase 5-D fixture:

```bash
dora up
PHASE5E_MAX_FRAMES=1000 \
PHASE5E_LIVE_OUTPUT=artifacts/phase5e_live/20260715_215112 \
  dora start contracts/phase5/dora_dataflow_phase5e_live.yaml --attach
python3 contracts/phase5/validate_phase5e.py
```

The live gate passes 1000 frames. The current host has no `/dev/video*` device
or real calibration evidence, so the real-camera gate and Phase 5-E as a whole
remain blocked. No Phase 5-E output is connected to vehicle control.

## Phase 5-F Perception NMPC Shadow Loop

Phase 5-F gives the Oracle BEV and the warehouse candidate BEV identical frozen
vehicle states, A* paths, goals and acados NMPC configuration. Only the three
local obstacle ellipses differ. The candidate command is written to telemetry
and has no Dora output edge, so Phase 5-A Oracle NMPC remains the sole authority:

```bash
dora up
PHASE5F_MAX_FRAMES=1000 \
PHASE5F_OUTPUT=/home/zhz/fsd-car/artifacts/phase5f_shadow/20260715_234420 \
  dora start contracts/phase5/dora_dataflow_phase5f_shadow.yaml --attach
python3 contracts/phase5/validate_phase5f.py
```

The initial v1 gate was retained as rejected evidence. It incorrectly counted
moderate Oracle deceleration as release and treated near-zero or symmetric
bypass steering sign as unique ground truth. The later V2 adapter shifted an
observed obstacle surface by the ellipse half-axis. Phase 5-G continuous motion
showed that this removed the point-robot Minkowski safety inflation and could
permit a swept footprint collision, so V2 is retained as invalidated evidence.

V3 keeps the observed occupied point as the local obstacle point and treats the
ellipse axes as robot-footprint safety inflation. All 1000 frames still gate
pairing, validity, solver success and latency. Action equivalence is restricted
to 400 physically eligible far/absent frames: `center_stop` teleports an object
into footprint contact, while side modes allow more than one safe bypass side.
V3 passes with exact pairing, 100% runtime validity, 100% solver success,
23.68 ms candidate pipeline p95 latency, 0.0022 m/s^2 acceleration MAE,
0.0038 rad/s omega MAE and 100% eligible steering-direction agreement.

Passing Phase 5-F proves counterfactual command agreement on frozen simulation
states. It does not prove candidate-driven closed-loop stability and does not
open real-vehicle control promotion.

## Phase 5-G Controlled Simulation Takeover

Phase 5-G makes the warehouse candidate perception and candidate acados NMPC
the actual command owner of a deterministic 20 Hz differential-drive
simulation. The Phase 5-A USD Oracle supplies the global A* reference and an
independent swept-collision supervisor. The supervisor may only allow a command
or abort before it is applied; it never steers or substitutes an Oracle command:

```bash
/home/zhz/isaacsim/python.sh \
  contracts/phase5/phase5g_controlled_takeover.py
python3 contracts/phase5/validate_phase5g.py
```

The frozen run covers straight aisle, diagonal turn, pallet detour and a
world-time crossing cart. All 4 scenarios reached their goals with 1473/1473
candidate commands applied, zero Oracle aborts, zero swept-footprint collisions
and zero solver failures. Candidate pipeline p95 latency is at most 32.45 ms;
path-error p95 is at most 0.0435 m. The crossing cart is observed for 205 frames
and maintains a minimum 0.598 m center distance.

This is controlled takeover of numeric vehicle kinematics while Isaac supplies
live warehouse RGB and metric depth. It is not yet Isaac articulation/wheel
physics takeover. Synchronous Isaac rendering currently makes full
sensor-to-command p95 as high as 229.11 ms, so real-time 20 Hz wall-clock
operation and all real-vehicle authority remain closed for the next gate.

## Phase 5-H Isaac Articulation Takeover

Phase 5-H references only `/Root/jetbot` from the frozen local vehicle USD into
the warehouse semantic stage. The candidate NMPC acceleration and yaw-rate are
converted to left/right wheel velocity targets. Vehicle position, heading and
speed are read back from the PhysX articulation; direct pose integration is
forbidden after scenario initialization:

```bash
/home/zhz/isaacsim/python.sh \
  contracts/phase5/phase5h_articulation_takeover.py
python3 contracts/phase5/validate_phase5h.py
```

The complete latency interval begins before the rendered physics step and ends
after both wheel targets are written. Rendering uses 320x240 with proportionally
scaled intrinsics, preserving camera field of view. The in-process JPEG round
trip and offline `rep.orchestrator.step()` are removed. Physics runs at 100 Hz;
perception and NMPC run at 20 Hz.

The first formal attempt is retained as rejected clock evidence. A rendered
step configured with `rendering_dt=0.05` already advanced five 10 ms physics
substeps; four extra substeps made the robot travel at about 1.8 times the
declared simulation clock and collide with the crossing cart. The repair sets
`rendering_dt=physics_dt=0.01` and executes exactly one rendered plus four
non-rendered steps. A root-path versus wheel-odometry ratio gate now detects
this class of clock error independently.

The corrected frozen run reaches all four goals with 1454/1454 candidate wheel
commands applied, zero Oracle aborts, zero collisions and zero solver failures.
Maximum full RGB+metric-depth-to-wheel p95 latency is 37.97 ms. Root-path to
wheel-odometry ratios are 0.9792 to 0.9812. The crossing cart is encountered for
206 frames with a 0.598 m minimum center distance. This opens Isaac articulation
control only; real-vehicle authority remains closed.

## Phase 5-I Formal Dora Control and Safety Gate

Phase 5-I moves the Phase 5-H articulation loop into the formal Dora topology.
The articulation runtime may propose an NMPC command, but only the independent
safety supervisor can publish the command consumed by the wheel actuator. The
supervisor requires five consecutive healthy startup frames, fails closed after
150 ms of stale input, latches emergency stop until an explicit reset and then
requires a fresh warmup. The actuator has a second local 150 ms watchdog.

The camera contract keeps a native 640x480 source. A 2 Hz JPEG branch preserves
native 640x480 frames for XFeat localization, while the warehouse semantic and
metric-depth control branch downsamples to its native 320x240 input with scaled
intrinsics at 20 Hz. Upscaling a 320x240 image is explicitly forbidden because
it cannot recreate localization detail. XFeat inference remains outside the
50 ms control-critical path.

Three independent 180-frame runs exercise nominal control, latched emergency
stop plus reset, and watchdog stop plus reset:

```bash
powerprofilesctl set performance
dora up

PHASE5I_RUN_MODE=nominal PHASE5I_MAX_FRAMES=180 \
PHASE5I_OUTPUT=artifacts/phase5i_dora/final_nominal_20260716 \
  dora start contracts/phase5/dora_dataflow_phase5i_control.yaml --attach

PHASE5I_RUN_MODE=emergency_stop_reset PHASE5I_MAX_FRAMES=180 \
PHASE5I_OUTPUT=artifacts/phase5i_dora/final_emergency_stop_reset_20260716 \
  dora start contracts/phase5/dora_dataflow_phase5i_control.yaml --attach

PHASE5I_RUN_MODE=watchdog_reset PHASE5I_MAX_FRAMES=180 \
PHASE5I_OUTPUT=artifacts/phase5i_dora/final_watchdog_reset_20260716 \
  dora start contracts/phase5/dora_dataflow_phase5i_control.yaml --attach

python3 contracts/phase5/validate_phase5i.py
powerprofilesctl set balanced
```

All 540 frames pass exact sequencing, startup, reset, collision and timing
gates. The worst run has 31.60 ms sensor-to-wheel p95 latency and 10.14 ms p95
jitter, with zero static or dynamic collisions. Each run also decodes the exact
18-frame native 640x480 localization sequence expected at 2 Hz. This opens the
formal Dora Isaac-simulation control topology only. Real-vehicle authority
remains closed.

## Phase 5-J Endurance and Fault Injection

Phase 5-J separates the Isaac articulation plant and its local actuator
watchdog from the learned perception/NMPC controller. This split is required to
observe physical wheel shutdown after the controller or Dora safety-supervisor
process is killed. The safety supervisor remains the only Dora node that can
publish `safe_control`; the controller and fault injector have no wheel output.

The formal gate contains five independent runs. The endurance run executes
4400 frames (220 simulated seconds) across straight aisle, diagonal turn,
pallet detour and crossing cart, then repeats all four scenarios. Four 300-frame
fault runs inject controller SIGKILL, safety-supervisor SIGKILL, eight replayed
sensor frames plus reset, and a fresh controller process generation:

```bash
powerprofilesctl set performance
dora coordinator
# In a second terminal:
dora daemon
# In a third terminal, set the mode, generation, output and frame count:
PHASE5J_RUN_MODE=restart_recovery PHASE5J_GENERATION=5 \
PHASE5J_MAX_FRAMES=300 \
PHASE5J_OUTPUT=artifacts/phase5j_resilience/final_restart_recovery_20260716 \
  dora start contracts/phase5/dora_dataflow_phase5j_resilience.yaml --attach

python3 contracts/phase5/validate_phase5j.py
powerprofilesctl set balanced
```

SIGKILL runs intentionally make `dora start` return nonzero because Dora
correctly reports the killed node. Their independent evidence sink must still
run to completion and pass. The first 150 ms watchdog contract is retained as
rejected evidence because asynchronous 20 Hz telemetry observed zero on the
fourth frame. V2 uses dual 100 ms watchdogs and observes zero by the third
frame. A separate rejected endurance run exposed that theoretical `-0.6`
rad/s becomes slightly less than `-0.6` after float32 serialization. The
controller now clips one float32 ULP inside the frozen bound; the supervisor
limit is not relaxed.

The final aggregate covers 5600 frames with zero static/dynamic collisions,
zero wrong-generation commands and a maximum three-frame fault-stop latency.
The endurance run has 4387 active frames and 37.42 ms sensor-to-wheel p95
latency. Controller and supervisor SIGKILL, sensor replay rejection, explicit
reset and fresh-process restart all pass. Real-vehicle authority remains
closed.

## Phase 5-K Infrastructure and Resource Resilience

Phase 5-K keeps the Phase 5-J controller, safety supervisor and Isaac plant
boundaries, then tests failures below the Dora node layer. Coordinator loss
must stop the actual articulation through the surviving local data plane.
Daemon loss is handled by an independent host watchdog that signals the Isaac
plant and records the zero-wheel command in a preallocated mmap ledger outside
Dora routing. GPU and disk tests use a real CUDA allocation failure and Linux
`/dev/full` ENOSPC instead of mocked exceptions.

The hour run must execute 72,000 exact frames at 20 Hz for at least 3,600 wall
seconds. Run coordinator and daemon in sessions independent of the invoking
terminal so an SSH or Codex session interruption does not terminate the gate:

```bash
powerprofilesctl set performance
setsid -f dora coordinator --quiet
setsid -f dora daemon --quiet

PHASE5K_RUN_MODE=hour_endurance PHASE5K_GENERATION=41 \
PHASE5K_MAX_FRAMES=72000 \
PHASE5K_OUTPUT=artifacts/phase5k/hour_endurance_final_v4_20260716_144513 \
  dora start --detach contracts/phase5/dora_dataflow_phase5k_resilience.yaml

python3 contracts/phase5/validate_phase5k.py
powerprofilesctl set balanced
```

All five gates pass. The 72,000-frame run lasted 3,611.33 seconds across 131
episodes, with 99.579% active frames, zero collisions, zero wrong-generation
commands and 38.12 ms sensor-to-wheel p95 latency. Coordinator SIGKILL stopped
the articulation in 41.88 ms; daemon SIGKILL was stopped by the host watchdog
in 7.43 ms. The CUDA probe allocated 9.40 GB before a real OOM and recovered
97.1% of pre-fault free memory. `/dev/full` returned the required errno 28 and
the control path recovered one frame after the fault.

This gate proves simulation infrastructure resilience on the frozen host. It
does not authorize real-vehicle control or claim that a single 149.10 ms
latency outlier is suitable for a hard real-time actuator deadline.
