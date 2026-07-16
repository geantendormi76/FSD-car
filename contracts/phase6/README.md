# Phase 6 Final Simulation Acceptance Matrix

Phase 6 is a single final simulation gate. It does not add another controller
or retrain the warehouse model. It composes the frozen Phase 5-H Isaac
articulation loop with deterministic domain perturbations and binds the result
to the frozen Phase 5-K hour-endurance and fault-recovery evidence.

The matrix contains exactly three seeds and four scenarios, for 12 independent
closed-loop cases. Every seed has a different lighting intensity, dynamic-cart
material color, JPEG/RGB distribution, metric-depth noise distribution, camera
extrinsic residual, start/goal offset and dynamic-obstacle offset. Candidate
warehouse perception and acados NMPC own the wheel command. The USD Oracle is
limited to the global A* reference and an allow-or-abort collision supervisor.

Run the matrix with Isaac Sim Python:

```bash
powerprofilesctl set performance
/home/zhz/isaacsim/python.sh contracts/phase6/run_phase6_matrix.py \
  --output artifacts/phase6_matrix/final_20260716_1642
python3 contracts/phase6/validate_phase6.py
cd contracts/phase6 && sha256sum -c SHA256SUMS
powerprofilesctl set balanced
```

The frozen run passed all 12 cases. Every goal was reached with zero static or
dynamic collisions, zero Oracle supervisor aborts and zero NMPC solver
failures. The worst sensor-to-wheel p95 was 33.06 ms against a 50 ms limit;
worst path-error p95 was 0.0914 m and worst angular-command delta p95 was
0.2332 rad/s. All three crossing-cart cases encountered the dynamic obstacle;
their minimum center distances were 0.556 m to 0.642 m.

The same gate binds the Phase 5-K 72,000-frame, 3,611-second endurance run and
its one-frame maximum recoverable-fault stop latency. This avoids rerunning an
identical hour test while preserving its evidence hash.

Passing Phase 6 creates a simulation release candidate only. The global route
and independent safety supervisor still use the USD Oracle, and no real camera,
real localization or physical actuator has passed this matrix. Real-vehicle
control remains forbidden. The next and only planned stage is Phase 7 hardware
sizing and deployment profiling.
