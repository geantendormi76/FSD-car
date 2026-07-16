# Phase 3 Domain and Scene Baseline

This directory closes Phase 1 gate `P1-U001/G0` and freezes the first FSD-car
operational domain as low-speed indoor warehouse and closed-track navigation.

## Frozen scene roles

- Primary: Isaac Sim 6.0 `warehouse_multiple_shelves.usd`, used as the immutable
  geometry and visual source for the project wrapper stage.
- Stress: Isaac Sim 6.0 `full_warehouse.usd`, used only after the primary scene
  passes performance and perception gates.
- Negative control: `assets/fsd_car_racetrack.usd`, retained to reproduce the
  Phase 2 synthetic-domain failure. It is forbidden as perception training data.

The primary vendor scene has realistic materials, collision geometry, lighting
and NavMesh volumes, but no semantic labels. Phase 4 must author labels in a
project-owned wrapper or overlay layer; vendor assets must not be edited in
place.

## Semantic contract

The transport shape remains `14x192x192` to preserve the frozen Phase 1 tensor
boundary. The meaning is versioned as `warehouse_nav14_v1`; only
`traversable_floor` and `floor_marking` encode free space. Cityscapes `road` is
not a valid alias for warehouse floor.

## Validation

Fast manifest, hash and evidence validation:

```bash
uv run contracts/phase3/validate_phase3.py
```

Deep USD inventory validation using Isaac Sim 6.0:

```bash
/home/zhz/isaacsim/python.sh contracts/phase3/inspect_usd_scenes.py
```

Integrity validation:

```bash
cd contracts/phase3 && sha256sum -c SHA256SUMS
```

Passing these commands completes Phase 3 baseline freezing. It does not open
the Phase 4 perception gate: semantic overlay authoring, real-camera distortion
calibration and a warehouse-adapted segmentation model remain mandatory.
