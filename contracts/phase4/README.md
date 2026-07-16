# Phase 4 Semantic Overlay and Perception Data Loop

Phase 4 keeps the Isaac Sim 6.0 vendor warehouse immutable. A project-owned
stronger USD layer labels every composed mesh with `warehouse_nav14_v1`.

## P4-A: Build and validate the overlay

```bash
/home/zhz/isaacsim/python.sh contracts/phase4/build_semantic_overlay.py
uv run contracts/phase4/validate_phase4.py
```

The builder refuses to run if the vendor USD hash differs from Phase 3. The
generated layer is `assets/phase4/warehouse_nav14_overlay.usda`.

## P4-B: Capture synchronized smoke evidence

```bash
/home/zhz/isaacsim/python.sh contracts/phase4/capture_smoke_dataset.py --frames 8
uv run contracts/phase4/validate_phase4.py --capture artifacts/phase4_capture/<timestamp>
```

Each frame stores raw RGB8, deployed JPEG, semantic IDs, metric depth, semantic
GT-IPM, depth-lift semantic BEV and a geometry oracle in one compressed NPZ archive. `summary.json`
binds every sample to the overlay and records file hashes and camera geometry.

## P4-C: Candidate perception gate

The smoke capture proves the data path, not model quality. The gate remains
closed until a warehouse-adapted model is evaluated on at least 1000 exact
synchronized frames against the Phase 3 thresholds. Real deployment also
requires measured lens distortion; simulation coefficients are intentionally
kept at zero and may not be presented as real calibration.

The frozen smoke evidence is
`artifacts/phase4_capture/20260715_155636`. `phase4_status.json` records its
hashes and the measured difference between flat IPM, depth-lift BEV and the USD
geometry oracle. P4-A and P4-B are complete; P4-C remains explicitly closed.
