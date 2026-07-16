# Phase 1 Data Contract v1

This directory freezes the transport and coordinate contracts used before the
perception-policy-control refactor. `data_contracts.json` is normative;
implementation comments and historical CSV headers are not contracts.

## Validation

```bash
uv run contracts/v1/validate_contracts.py
```

The validator checks that every input edge in the four current DORA topologies
has exactly one documented contract and that every stream declares dtype,
shape, fields, units, frame, frequency, freshness, and failure behavior.

## Frozen geometry

- World frame `W`: ground-plane `x/y`, `z` up, yaw zero faces `+x`.
- Ego frame `E`: `x` forward, `y` left, `z` up.
- Camera frame `C`: OpenCV convention, `x` right, `y` down, `z` forward.
- BEV: `192x192`, `20m x 20m`, origin `(95.5,95.5)`; decreasing row is
  forward and decreasing column is left.
- Occupancy: `0=free`, `255=occupied-or-unknown` for perception output.

## Policy ownership

The frozen policy boundary consumes synchronized ego state, local goal and
14-channel semantic BEV blocks. It emits normalized desired speed and
curvature references. It does not own wheel commands: NMPC and TTC retain final
control and safety authority.

## Deliberate compatibility profiles

`human_prior` and `control_cmd` currently carry different semantics or limits
across topologies. The JSON therefore binds profile-specific contracts to each
edge instead of pretending the payloads are globally identical. These profiles
preserve current interfaces while making the conflict visible for Phase 2.

## Open gate

`P1-U001/G0` remains unresolved: the first real deployment domain must be
declared as indoor warehouse/track or outdoor road before semantic taxonomy and
PIDNet adaptation data can be frozen.
