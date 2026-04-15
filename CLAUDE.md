# CLAUDE.md - Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes in a `100 x 100 m` field, BS at `[50,50]`, and a UAV jammer moving on a circular orbit. The proposed scheme uses jamming risk (JR) in both CH election and inter-cluster routing, and is compared against standard LEACH.

---

## Current State (as of 2026-04-15, Run 013)

### Implemented
- `schemes/run_proposed.m` - proposed scheme: JR-aware CHScore election + Dijkstra routing + emergency CH re-election when no CH remains
- `schemes/run_leach.m` - standard LEACH: probabilistic CH election, direct CH-to-BS
- `run_multiseed.m` - current default evaluation using seeds `1:20`, Proposed + LEACH, 3-window PDR reporting
- `run_lambda_sensitivity.m` - lambda sensitivity sweep for the proposed scheme
- `plotting/plot_multiseed.m` - mean ± std band plots
- `plotting/visualize_snapshot.m` - 2D network map with stranded-node visualization
- `diag_proposed_zeros.m` / `diag_leach_zeros.m` - per-round blackout diagnostics

### Entry Points
- `main.m` - single seed quick check
- `run_multiseed.m` - current comparative evaluation entry point
- `run_lambda_sensitivity.m` - parameter sensitivity entry point for `lambda`

### Active Model
- `kappa = 10`
- `K_elec = 5`
- `lambda = 0.6`
- `p_CH = 0.05`
- `r_tx = 50m`
- Emergency CH re-election is enabled only when a round begins with no alive CHs

### Current Best Comparative Results (20 seeds, `1:20`)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 603.0 ± 35.1 | **726.6 ± 30.4** |
| PDR all rounds (%) | **71.24 ± 1.58** | 62.20 ± 0.85 |
| PDR FND-trunc (%) | **82.03 ± 1.28** | 72.81 ± 2.91 |
| Zero-PDR rounds | **1.2 ± 5.6** | 164.5 ± 15.7 |
| Energy @ round 300 (J) | 31.61 ± 0.43 | **32.39 ± 1.25** |

Headline:
- Proposed is clearly better on PDR and blackout avoidance.
- LEACH still wins on first-node-death and slightly on residual energy because relay burden is concentrated in the proposed multi-hop routing layer.

---

## Important Gotchas

**`lambda = 0.6` is still the best tested value.**
The sensitivity sweep over `{0.6, 0.7, 0.8}` showed no meaningful PDR gain from larger `lambda`, but worse blackout behavior due to noise amplification from `M = 10` packet bursts.

**`p_CH` is now actually controlled by `core/config.m`.**
`layer1/elect_ch_proposed.m` used to hardcode `p_CH = 0.05`. That has been fixed: `p_CH` now flows through the function signature from config. Any future `p_CH` sweep is now real.

**Emergency CH re-election only fixes `CHs = 0` rounds.**
It does not repair every late-stage failure mode. Remaining proposed zero-PDR rounds are end-of-life topology collapse cases: one CH left, many stranded nodes, and often no active members.

**`r_c` is not a radio limit.**
`r_c = 15m` is only a neighbor-counting radius for the CHScore connectivity term. The actual hard communication limit is `r_tx = 50m`.

**PDR must be read in three windows.**
Use all-round PDR, FND-truncated PDR, and zero-PDR round count together. End-of-life zeros are physically real and should not be silently dropped.

**`gamma_` not `gamma`.**
`gamma` conflicts with a MATLAB builtin. The CHScore jamming-risk weight is `gamma_` everywhere.

**LEACH comparison is intentionally apples-to-apples on energy constants.**
`schemes/run_leach.m` uses the same `E_elec`, `E_amp`, `E_da`, `L`, and `r_tx` model as the proposed scheme. Do not revert to `reference/LEACH.m` behavior for reported comparisons.

---

## Repo Structure

```text
main.m
run_multiseed.m
run_lambda_sensitivity.m
CLAUDE.md
core/
  config.m
  init_network.m
  uav_trajectory.m
  compute_packet_success.m
  compute_energy.m
layer1/
  update_jamming_risk.m
  elect_ch_proposed.m
layer2/
  route_dijkstra.m
schemes/
  run_proposed.m
  run_leach.m
plotting/
  plot_results.m
  plot_multiseed.m
  visualize_snapshot.m
docs/
  README.md
  SIMULATION_LOG.md
  proposed_model_for_paper_search.md
reference/
  LEACH.m
```

---

## What To Work On Next

Priority order:

1. Add residual-energy awareness to `route_dijkstra.m` so overloaded relay CHs are penalized.
2. Tune CHScore weights `alpha`, `beta`, `gamma_`, `delta` for better average PDR without a major lifetime collapse.
3. Sweep routing weights `phi1` and `phi3` to reduce unnecessary relays and average hop delay.
4. Consider adaptive CH density late in life instead of raising `p_CH` globally.
5. Keep zero-PDR rounds as a secondary issue for now; they are mostly after round 900 and are no longer the main bottleneck.

---

## How To Add a New Baseline

1. Create `schemes/run_<name>.m` returning a struct with `PDR`, `energy`, `delay`, `alive`, `t_death`, `label`.
2. Use the same `r_tx` stranded-node accounting as the current schemes.
3. Wire the new baseline into `main.m` and `run_multiseed.m`.
4. Log the new results in `docs/SIMULATION_LOG.md`.
