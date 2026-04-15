# CLAUDE.md - Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes in a `100 x 100 m` field, BS at `[50,50]`, and a UAV jammer moving on a circular orbit. The proposed scheme uses jamming risk (JR) in CH election and inter-cluster routing, compared against standard LEACH.

---

## Current State (as of 2026-04-15, Run 015)

### Implemented
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing + emergency CH re-election
- `schemes/run_proposed_direct.m` — proposed scheme variant: same election, direct CH-to-BS (no Dijkstra)
- `schemes/run_leach.m` — standard LEACH baseline
- `run_multiseed.m` — main evaluation: seeds 1:20, Proposed + LEACH, 3-window PDR
- `plotting/visualize_snapshot.m` — 2D network map with JR heatmap and routing paths
- `testing/` — all sensitivity sweeps, routing experiments, and diagnostics (run from project root)

### Entry Points
- `main.m` — single seed quick check
- `run_multiseed.m` — canonical comparative evaluation

### Active Model
- `kappa = 10`
- `K_elec = 5`
- `lambda = 0.6`
- `phi1 = 5e-4` (updated in Run 014)
- `p_CH = 0.05`
- `r_tx = 50m`
- Emergency CH re-election fires only when a round starts with no alive CHs

### Current Best Comparative Results (Run 014, 20 seeds)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 605.7 +/- 27.6 | **726.6 +/- 30.4** |
| PDR all rounds (%) | **71.59 +/- 1.64** | 62.20 +/- 0.85 |
| PDR FND-trunc (%) | **82.20 +/- 1.32** | 72.81 +/- 2.91 |
| Zero-PDR rounds | **0.0 +/- 0.0** | 164.5 +/- 15.7 |
| Energy @ round 300 (J) | 31.62 +/- 0.43 | **32.39 +/- 1.25** |

Proposed wins on PDR (+9.4pp) and eliminates blackouts entirely. LEACH outlasts by ~121 rounds due to relay burden in the proposed multi-hop layer.

---

## Important Gotchas

**The routing layer does not contribute PDR in this geometry (Run 015).**
`run_proposed_direct.m` (JR-aware election + direct CH-to-BS) matches or beats Dijkstra on every metric at center BS. ~78% of the 100x100m field is within r_tx=50m of BS=[50,50], so Dijkstra almost always confirms the direct path anyway. The entire ~9pp PDR advantage over LEACH comes from the election layer. Geometry tests at edge and corner BS confirmed routing does not improve at off-center BS either — the relay burden worsens zero-PDR rounds at every position tested.

**`lambda = 0.6` is the best tested value.**
Sweep over {0.6, 0.7, 0.8}: no PDR gain from higher lambda, worse blackouts from noise amplification with M=10 bursts.

**`p_CH` flows from `core/config.m` through function signatures.**
`elect_ch_proposed.m` no longer hardcodes p_CH. Any sweep is real.

**Emergency CH re-election only fixes `CHs=0` rounds.**
Remaining proposed zero-PDR rounds (now 0.0 +/- 0.0) are fully resolved. If they reappear, they are end-of-life topology collapses.

**`r_c` is not a radio limit.**
`r_c=15m` is only used in the CHScore beta term for neighbor counting. Hard radio limit is `r_tx=50m`.

**PDR must be read in three windows.**
All-rounds PDR, FND-truncated PDR, and zero-PDR count together. Never report just one.

**`gamma_` not `gamma`.**
`gamma` conflicts with a MATLAB builtin. The CHScore jamming-risk weight is `gamma_` everywhere.

**LEACH comparison is apples-to-apples.**
`run_leach.m` uses identical energy constants and r_tx accounting as the proposed scheme. Do not revert to `reference/LEACH.m`.

---

## Repo Structure

```text
main.m
run_multiseed.m
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
  run_proposed_direct.m
  run_leach.m
plotting/
  plot_results.m
  plot_multiseed.m
  visualize_snapshot.m
testing/
  run_lambda_sensitivity.m
  run_phi1_sweep.m
  run_routing_comparison.m
  run_geometry_test.m
  diag_proposed_zeros.m
  diag_leach_zeros.m
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

1. **Decide on routing** — Dijkstra is proven neutral/negative in this geometry (Run 015). Options: keep as framework, replace with direct, or redesign with threshold energy penalty.
2. **CHScore weight tuning** — narrow sweep around current values. Run 003 showed blunt alpha increase hurts; try small steps.
3. **Adaptive p_CH** — increase CH density only in late rounds when stranding occurs, not globally.
4. **Scaled-up geometry test** — 200x200m, 200 nodes, BS at corner — the scenario where Dijkstra routing would genuinely matter.
5. ~~phi1 sweep~~ — done in Run 014. phi1=5e-4 is canonical.
6. ~~Zero-PDR rounds~~ — solved (0.0 +/- 0.0).

---

## How To Add a New Baseline

1. Create `schemes/run_<name>.m` returning a struct with `PDR`, `energy`, `delay`, `alive`, `t_death`, `label`.
2. Use the same `r_tx` stranded-node accounting as the current schemes.
3. Wire into `main.m` and `run_multiseed.m`.
4. Log results in `docs/SIMULATION_LOG.md`.
