# CLAUDE.md - Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes in a `100 x 100 m` field, BS at `[50,50]`, and a UAV jammer moving on a circular orbit. The proposed scheme uses jamming risk (JR) in CH election and inter-cluster routing, compared against standard LEACH.

---

## Current State (as of 2026-04-17, Run 018)

### Implemented
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing + proactive emergency CH re-election + adaptive burst size (M_eff)
- `schemes/run_proposed_direct.m` — proposed scheme variant: same election, direct CH-to-BS (no Dijkstra)
- `schemes/run_leach.m` — standard LEACH baseline
- `schemes/run_tbc.m` — TBC baseline: flat multi-hop topology, instantaneous PDR detection, energy-aware Dijkstra, threshold suppression (Run 018)
- `run_multiseed.m` — main evaluation: seeds 1:20, Proposed + LEACH + TBC, 3-window PDR
- `plotting/visualize_snapshot.m` — 2D network map with JR heatmap and routing paths
- `testing/visualize_tbc_routing.m` — TBC routing snapshot: paths, relay load, jammed/isolated nodes
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
- Emergency CH re-election fires when a CH died last round OR no alive CHs remain (proactive)
- Adaptive burst size: `M_eff(i) = max(M_min, round(M*(1-JR(i))))`, `M_min=2` — jammed nodes conserve energy by reducing transmissions proportionally to their EWMA jamming risk (Run 017)

### Current Best Comparative Results (Run 018, 20 seeds)

| Metric | Proposed | LEACH | TBC |
|---|---|---|---|
| First node death (round) | **704.7 +/- 33.1** | 723.2 +/- 29.3 | 46.6 +/- 4.2 |
| PDR all rounds (%) | **85.11 +/- 2.02** | 62.46 +/- 0.82 | 5.20 +/- 0.35 |
| PDR FND-trunc (%) | **88.77 +/- 1.42** | 72.87 +/- 2.70 | 81.42 +/- 0.53 |
| Zero-PDR rounds | **0.0 +/- 0.0** | 158.4 +/- 13.7 | 932.5 +/- 3.4 |
| Energy @ round 300 (J) | **33.95 +/- 0.41** | 32.25 +/- 1.02 | 0.88 +/- 0.55 |

Proposed wins on every metric. TBC dies at ~round 47 from relay overload — flat topology is not viable under this energy model. TBC's FND-truncated PDR (81.4%) shows its detection mechanism works during its brief lifetime; the failure is structural.

---

## Important Gotchas

**The routing layer does not contribute PDR in this geometry (Run 015).**
`run_proposed_direct.m` (JR-aware election + direct CH-to-BS) matches or beats Dijkstra on every metric at center BS. ~78% of the 100x100m field is within r_tx=50m of BS=[50,50], so Dijkstra almost always confirms the direct path anyway. The entire PDR advantage over LEACH comes from the election layer. Geometry tests at edge and corner BS confirmed routing does not improve at off-center BS either.

**Scaled-up geometry (Run 016): Dijkstra is net-negative at 200x200m too.**
At 200x200m with r_tx=75m (44% field coverage), Dijkstra loses -6.56pp PDR vs Direct due to relay overload at scale. The election advantage grows to +17pp over LEACH at scale.

**`lambda = 0.6` is the best tested value.**
Sweep over {0.6, 0.7, 0.8}: no PDR gain from higher lambda, worse blackouts from noise amplification with M=10 bursts.

**Adaptive M_eff uses EWMA JR (not instantaneous p).**
`M_eff(i) = max(2, round(10*(1-JR(i))))`. With lambda=0.6, recovery after jammer leaves takes ~3 rounds — negligible vs 50-round orbit period. All three loop-replaying scripts (visualize_snapshot, diag_proposed_zeros, diag_scale) are kept in sync with run_proposed.m.

**`p_CH` flows from `core/config.m` through function signatures.**
`elect_ch_proposed.m` no longer hardcodes p_CH. Any sweep is real.

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
  run_tbc.m
plotting/
  plot_results.m
  plot_multiseed.m
  visualize_snapshot.m
testing/
  run_lambda_sensitivity.m
  run_phi1_sweep.m
  run_routing_comparison.m
  run_geometry_test.m
  run_scale_test.m
  diag_proposed_zeros.m
  diag_leach_zeros.m
  diag_scale.m
  visualize_tbc_routing.m
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

1. **Implement remaining baselines** — next candidates: EWMA-Detect (no action on JR), Threshold-JR (suppress jammed members), Reactive-CH (re-elect jammed CHs). Must use: r_tx stranded accounting, 3-window PDR, fixed M (fair comparison vs proposed's adaptive M_eff). See `proposed_model_for_paper_search.md` for literature targets.
2. **Wire remaining baselines into `run_multiseed.m`** and log in `SIMULATION_LOG.md`.
3. ~~TBC baseline~~ — done in Run 018. Flat multi-hop, instantaneous detection, energy-aware routing. Dies ~round 47 from relay overload.
4. ~~Adaptive burst size~~ — done in Run 017. M_eff = max(2, round(M*(1-JR))).
5. ~~Scaled-up geometry test~~ — done in Run 016. Dijkstra confirmed net-negative at 200x200m.
6. ~~phi1 sweep~~ — done in Run 014. phi1=5e-4 is canonical.
7. ~~Zero-PDR rounds~~ — solved (0.0 +/- 0.0).

---

## How To Add a New Baseline

1. Create `schemes/run_<name>.m` returning a struct with `PDR`, `energy`, `delay`, `alive`, `t_death`, `label`.
2. Use the same `r_tx` stranded-node accounting as the current schemes.
3. Use `compute_energy`, `compute_packet_success`, `update_jamming_risk` from `core/` and `layer1/`.
4. Wire into `main.m` and `run_multiseed.m`.
5. Log results in `docs/SIMULATION_LOG.md`.
