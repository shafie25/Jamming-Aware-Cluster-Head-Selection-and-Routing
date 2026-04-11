# CLAUDE.md — Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes, 100×100m field, BS at center [50,50]. A UAV jammer follows a circular orbit and degrades packet delivery. The proposed scheme integrates jamming risk (JR) into CH election and inter-cluster routing to outperform standard LEACH under jamming.

RNG seed is fixed at `rng(42)` in `main.m` — all schemes see the identical network topology and packet draws.

---

## Current State (as of 2026-04-11)

### Implemented — All Schemes Complete
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing
- `schemes/run_leach.m` — standard LEACH: probabilistic CH election, direct CH→BS
- `schemes/run_ewma_detect.m` — Baseline 1: LEACH + EWMA tracking (detection unused)
- `schemes/run_threshold.m` — Baseline 2: LEACH + member suppression when JR > 0.70
- `schemes/run_reactive_ch.m` — Baseline 3: LEACH + reactive CH re-election when CH JR > 0.50
- `run_multiseed.m` — 5-seed averaging entry point (all 5 schemes)
- `plotting/plot_multiseed.m` — mean ± std band plots (5 schemes, colorblind-friendly)

### Entry Points
- `main.m` — single seed (42), all 5 schemes, quick sanity checks
- `run_multiseed.m` — 5-seed average, final reported results

---

## Final Results (Run 005 — 5-scheme, 5-seed average — USE THESE IN THE PAPER)

| Metric | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| First node death (round) | 394.0 ± 86.6 | 432.0 ± 34.0 | 413.8 ± 23.6 | 414.4 ± 22.4 | 401.8 ± 25.4 |
| PDR mean (%) | **81.63 ± 3.46** | 58.99 ± 1.02 | 58.96 ± 1.87 | 60.25 ± 1.90 | 58.70 ± 1.56 |
| Energy @ round 300 (J) | **30.42 ± 0.73** | 26.92 ± 1.03 | 26.06 ± 1.07 | 27.50 ± 0.99 | 26.29 ± 0.43 |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 | 330.2 ± 19.3 | 308.6 ± 20.3 | 330.6 ± 17.4 |

**Headline result:** Proposed scheme delivers +21.4pp PDR over the best baseline (Threshold-JR), with 4× fewer zero-PDR rounds and better energy efficiency across all 5 seeds.

**EWMA-Detect ≈ LEACH** — confirms detection without adaptation provides no benefit.

**Reactive-CH slightly worse than LEACH** — reactive re-clustering inside a jammed cluster zone is ineffective; the whole cluster tends to be jammed, so replacing the CH with a nearby member doesn't escape the jammer.

---

## Important Gotchas

**PDR reporting:** Always report mean separately for all-rounds vs active-rounds-only. The `min(PDR(PDR>0))` filter hides zero-PDR rounds — be explicit about this when comparing metrics.

**M=10 granularity:** With 10 packets per burst, PDR resolution per single node is 0.1. When the network shrinks to 1–2 nodes, per-round PDR snaps to {0, 0.1, 0.2, ...}. This is expected, not a bug.

**`gamma_` not `gamma`:** `gamma` is a MATLAB built-in. The CHScore weight is named `gamma_` everywhere — in `core/config.m`, `schemes/run_proposed.m`, `layer1/elect_ch_proposed.m`, and the `main.m` function call.

**`r_c` must be in `core/config.m`:** `r_c = 15` (communication range for neighbor counting) was missing from the original `config.m` and caused an undefined variable error. It has been added. If config.m is ever reset, this needs to be there.

**LEACH energy model:** The original `reference/LEACH.m` script used its own energy constants (epsilon_fs, epsilon_mp, d_0 threshold). `schemes/run_leach.m` was deliberately rewritten to use the same `E_elec`, `E_amp`, `E_da`, `L` from `core/config.m` as the proposed scheme — apples-to-apples comparison. Do not revert this.

**LEACH.m vs run_leach.m:** `reference/LEACH.m` is the original standalone script (kept for reference). `schemes/run_leach.m` is the integrated function used in the simulation. They are different files.

**`r_c` is NOT a radio range limit:** `r_c = 15m` is used exclusively in `elect_ch_proposed.m` to count neighbors for the CHScore beta term. Cluster assignment joins every member to its nearest CH with no maximum range check — this is a standard LEACH-simulation simplification. The d² energy model penalizes long links; it does not block them. Do not add a range check to cluster assignment unless explicitly asked. Document this assumption in the paper.

---

## How to Add a New Baseline

1. Create `schemes/run_<name>.m` as a function returning a struct with fields: `PDR`, `energy`, `delay`, `alive`, `t_death`, `label` — all vectors of length T except `t_death` (scalar) and `label` (string)
2. Call it in `main.m` after the existing scheme calls, passing network state from the workspace
3. Append its result to `results_all`
4. Log the run in `docs/SIMULATION_LOG.md`

---

## Repo Structure

```
main.m                       ← entry point (addpath(genpath('.')) at top)
CLAUDE.md                    ← this file
core/
  config.m                   ← all parameters — edit here, nowhere else
  init_network.m             ← node deployment, initial state vectors
  uav_trajectory.m           ← precomputes J_x, J_y for all T rounds
  compute_packet_success.m   ← p_i(t) per node given jammer position
  compute_energy.m           ← energy cost for tx / rx / agg / overhead
layer1/
  update_jamming_risk.m      ← EWMA PDR → JR update
  elect_ch_proposed.m        ← CHScore greedy election
layer2/
  route_dijkstra.m           ← Dijkstra inter-cluster routing
schemes/
  run_proposed.m             ← proposed scheme round loop
  run_leach.m                ← standard LEACH baseline round loop
  run_ewma_detect.m          ← Baseline 1: LEACH + EWMA detection (unused)
  run_threshold.m            ← Baseline 2: LEACH + member suppression (JR > 0.70)
  run_reactive_ch.m          ← Baseline 3: LEACH + reactive CH re-election (JR > 0.50)
plotting/
  plot_results.m             ← 4-panel results figure
reference/
  LEACH.m                    ← original LEACH script (reference only, not called)
docs/
  README.md                  ← project overview
  SIMULATION_LOG.md          ← per-run results log — update after every run
```

---

## What to Work on Next

Simulation is complete. All 5 schemes implemented, all results logged (Run 005).
Next step is writing the paper discussion section using the takeaways in SIMULATION_LOG.md Run 005.
