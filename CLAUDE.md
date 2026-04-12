# CLAUDE.md — Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes, 100×100m field, BS at center [50,50]. A UAV jammer follows a circular orbit and degrades packet delivery. The proposed scheme integrates jamming risk (JR) into CH election and inter-cluster routing to outperform standard LEACH under jamming.

RNG seed is fixed at `rng(42)` in `main.m` — all schemes see the identical network topology and packet draws.

---

## Current State (as of 2026-04-12)

### Implemented — All Schemes Complete
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing
- `schemes/run_leach.m` — standard LEACH: probabilistic CH election, direct CH→BS
- `schemes/run_ewma_detect.m` — Baseline 1: LEACH + EWMA tracking (detection unused)
- `schemes/run_threshold.m` — Baseline 2: LEACH + member suppression when JR > 0.70
- `schemes/run_reactive_ch.m` — Baseline 3: LEACH + reactive CH re-election when CH JR > 0.50
- `run_multiseed.m` — 5-seed averaging entry point (currently Proposed + LEACH only — see note)
- `plotting/plot_multiseed.m` — mean ± std band plots
- `plotting/visualize_snapshot.m` — 2D network map with stranded node visualization

### Entry Points
- `main.m` — single seed (42), Proposed + LEACH, quick sanity checks
- `run_multiseed.m` — 5-seed average, final reported results

### Active Model: r_tx = 50m transmission range limit
As of Run 007, a hard 50m transmission range limit is active. Member nodes farther than 50m from every CH are stranded — their packets count as lost in the PDR denominator. This applies to both Proposed and LEACH. The 3 baselines have not yet been updated with r_tx.

---

## Current Best Results (Run 007 — Proposed vs LEACH, r_tx=50m, 5-seed average)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 581.0 ± 23.0 | 718.2 ± 28.7 |
| PDR mean (%) | **73.81 ± 1.72** | 64.54 ± 0.46 |
| Energy @ round 300 (J) | 31.64 ± 0.24 | 32.34 ± 1.08 |
| Zero-PDR rounds | **13.6 ± 2.8** | 155.0 ± 23.9 |

**Headline:** Proposed delivers +9.3pp PDR over LEACH and 11× fewer zero-PDR rounds under r_tx=50m.

## Pre-r_tx Results (Run 005 — All 5 Schemes, no range limit — reference only)

| Metric | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| First node death (round) | 394.0 ± 86.6 | 432.0 ± 34.0 | 413.8 ± 23.6 | 414.4 ± 22.4 | 401.8 ± 25.4 |
| PDR mean (%) | **81.63 ± 3.46** | 58.99 ± 1.02 | 58.96 ± 1.87 | 60.25 ± 1.90 | 58.70 ± 1.56 |
| Energy @ round 300 (J) | **30.42 ± 0.73** | 26.92 ± 1.03 | 26.06 ± 1.07 | 27.50 ± 0.99 | 26.29 ± 0.43 |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 | 330.2 ± 19.3 | 308.6 ± 20.3 | 330.6 ± 17.4 |

---

## Important Gotchas

**PDR reporting:** Always report mean separately for all-rounds vs active-rounds-only. The `min(PDR(PDR>0))` filter hides zero-PDR rounds — be explicit about this when comparing metrics.

**M=10 granularity:** With 10 packets per burst, PDR resolution per single node is 0.1. When the network shrinks to 1–2 nodes, per-round PDR snaps to {0, 0.1, 0.2, ...}. This is expected, not a bug.

**`gamma_` not `gamma`:** `gamma` is a MATLAB built-in. The CHScore weight is named `gamma_` everywhere — in `core/config.m`, `schemes/run_proposed.m`, `layer1/elect_ch_proposed.m`, and the `main.m` function call.

**`r_c` must be in `core/config.m`:** `r_c = 15` (communication range for neighbor counting) was missing from the original `config.m` and caused an undefined variable error. It has been added. If config.m is ever reset, this needs to be there.

**LEACH energy model:** The original `reference/LEACH.m` script used its own energy constants (epsilon_fs, epsilon_mp, d_0 threshold). `schemes/run_leach.m` was deliberately rewritten to use the same `E_elec`, `E_amp`, `E_da`, `L` from `core/config.m` as the proposed scheme — apples-to-apples comparison. Do not revert this.

**LEACH.m vs run_leach.m:** `reference/LEACH.m` is the original standalone script (kept for reference). `schemes/run_leach.m` is the integrated function used in the simulation. They are different files.

**`r_c` is NOT a radio range limit:** `r_c = 15m` is used exclusively in `elect_ch_proposed.m` to count neighbors for the CHScore beta term. Cluster assignment joins every member to its nearest CH within `r_tx` — no other range check exists. Do not confuse r_c with r_tx.

**`r_tx` is the transmission range limit:** `r_tx = 50m` enforces a hard maximum distance between a member node and its CH. Nodes beyond r_tx from every CH are stranded (CH_assign = 0). Their M packets are added to total_sent as lost — honest PDR accounting. This was added in Run 007. The guard `if CH_assign(i) == 0; continue; end` in the overhead and transmission loops handles stranded nodes silently.

**`r_exc` is derived, not arbitrary:** `r_exc = 25m` comes from the formula `sqrt(area² / (p_CH × N × π))`. At p_CH=0.05, K=5 CHs, area per CH = 2000m², r_exc = sqrt(2000/π) ≈ 25m. If p_CH is ever changed, r_exc must be updated consistently — or replaced with the dynamic formula `r_exc = sqrt(area^2 / (p_CH * N * pi))` in config.m.

**Baselines not updated with r_tx:** `run_ewma_detect.m`, `run_threshold.m`, `run_reactive_ch.m` do not have the r_tx range limit or stranded node accounting. Do not run them in multi-seed comparisons with the proposed scheme and LEACH until they are updated.

**run_multiseed.m is currently 2-scheme only:** After Run 007, `run_multiseed.m` was trimmed to Proposed + LEACH. To restore all 5 schemes, the baselines need r_tx updates first.

---

## How to Add a New Baseline

1. Create `schemes/run_<name>.m` as a function returning a struct with fields: `PDR`, `energy`, `delay`, `alive`, `t_death`, `label` — all vectors of length T except `t_death` (scalar) and `label` (string)
2. Add `r_tx` parameter to the function signature
3. Enforce `min_d <= r_tx` in cluster assignment; add `n_stranded * M` to `total_sent`
4. Call it in `main.m` after the existing scheme calls, passing network state from the workspace
5. Append its result to `results_all`
6. Log the run in `docs/SIMULATION_LOG.md`

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
  plot_multiseed.m           ← mean ± std band plots (multi-seed)
  visualize_snapshot.m       ← 2D network map at a specific round
reference/
  LEACH.m                    ← original LEACH script (reference only, not called)
docs/
  README.md                  ← project overview and design decisions
  SIMULATION_LOG.md          ← per-run results log — update after every run
```

---

## What to Work on Next

- Dynamic p_CH: scale CH density up as N_alive drops (0.05 at full network → higher in late rounds) to reduce stranded nodes without paying overhead cost in healthy rounds
- Update baselines (run_ewma_detect, run_threshold, run_reactive_ch) with r_tx so full 5-scheme comparison can be run under the range limit
- Write paper discussion section — use Run 007 numbers if r_tx is kept as the model, Run 005 numbers if reverting to no range limit
