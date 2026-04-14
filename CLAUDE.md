# CLAUDE.md — Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes, 100×100m field, BS at center [50,50]. A UAV jammer follows a circular orbit and degrades packet delivery. The proposed scheme integrates jamming risk (JR) into CH election and inter-cluster routing to outperform standard LEACH under jamming.

RNG seed is fixed at `rng(42)` in `main.m` — all schemes see the identical network topology and packet draws.

---

## Current State (as of 2026-04-14, Run 010)

### Implemented
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing
- `schemes/run_leach.m` — standard LEACH: probabilistic CH election, direct CH→BS
- `run_multiseed.m` — 5-seed averaging, Proposed + LEACH, 3-window PDR reporting
- `plotting/plot_multiseed.m` — mean ± std band plots
- `plotting/visualize_snapshot.m` — 2D network map with stranded node visualization

### Entry Points
- `main.m` — single seed (42), Proposed + LEACH, quick sanity checks
- `run_multiseed.m` — 5-seed average, final reported results

### Active Model: r_tx = 50m transmission range limit
A hard 50m transmission range limit is active. Member nodes farther than 50m from every CH are stranded — their packets count as lost in the PDR denominator. Applies to both Proposed and LEACH.

---

## Current Best Results (Run 010 — kappa=10, r_tx=50m, 5-seed average)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 597.2 ± 26.8 | 728.0 ± 34.1 |
| PDR all rounds (%) | **71.03 ± 1.23** | 62.74 ± 0.63 |
| PDR FND-trunc (%) | **82.49 ± 1.13** | 72.40 ± 2.27 |
| Zero-PDR rounds | **9.8 ± 7.3** | 136.6 ± 28.4 |
| Energy @ round 300 (J) | 31.64 ± 0.29 | 32.72 ± 1.23 |

**Headline:** Proposed delivers +8.3pp PDR (all rounds), +10.1pp PDR (FND-truncated), and 14× fewer zero-PDR rounds over LEACH under kappa=10 jamming.

**Key trade-off:** Proposed dies 131 rounds earlier (597 vs 728) due to relay CH energy concentration in multi-hop routing — but delivers more per round while alive. FND-truncated PDR gap (+10.1pp) is wider than all-rounds gap (+8.3pp), confirming the advantage holds within the operational window.


---

## Important Gotchas

**PDR reporting — three complementary windows:** Most WSN papers (2022–2025) do not specify their PDR evaluation window — this is a documented gap in the literature (see systematic review PMC12845974). We report PDR under three windows to address this:
- **All T rounds:** `mean(PDR)` — full lifecycle, most common in literature, use as primary for comparability
- **FND-truncated:** `mean(PDR(1:t_death))` — operational period only, isolates protocol performance from mortality timing
- **Zero-PDR round count:** `sum(PDR == 0)` — communication blackout duration under jamming, most direct adversarial reliability metric
End-of-life PDR=0 causes: (1) last node alive becomes sole CH with no members (`total_sent=0`), (2) jammer wipes all packets with 1–2 members remaining, (3) all non-CH nodes stranded beyond `r_tx`. These are not bugs — they are physically real. The three-window approach is the honest response. See README for paper phrasing and full source list.

**M=10 granularity:** With 10 packets per burst, PDR resolution per single node is 0.1. When the network shrinks to 1–2 nodes, per-round PDR snaps to {0, 0.1, 0.2, ...}. This is expected, not a bug.

**`gamma_` not `gamma`:** `gamma` is a MATLAB built-in. The CHScore weight is named `gamma_` everywhere — in `core/config.m`, `schemes/run_proposed.m`, `layer1/elect_ch_proposed.m`, and the `main.m` function call.

**`r_c` must be in `core/config.m`:** `r_c = 15` (communication range for neighbor counting) was missing from the original `config.m` and caused an undefined variable error. It has been added. If config.m is ever reset, this needs to be there.

**LEACH energy model:** The original `reference/LEACH.m` script used its own energy constants (epsilon_fs, epsilon_mp, d_0 threshold). `schemes/run_leach.m` was deliberately rewritten to use the same `E_elec`, `E_amp`, `E_da`, `L` from `core/config.m` as the proposed scheme — apples-to-apples comparison. Do not revert this.

**LEACH.m vs run_leach.m:** `reference/LEACH.m` is the original standalone script (kept for reference). `schemes/run_leach.m` is the integrated function used in the simulation. They are different files.

**`r_c` is NOT a radio range limit:** `r_c = 15m` is used exclusively in `elect_ch_proposed.m` to count neighbors for the CHScore beta term. Cluster assignment joins every member to its nearest CH within `r_tx` — no other range check exists. Do not confuse r_c with r_tx.

**`r_tx` is the transmission range limit:** `r_tx = 50m` enforces a hard maximum distance between a member node and its CH. Nodes beyond r_tx from every CH are stranded (CH_assign = 0). Their M packets are added to total_sent as lost — honest PDR accounting. This was added in Run 007. The guard `if CH_assign(i) == 0; continue; end` in the overhead and transmission loops handles stranded nodes silently.

**`r_exc` is derived, not arbitrary:** `r_exc = 25m` comes from the formula `sqrt(area² / (p_CH × N × π))`. At p_CH=0.05, K=5 CHs, area per CH = 2000m², r_exc = sqrt(2000/π) ≈ 25m. If p_CH is ever changed, r_exc must be updated consistently — or replaced with the dynamic formula `r_exc = sqrt(area^2 / (p_CH * N * pi))` in config.m.


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

- Decide on new baselines to rebuild from scratch (must include r_tx + honest 3-window PDR accounting from day one)
- Write paper discussion section using Run 010 numbers (kappa=10, 3-window PDR)
- Consider kappa sensitivity figure (kappa=3, 5, 10) to show robustness of proposed scheme
