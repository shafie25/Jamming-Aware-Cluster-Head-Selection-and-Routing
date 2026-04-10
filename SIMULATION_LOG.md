# Simulation Log — Jamming-Aware WSN Project

Each entry documents one simulation run: the parameters used, the state of the code at the time, what was being tested, and what the results mean.

---

## Run 001 — Baseline Sanity Check (Proposed Scheme, No Baselines)

**Date:** 2026-04-10
**Run by:** Ahmed

### What This Run Was

First end-to-end run of the proposed scheme after the initial codebase was written. The goal was to verify the simulation runs without errors and that the output metrics are physically reasonable — not to draw comparative conclusions yet (baselines are not implemented). Think of this as a sanity check and a reference point for all future runs.

### Code State

- All core modules implemented: `config.m`, `init_network.m`, `uav_trajectory.m`, `compute_packet_success.m`, `compute_energy.m`, `update_jamming_risk.m`, `elect_ch_proposed.m`, `route_dijkstra.m`, `run_proposed.m`, `plot_results.m`
- Bug fixed prior to run: `r_c = 15` was missing from `config.m` — added before running
- Baselines: **not yet implemented**
- RNG seed: `rng(42)` — fixed for reproducibility

### Parameters Used

| Parameter | Value | Description |
|---|---|---|
| `N` | 100 | Number of sensor nodes |
| `area` | 100 m | Field side length (100×100 m deployment) |
| `BS` | [50, 50] | Base station at field center |
| `E0` | 0.5 J | Initial energy per node |
| `T` | 1000 | Total simulation rounds |
| `K_elec` | 10 | CH re-election interval (rounds) |
| `M` | 10 | Burst size (packets/round) |
| `p_CH` | 0.05 | Target CH fraction (dynamic K = round(0.05 × N_alive)) |
| `r_c` | 15 m | Communication range for neighbor counting |
| `r_exc` | 25 m | CH spatial exclusion radius |
| `lambda` | 0.6 | EWMA smoothing constant |
| `alpha` | 0.35 | CHScore weight — residual energy |
| `beta` | 0.20 | CHScore weight — connectivity |
| `gamma_` | 0.35 | CHScore weight — jamming risk penalty |
| `delta` | 0.10 | CHScore weight — distance-to-BS penalty |
| `phi1` | 1e-4 | Routing cost — fixed per-hop penalty (J) |
| `phi2` | 1 | Routing cost — energy term scale |
| `phi3` | 5e-4 | Routing cost — JR penalty scale (J) |
| `r_j` | 20 m | Jamming radius |
| `kappa` | 3 | Jamming decay constant |
| `p_base` | 0.95 | Baseline packet success probability |
| `orbit_radius` | 35 m | UAV orbit radius |
| `omega` | 2π/50 rad/round | UAV angular speed (1 orbit per 50 rounds) |
| `E_elec` | 50 nJ/bit | Circuit energy |
| `E_amp` | 100 pJ/bit/m² | Amplifier energy |
| `E_da` | 5 nJ/bit | Data aggregation energy |
| `L` | 4000 bits | Packet length |

### Results

| Metric | Value |
|---|---|
| **First node death (t_death)** | Round 318 |
| Alive nodes @ round 100 | 100 / 100 |
| Alive nodes @ round 200 | 100 / 100 |
| Alive nodes @ round 300 | 100 / 100 |
| Alive nodes @ round 400 | 98 / 100 |
| Alive nodes @ round 500 | 97 / 100 |
| Alive nodes @ round 1000 | 1 / 100 |
| PDR mean (rounds 1–100) | 88.40% |
| PDR mean (rounds 300–318) | 89.02% |
| PDR mean (all 1000 rounds) | 80.96% |
| PDR min (any round) | 10.00% |
| PDR max (any round) | 100.00% |
| Avg end-to-end delay | 1.25 hops |
| Max delay (any round) | 2 hops |
| Total residual energy @ round 100 | 43.42 J |
| Total residual energy @ round 200 | 36.62 J |
| Total residual energy @ round 300 | 29.79 J |
| Total residual energy @ round 318 | 28.60 J |

### Takeaways

**1. The scheme is functionally correct.**
No crashes, no NaN metrics, no zero-PDR rounds in the healthy phase. All modules are wiring together properly.

**2. Network lifetime of 318 rounds is reasonable, and degradation is slow.**
All 100 nodes survive until round 318. After the first death, the network degrades gracefully — 97 nodes are still alive at round 500. This indicates the energy load is fairly balanced across nodes, which is the intended effect of the CHScore energy term and the spatial exclusion radius.

**3. PDR is strong and stable in the active phase (~88–89%).**
In the healthy network phase (rounds 1–318), PDR holds consistently between 88–89%. The overall 1000-round mean drops to ~81% only because the degraded network in later rounds (fewer CHs, broken routing paths) pulls it down — not because the jamming mitigation is failing.

**4. Routing is almost always 1 hop to BS.**
Average delay of 1.25 hops (max 2) means CHs are routing directly to the BS nearly all the time rather than through relay CHs. With 5 CHs spread across 100×100 m and BS at center [50, 50], most CHs are within direct range. This is expected geometry — multi-hop routing would be more prominent in larger fields or with BS at a corner.

**5. Energy drains at a linear, balanced rate.**
Total energy consumption is approximately 0.068 J/round throughout the entire healthy phase:
- Rounds 0→100: −6.58 J
- Rounds 100→200: −6.80 J
- Rounds 200→300: −6.82 J

No accelerating drain, no hotspot death spiral. This is a good sign for the clustering algorithm.

**6. The PDR floor of ~10% reflects the jamming model working correctly.**
The minimum PDR of 0.10 occurs when the UAV flies directly over a cluster. At the jamming center, `p = p_base * exp(-kappa) ≈ 0.95 × e^{-3} ≈ 0.047`. With M=10 stochastic packet draws, a node with p≈0.05 will frequently deliver 0 or 1 packets, pushing cluster PDR down transiently. This is the intended behavior — not a bug.

### What to Do Next

- Implement the three baselines (`run_baseline1.m`, etc.) and run a comparative simulation
- Once baselines are ready, re-run with identical RNG seed (`rng(42)`) so all schemes see the same network and packet draws
- Consider running multiple seeds and averaging to reduce variance in the comparison

---
