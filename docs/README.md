# WSN Jamming-Aware Simulation — README for Claude Code

## Project Overview
This is a MATLAB simulation for a graduate wireless networks course project titled:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

The simulation implements and evaluates a proposed heuristic scheme that integrates jamming risk into both cluster head (CH) selection and inter-cluster routing in a WSN under a moving UAV-based jammer.

---

## How to Run

**Single seed (quick check):**
```matlab
main.m
```

**5-seed averaged results (final paper numbers):**
```matlab
run_multiseed.m
```

**Network state visualization at a specific round:**
```matlab
plotting/visualize_snapshot.m   % set snapshot_round at the top, or leave 0 for random
```

## Logging Simulation Results
Every time you run a simulation, add an entry to `SIMULATION_LOG.md`. This keeps a permanent record of what was tested, what parameters were used, and what the results looked like.

Each entry should include:
- **Date and who ran it**
- **Code state** — what was implemented or changed since the last run, any bugs fixed, the RNG seed used
- **Full parameters table** — copy from `config.m` at the time of the run (parameters change over time, so record them per-run)
- **Results table** — `t_death`, PDR, energy, delay, alive node counts at key rounds
- **Takeaways** — what the results mean, anything unexpected
- **What to do next** — follow-up actions or open questions

See `SIMULATION_LOG.md` for the format — use Run 001 as a template for all subsequent entries.

---

## File Structure

| File | Role |
|---|---|
| `core/config.m` | All simulation parameters — edit values here |
| `core/init_network.m` | Deploys 100 nodes randomly, initializes state vectors |
| `core/uav_trajectory.m` | Precomputes circular UAV orbit J(t) for all T rounds |
| `core/compute_packet_success.m` | Computes p_i(t) per node using jammer distance (Eq. 2) |
| `core/compute_energy.m` | LEACH radio energy model: tx, rx, agg, overhead cases |
| `layer1/update_jamming_risk.m` | PDR burst → EWMA → JR update (Eq. 3, 4, 5) |
| `layer1/elect_ch_proposed.m` | CHScore election with greedy spatial suppression (Eq. 6) |
| `layer2/route_dijkstra.m` | Builds inter-cluster graph, runs Dijkstra with C(i,j,t) (Eq. 7, 8) |
| `schemes/run_proposed.m` | Proposed scheme round loop |
| `schemes/run_leach.m` | Standard LEACH baseline round loop |
| `schemes/run_ewma_detect.m` | Baseline 1: LEACH + EWMA detection (unused) |
| `schemes/run_threshold.m` | Baseline 2: LEACH + member suppression when JR > 0.70 |
| `schemes/run_reactive_ch.m` | Baseline 3: LEACH + reactive CH re-election when CH JR > 0.50 |
| `main.m` | Single-seed entry point |
| `run_multiseed.m` | 5-seed averaging entry point (currently Proposed + LEACH only) |
| `plotting/plot_results.m` | 4-panel results figure (single seed) |
| `plotting/plot_multiseed.m` | Mean ± std band plots (multi-seed) |
| `plotting/visualize_snapshot.m` | 2D network map at a specific round (clusters, JR, routing, stranded nodes) |

---

## Key Design Decisions

### Network
- N = 100 nodes, 100×100m field
- BS fixed at field center [50, 50]
- Nodes static, randomly deployed with rng(42)
- Initial energy E0 = 0.5J per node

### UAV Jammer
- Circular orbit: center [50,50], radius 35m, angular speed 2π/50 rad/round
- One full orbit every 50 rounds
- Jamming radius r_j = 20m
- Jammer modeled in 2D (ground plane projection)
- Packet success probability degrades exponentially inside r_j (Eq. 2, kappa=3)

### Proposed Scheme — Three Core Components

**1. Jamming Risk (JR)**
- Each node transmits M=10 packets per round to its CH
- Instantaneous PDR = packets received / M
- EWMA smoothing: PDR_ewma = λ·PDR + (1-λ)·PDR_ewma_prev, λ=0.6
- JR = 1 - PDR_ewma ∈ [0,1]

**2. CH Election (every K_elec=10 rounds)**
- CHScore = α·(E/E0) + β·(|N|/N_max) - γ·JR - δ·(d_BS/d_max)
- Weights: α=0.35, β=0.20, γ=0.35, δ=0.10
- Dynamic K = round(0.05 × N_alive) CHs elected per round
- Greedy spatial election with r_exc=25m exclusion radius
- `r_c=15m` — **neighbor counting range only** (see note below)

**3. Inter-Cluster Routing**
- Path cost: C(i,j,t) = φ1 + φ2·ε_amp·L·d²(i,j) + φ3·JR_j
- φ1=1e-4, φ2=1, φ3=5e-4
- Dijkstra's algorithm on weighted CH graph
- BS treated as node N+1 with JR=0

---

## Parameter Derivations

### r_exc — CH Exclusion Radius

`r_exc = 25m` is not an arbitrary value. It is derived from the expected spatial footprint of each CH.

With K = round(p_CH × N) = round(0.05 × 100) = 5 CHs covering a 100×100m = 10,000 m² field:

```
area per CH  = 10,000 / 5  = 2,000 m²
r_exc        = sqrt(area_per_CH / π)
             = sqrt(2,000 / π)
             ≈ 25.2 m  → rounded to 25m
```

This ensures each elected CH suppresses the nodes within its expected coverage zone, guaranteeing spatial spread across the field. If `p_CH` is changed, `r_exc` must be updated using the same formula:

```
r_exc = sqrt(area² / (p_CH × N × π))
```

**Why this matters:** If r_exc is too small relative to K, CHs cluster together and leave field edges uncovered. If too large, the greedy election cannot find K non-suppressed nodes and elects fewer CHs than intended — tested and confirmed in Run 008 when p_CH was raised to 0.10 without correctly accounting for overlap.

### r_tx — Transmission Range Limit

`r_tx = 50m` is derived as approximately 2× the average nearest-CH distance in a healthy network.

With K = 5 CHs deployed randomly across 10,000 m², the average nearest-CH distance follows from the Poisson point process approximation:

```
λ_CH         = K / area²  = 5 / 10,000  = 0.0005 CHs/m²
avg nearest  = 1 / (2 × sqrt(λ_CH))
             = 1 / (2 × sqrt(0.0005))
             ≈ 22.4 m
```

Setting `r_tx = 50m ≈ 2.2 × avg_nearest` means:
- **Healthy rounds (100 nodes, 5 CHs):** virtually no stranded nodes — almost all members within 50m of their nearest CH
- **Mid rounds (2 CHs):** average nearest distance ≈ 35m → some peripheral nodes begin stranding
- **Late rounds (1 CH):** average nearest distance ≈ 50m → meaningful stranding, honestly captured in PDR

The limit is intentionally generous to avoid artificially penalising healthy-round PDR. Its primary effect is in the network degradation phase.

**Stranded node accounting:** Stranded nodes (CH_assign = 0) skip all transmission. Their M packets are added to `total_sent` with zero contribution to `total_recv` — honest PDR accounting consistent with the threshold suppression approach in run_threshold.m.

### r_j — Jamming Radius

`r_j = 20m` is sized so the jammer threatens 1–2 CHs per round at most.

With 5 CHs spread across 10,000 m², the average inter-CH distance is ~45m. A jamming radius of 20m ensures the jammer can cover at most one full CH coverage zone per round, making jamming a meaningful but not overwhelming threat. At r_j = 20m, packet success at the jamming center drops to:

```
p = p_base × exp(−kappa) = 0.95 × e^{−3} ≈ 0.047
```

So a node directly under the UAV delivers roughly 0–1 out of 10 packets per round.

### r_c — Neighbor Counting Range

`r_c = 15m` is used **only** in `elect_ch_proposed.m` to count how many alive nodes are within communication range of a candidate CH (the β connectivity term in CHScore). It is **not** a radio range limit for data transmission.

The value 15m is smaller than r_exc (25m) by design — it counts only close neighbors, giving the β term sensitivity to local density without double-counting nodes already covered by adjacent CHs.

---

## Radio Range — Important Modeling Note

`r_c = 15m` is used **only** for counting neighbors in the CHScore beta term. It is **not** a hard radio range limit.

`r_tx = 50m` is the hard transmission range limit added in Run 007. Member nodes beyond 50m from every CH are stranded — they cannot join any cluster and their packets count as lost. This is the only range check in the simulation.

Cluster assignment (`elect_ch_proposed.m`) joins every alive non-CH node to its nearest CH within `r_tx`. Members beyond `r_tx` from all CHs get `CH_assign = 0`. The d² energy model additionally penalizes long links within the reachable set.

**Practical implication in late rounds:** With 1–2 CHs remaining, some nodes will be stranded. This is physically honest — a node 60m from its nearest CH genuinely cannot communicate in a real WSN with limited radio range.

---

## Radio Energy Model (Heinzelman et al. LEACH)
- E_elec = 50 nJ/bit (circuit energy)
- E_amp = 100 pJ/bit/m² (amplifier energy)
- E_da = 5 nJ/bit (aggregation energy at CH)
- L = 4000 bits per packet

---

## Output Metrics (per round)
- **PDR(t)** — ratio of packets successfully received at CHs to total sent (including stranded nodes' lost packets)
- **energy(t)** — total residual energy across all alive nodes (J)
- **delay(t)** — average hop count from CH to BS across active paths
- **alive(t)** — number of alive nodes
- **t_death** — round number of first node death (network lifetime metric)

---

## Variable Naming Conventions
- All node state vectors are length N (index = node number)
- `alive` — logical vector, true = node is alive
- `is_CH` — logical vector, true = node is elected CH this round
- `CH_assign(i)` — index of the CH that node i belongs to; **0 = stranded** (no CH within r_tx)
- `gamma_` used instead of `gamma` (MATLAB built-in conflict)
- `K_elec` = re-election interval (rounds); `K` = number of CHs to elect (computed dynamically)
- BS is treated as node N+1 inside route_dijkstra.m

---

## Current Project Status

| Component | Status |
|---|---|
| Proposed scheme (`run_proposed.m`) | Done — r_tx active |
| Standard LEACH baseline (`run_leach.m`) | Done — r_tx active |
| Baseline 1 — EWMA Detection (`run_ewma_detect.m`) | Done — **r_tx not yet added** |
| Baseline 2 — Threshold-JR (`run_threshold.m`) | Done — **r_tx not yet added** |
| Baseline 3 — Reactive-CH (`run_reactive_ch.m`) | Done — **r_tx not yet added** |
| Multi-seed averaging (`run_multiseed.m`) | Done — 2 schemes only until baselines updated |
| Network snapshot visualization | Done — shows stranded nodes as yellow diamonds |
| Paper writing | In progress |

## Baselines

| Scheme | File | Design summary |
|---|---|---|
| Standard LEACH | `run_leach.m` | Probabilistic CH election, direct CH→BS, no jamming awareness |
| EWMA-Detect | `run_ewma_detect.m` | LEACH + EWMA JR tracking — detection computed but never used in decisions |
| Threshold-JR | `run_threshold.m` | LEACH + member transmission suppressed when JR > 0.70 |
| Reactive-CH | `run_reactive_ch.m` | LEACH + reactive CH re-election when elected CH JR > 0.50 |

---

## Common Debug Entry Points
- If PDR is always 0: check `compute_packet_success.m` and `update_jamming_risk.m`
- If energy drains in 1 round: check `compute_energy.m` unit consistency (everything should be in Joules)
- If no CHs are elected: check `elect_ch_proposed.m` — r_exc may be too large relative to N_alive
- If Dijkstra returns empty paths: check `route_dijkstra.m` — graph may be disconnected if CHs are too far apart
- If plots are flat/zero: check that `run_proposed.m` is correctly packaging results into the struct
- If "Array indices must be positive integers": a stranded node (CH_assign=0) is being used as an index — check for missing `if CH_assign(i) == 0; continue; end` guard

---

## Paper Equations Reference
- Eq. 1: d_iJ(t) — distance from node to jammer
- Eq. 2: p_i(t) — packet success probability
- Eq. 3: PDR_i(t) — instantaneous PDR
- Eq. 4: PDR_ewma — EWMA smoothed PDR
- Eq. 5: JR_i(t) — jamming risk score
- Eq. 6: CHScore — cluster head election metric
- Eq. 7: C(s_i,s_j,t) — inter-cluster link cost
- Eq. 8: P* — optimal path via Dijkstra
