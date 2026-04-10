# WSN Jamming-Aware Simulation — README for Claude Code

## Project Overview
This is a MATLAB simulation for a graduate wireless networks course project titled:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

The simulation implements and evaluates a proposed heuristic scheme that integrates jamming risk into both cluster head (CH) selection and inter-cluster routing in a WSN under a moving UAV-based jammer.

---

## How to Run
Open MATLAB, navigate to this folder, and run:
```matlab
main.m
```
This is the only entry point. Everything else is called from main.m.

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

| File | Role | Layer |
|---|---|---|
| `config.m` | All simulation parameters — edit values here | 1 |
| `init_network.m` | Deploys 100 nodes randomly, initializes state vectors | 1 |
| `uav_trajectory.m` | Precomputes circular UAV orbit J(t) for all T rounds | 1 |
| `compute_packet_success.m` | Computes p_i(t) per node using jammer distance (Eq. 2) | 1 |
| `compute_energy.m` | LEACH radio energy model: tx, rx, agg, overhead cases | 1 |
| `update_jamming_risk.m` | PDR burst → EWMA → JR update (Eq. 3, 4, 5) | 2 |
| `elect_ch_proposed.m` | CHScore election with greedy spatial suppression (Eq. 6) | 2 |
| `route_dijkstra.m` | Builds inter-cluster graph, runs Dijkstra with C(s_i,s_j,t) (Eq. 7, 8) | 2 |
| `run_proposed.m` | Main round loop tying all modules together | 2 |
| `main.m` | Entry point: seeds RNG, calls init, runs scheme, plots | 4 |
| `plot_results.m` | Plots PDR, energy, delay, alive nodes vs round | 4 |

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
- Communication range for neighbor counting: r_c=15m

**3. Inter-Cluster Routing**
- Path cost: C(i,j,t) = φ1 + φ2·ε_amp·L·d²(i,j) + φ3·JR_j
- φ1=1e-4, φ2=1, φ3=5e-4
- Dijkstra's algorithm on weighted CH graph
- BS treated as node N+1 with JR=0

### Radio Energy Model (Heinzelman et al. LEACH)
- E_elec = 50 nJ/bit (circuit energy)
- E_amp = 100 pJ/bit/m² (amplifier energy)
- E_da = 5 nJ/bit (aggregation energy at CH)
- L = 4000 bits per packet

---

## Output Metrics (per round)
- **PDR(t)** — ratio of packets successfully received at CHs to total sent
- **energy(t)** — total residual energy across all alive nodes (J)
- **delay(t)** — average hop count from CH to BS across active paths
- **alive(t)** — number of alive nodes
- **t_death** — round number of first node death (network lifetime metric)

---

## Variable Naming Conventions
- All node state vectors are length N (index = node number)
- `alive` — logical vector, true = node is alive
- `is_CH` — logical vector, true = node is elected CH this round
- `CH_assign(i)` — index of the CH that node i belongs to
- `gamma_` used instead of `gamma` (MATLAB built-in conflict)
- `K_elec` = re-election interval (rounds); `K` = number of CHs to elect (computed dynamically)
- BS is treated as node N+1 inside route_dijkstra.m

---

## Current Project Status

| Component | Status |
|---|---|
| Proposed scheme (`run_proposed.m`) | Done |
| Standard LEACH baseline (`run_leach.m`) | Done |
| Baseline 1 — EWMA Detection | Not started |
| Baseline 2 — TBC | Not started |
| Baseline 3 — FCPA simplified | Not started |
| Multi-seed averaging | Not started |
| CHScore weight tuning | Not started |

## Baselines

**Implemented:**
- `run_leach.m` — Standard LEACH (Heinzelman et al.). Probabilistic CH election, direct CH→BS transmission, no jamming awareness. Uses same energy model and network topology as proposed scheme for fair comparison.

**Planned:**
1. **Baseline 1 — EWMA Detection** (Paper: IEEE ICDSIS 2022): detection-only, no re-election or routing adaptation
2. **Baseline 2 — TBC** (Paper: IEEE MRIE 2025): PDR threshold, flat network, topological node removal
3. **Baseline 3 — FCPA simplified** (Paper: Sensors/Springer 2023): reactive K-medoids re-clustering on CH failure

Each baseline has its own `run_*.m` file and is added to `results_all` in `main.m`.

---

## Common Debug Entry Points
- If PDR is always 0: check `compute_packet_success.m` and `update_jamming_risk.m`
- If energy drains in 1 round: check `compute_energy.m` unit consistency (everything should be in Joules)
- If no CHs are elected: check `elect_ch_proposed.m` — r_exc may be too large relative to N_alive
- If Dijkstra returns empty paths: check `route_dijkstra.m` — graph may be disconnected if CHs are too far apart
- If plots are flat/zero: check that `run_proposed.m` is correctly packaging results into the struct

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