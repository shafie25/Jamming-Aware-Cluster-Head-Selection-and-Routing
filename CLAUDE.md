# CLAUDE.md — Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes, 100×100m field, BS at center [50,50]. A UAV jammer follows a circular orbit and degrades packet delivery. The proposed scheme integrates jamming risk (JR) into CH election and inter-cluster routing to outperform standard LEACH under jamming.

RNG seed is fixed at `rng(42)` in `main.m` — all schemes see the identical network topology and packet draws.

---

## Current State (as of 2026-04-10)

### Implemented
- `run_proposed.m` — full proposed scheme: JR estimation (EWMA), CHScore election, Dijkstra routing
- `run_leach.m` — standard LEACH baseline: probabilistic CH election, direct CH→BS, no jamming awareness

### Not Yet Implemented
- Baselines 1–3 from README (EWMA Detection, TBC, FCPA simplified)
- Multi-seed averaging
- CHScore weight tuning

### Entry Point
`main.m` — runs both schemes, passes `results_all = {results_proposed, results_leach}` to `plot_results.m`.

---

## Results So Far (Run 002, seed 42)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death | Round 318 | Round 447 |
| PDR mean (all rounds) | 80.96% | 58.14% |
| PDR mean (active rounds only) | 88.39% | 87.96% |
| Zero-PDR rounds | 84 | 339 |
| Energy remaining @ round 300 | 29.79 J | 25.36 J |
| Avg delay | 1.25 hops | 1.00 hops |

**Key insight:** Both schemes deliver ~88% PDR when the network is actively transmitting. The proposed scheme's advantage is that it stays in a valid operating state for far more rounds (916 vs 661). LEACH wastes 339 rounds with all alive nodes elected as CHs (no members → PDR = 0) due to its epoch mechanism.

**Known trade-off:** LEACH's first node death is later (447 vs 318) despite draining energy faster overall. Cause: proposed scheme's inter-cluster routing creates relay nodes that carry extra traffic and drain faster than neighbors. LEACH is direct-to-BS only — no relay burden, more even load distribution. This is a candidate for improvement via CHScore weight tuning.

---

## Important Gotchas

**PDR reporting:** Always report mean separately for all-rounds vs active-rounds-only. The `min(PDR(PDR>0))` filter hides zero-PDR rounds — be explicit about this when comparing metrics.

**M=10 granularity:** With 10 packets per burst, PDR resolution per single node is 0.1. When the network shrinks to 1–2 nodes, per-round PDR snaps to {0, 0.1, 0.2, ...}. This is expected, not a bug.

**`gamma_` not `gamma`:** `gamma` is a MATLAB built-in. The CHScore weight is named `gamma_` everywhere — in `config.m`, `run_proposed.m`, `elect_ch_proposed.m`, and the `main.m` function call.

**`r_c` must be in `config.m`:** `r_c = 15` (communication range for neighbor counting) was missing from the original `config.m` and caused an undefined variable error. It has been added. If config.m is ever reset, this needs to be there.

**LEACH energy model:** The original `LEACH.m` script (kept in the repo for reference) used its own energy constants (epsilon_fs, epsilon_mp, d_0 threshold). `run_leach.m` was deliberately rewritten to use the same `E_elec`, `E_amp`, `E_da`, `L` from `config.m` as the proposed scheme — apples-to-apples comparison. Do not revert this.

**LEACH.m vs run_leach.m:** `LEACH.m` is the original standalone script (kept for reference). `run_leach.m` is the integrated function used in the simulation. They are different files.

---

## How to Add a New Baseline

1. Create `run_<name>.m` as a function returning a struct with fields: `PDR`, `energy`, `delay`, `alive`, `t_death`, `label` — all vectors of length T except `t_death` (scalar) and `label` (string)
2. Call it in `main.m` after the existing scheme calls, passing network state from the workspace
3. Append its result to `results_all`
4. Log the run in `SIMULATION_LOG.md`

---

## File Reference

| File | Purpose |
|---|---|
| `config.m` | All parameters — edit here, nowhere else |
| `init_network.m` | Node deployment, initial state vectors |
| `uav_trajectory.m` | Precomputes J_x, J_y for all T rounds |
| `compute_packet_success.m` | p_i(t) per node given jammer position |
| `compute_energy.m` | Energy cost for tx / rx / agg / overhead |
| `update_jamming_risk.m` | EWMA PDR → JR update |
| `elect_ch_proposed.m` | CHScore greedy election |
| `route_dijkstra.m` | Dijkstra inter-cluster routing |
| `run_proposed.m` | Proposed scheme round loop |
| `run_leach.m` | Standard LEACH baseline round loop |
| `LEACH.m` | Original LEACH script (reference only, not called) |
| `main.m` | Entry point |
| `plot_results.m` | 4-panel results figure |
| `SIMULATION_LOG.md` | Per-run results log — update after every run |

---

## What to Work on Next

1. **CHScore weight tuning** — increase `alpha` (energy weight) or `delta` (BS distance weight) to balance energy load and delay first node death beyond round 318
2. **Multi-seed runs** — run seeds {42, 7, 13, 99, 101}, average results, report mean ± std for a statistically robust comparison
3. **Remaining baselines** — implement Baselines 1–3 per README descriptions
