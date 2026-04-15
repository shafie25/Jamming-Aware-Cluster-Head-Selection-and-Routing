# WSN Jamming-Aware Simulation - README

## Project Overview

MATLAB simulation for the course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

The code evaluates a proposed heuristic that integrates jamming risk (JR) into cluster head (CH) selection and inter-cluster routing under a moving UAV jammer, and compares it against a standard LEACH baseline.

---

## How to Run

**Single seed (quick check):**
```matlab
main.m
```

**Multi-seed averaged results (current default: 20 seeds):**
```matlab
run_multiseed.m
```

All sensitivity, diagnostic, and comparison scripts live in `testing/` and should be run from the project root:

```matlab
% Sensitivity sweeps
testing/run_lambda_sensitivity.m   % EWMA lambda sweep (proposed only)
testing/run_phi1_sweep.m           % phi1 per-hop penalty sweep (proposed only)

% Routing experiments
testing/run_routing_comparison.m   % Dijkstra vs direct CH-to-BS vs LEACH
testing/run_geometry_test.m        % BS position geometry test (center/edge/corner)

% Diagnostics
testing/diag_proposed_zeros.m      % per-round zero-PDR cause breakdown (proposed)
testing/diag_leach_zeros.m         % per-round zero-PDR cause breakdown (LEACH)
```

**Network snapshot visualization:**
```matlab
plotting/visualize_snapshot.m      % set snapshot_round and seed at top of file
```

---

## Current Canonical Configuration

| Parameter | Value | Notes |
|---|---|---|
| `N` | 100 | sensor nodes |
| `area` | 100 m | 100x100m field |
| `BS` | [50, 50] | field center |
| `E0` | 0.5 J | initial energy per node |
| `T` | 1000 | simulation rounds |
| `r_j` | 20 m | jamming radius |
| `kappa` | 10 | jamming decay constant |
| `p_base` | 0.95 | baseline packet success probability |
| `K_elec` | 5 | CH re-election interval (rounds) |
| `lambda` | 0.6 | EWMA smoothing constant |
| `p_CH` | 0.05 | target CH fraction (~5 CHs) |
| `r_c` | 15 m | neighbor counting radius (CHScore only, not a radio limit) |
| `r_exc` | 25 m | CH spatial exclusion radius |
| `r_tx` | 50 m | hard member-to-CH transmission limit |
| `phi1` | 5e-4 | Dijkstra per-hop penalty (J) |
| `phi2` | 1 | Dijkstra energy scale |
| `phi3` | 5e-4 | Dijkstra JR penalty (J) |
| seeds | `1:20` | default multi-seed range |

---

## Proposed Scheme

### 1. Jamming Risk Estimation

Each alive non-CH node sends a burst of `M = 10` packets per round. Instantaneous PDR is EWMA-smoothed:

```
PDR_ewma = lambda * PDR_inst + (1 - lambda) * PDR_ewma
JR = 1 - PDR_ewma
```

### 2. CH Election (core contribution)

Every `K_elec = 5` rounds, alive nodes are scored and CHs elected greedily with spatial suppression:

```
CHScore = alpha*(E/E0) + beta*(|N_i|/N_max) - gamma_*JR - delta*(d_BS/d_max)
```

Weights: `alpha=0.35, beta=0.20, gamma_=0.35, delta=0.10` (sum to 1.0).
Spatial exclusion radius `r_exc=25m` ensures field-wide CH coverage.

### 3. Emergency CH Re-election

If a non-election round begins with no alive CHs (all died since last scheduled election), an emergency election is triggered immediately using the same CHScore logic and paying the same control overhead.

### 4. Inter-Cluster Routing

CHs route to the BS via Dijkstra on a fully connected CH graph:

```
C(i,j) = phi1 + phi2 * E_amp * L * d(i,j)^2 + phi3 * JR_j
```

> **Note (Run 015):** Routing experiments showed that in this compact geometry (BS at center, 100x100m), direct CH-to-BS performs equivalently to Dijkstra — ~78% of the field is within r_tx=50m of the BS. The JR-aware election layer accounts for the full PDR advantage over LEACH. Dijkstra is retained as a framework component; it would contribute meaningfully in larger or asymmetric deployments.

---

## Important Modeling Notes

### Packet Success Model

```
p = p_base * exp(-kappa * (1 - d/r_j))   for d <= r_j
p = p_base                                 for d > r_j
```

With `kappa=10`, a node at the jammer center has `p ≈ 0.00004` — effectively zero.

### r_c vs r_tx

- `r_c = 15m` — neighbor counting radius for the CHScore connectivity term only. **Not a radio limit.**
- `r_tx = 50m` — hard member-to-CH transmission limit. Nodes farther than this from every CH are stranded and their packets count as lost in the PDR denominator.

### PDR Reporting (three windows)

| Window | Formula | Purpose |
|---|---|---|
| All T rounds | `mean(PDR)` | Full lifecycle average |
| FND-truncated | `mean(PDR(1:t_death))` | Operational period only |
| Zero-PDR count | `sum(PDR == 0)` | Communication blackout rounds |

---

## Current Best Results (Run 014, 20 seeds, phi1=5e-4)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 605.7 +/- 27.6 | **726.6 +/- 30.4** |
| PDR all rounds (%) | **71.59 +/- 1.64** | 62.20 +/- 0.85 |
| PDR FND-trunc (%) | **82.20 +/- 1.32** | 72.81 +/- 2.91 |
| Zero-PDR rounds | **0.0 +/- 0.0** | 164.5 +/- 15.7 |
| Energy @ round 300 (J) | 31.62 +/- 0.43 | **32.39 +/- 1.25** |

Proposed wins on PDR (+9.4pp all-rounds, +9.4pp FND-trunc) and eliminates communication blackouts entirely. LEACH outlasts the proposed scheme to first node death by ~121 rounds because multi-hop relay load concentrates energy drain on a subset of CHs.

---

## File Structure

```
main.m                          single-seed entry point
run_multiseed.m                 main 20-seed evaluation entry point
CLAUDE.md                       session context for Claude Code

core/
  config.m                      all simulation parameters
  init_network.m                node deployment and initial state
  uav_trajectory.m              precomputed jammer trajectory
  compute_packet_success.m      packet success probability model
  compute_energy.m              LEACH-style energy model

layer1/
  update_jamming_risk.m         EWMA JR estimation
  elect_ch_proposed.m           CHScore election with spatial suppression

layer2/
  route_dijkstra.m              JR-aware inter-cluster routing

schemes/
  run_proposed.m                proposed scheme (Dijkstra routing)
  run_proposed_direct.m         proposed scheme (direct CH-to-BS, no Dijkstra)
  run_leach.m                   standard LEACH baseline

plotting/
  plot_results.m                single-seed plots
  plot_multiseed.m              mean +/- std band plots
  visualize_snapshot.m          2D round snapshot with JR heatmap

testing/
  run_lambda_sensitivity.m      EWMA lambda sweep
  run_phi1_sweep.m              phi1 per-hop penalty sweep
  run_routing_comparison.m      Dijkstra vs direct CH-to-BS vs LEACH
  run_geometry_test.m           BS position geometry test
  diag_proposed_zeros.m         proposed zero-PDR round diagnostics
  diag_leach_zeros.m            LEACH zero-PDR round diagnostics

docs/
  README.md                     this file
  SIMULATION_LOG.md             permanent record of all simulation runs
  proposed_model_for_paper_search.md  literature context

reference/
  LEACH.m                       original LEACH reference implementation
```

---

## Common Debug Entry Points

- **PDR always 0:** check `compute_packet_success.m` and `update_jamming_risk.m`
- **Energy drains too fast:** check `compute_energy.m` units and path-counting logic in `run_proposed.m`
- **No CHs elected:** check `elect_ch_proposed.m` — is `r_exc` too large for current `N_alive`?
- **Many stranded nodes:** inspect `r_tx`, CH spacing, and end-of-life topology via `visualize_snapshot.m`
- **Plots flat or zero:** confirm result struct is filled correctly in the scheme runner

---

## Logging Simulation Results

Every major run should be added to `docs/SIMULATION_LOG.md` with:
- date and who ran it
- code changes since the previous run
- full parameter snapshot
- results table
- takeaways and next steps
