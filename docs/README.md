# WSN Jamming-Aware Simulation - README

## Project Overview

MATLAB simulation for the course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

The code evaluates a proposed heuristic that integrates jamming risk into both cluster head (CH) selection and inter-cluster routing under a moving UAV jammer, and compares it against a standard LEACH baseline.

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

**Lambda sensitivity (proposed scheme only):**
```matlab
run_lambda_sensitivity.m
```

**Network snapshot visualization at a specific round:**
```matlab
plotting/visualize_snapshot.m
```

---

## Current Canonical Configuration

- `N = 100`
- Field size: `100 x 100 m`
- BS at `[50, 50]`
- `E0 = 0.5 J`
- `T = 1000`
- `r_j = 20 m`
- `kappa = 10`
- `p_base = 0.95`
- `K_elec = 5`
- `lambda = 0.6`
- `p_CH = 0.05`
- `r_c = 15 m`
- `r_exc = 25 m`
- `r_tx = 50 m`
- `run_multiseed.m` currently uses seeds `1:20`

---

## Proposed Scheme

### 1. Jamming Risk Estimation

Each alive non-CH node sends a burst of `M = 10` packets per round. Instantaneous PDR is EWMA-smoothed using `lambda = 0.6`, and jamming risk is defined as:

```text
JR = 1 - PDR_ewma
```

### 2. CH Election

Every `K_elec = 5` rounds, alive nodes are scored using:

```text
CHScore = alpha*(E/E0) + beta*(|N|/Nmax) - gamma*JR - delta*(dBS/dmax)
```

with:
- `alpha = 0.35`
- `beta = 0.20`
- `gamma_ = 0.35`
- `delta = 0.10`

Greedy spatial suppression with `r_exc = 25 m` is used to spread CHs across the field.

### 3. Emergency CH Re-election

`run_proposed.m` performs an emergency CH re-election only when a round starts with **no alive CHs**. This uses the same election logic as a scheduled election and pays the same control overhead. It removes the artificial `CHs = 0` blackout window caused by all CHs dying between scheduled elections.

### 4. Inter-Cluster Routing

CHs route to the BS using Dijkstra on a fully connected CH graph with per-edge cost:

```text
C(i,j,t) = phi1 + phi2*E_amp*L*d(i,j)^2 + phi3*JR_j
```

with:
- `phi1 = 1e-4`
- `phi2 = 1`
- `phi3 = 5e-4`

---

## Important Modeling Notes

### Packet Success Model

Packet success degrades exponentially inside the jamming radius:

```text
p = p_base * exp(-kappa * (1 - d/r_j))
```

With `kappa = 10`, a node directly under the jammer has near-zero delivery probability.

### `r_c` vs `r_tx`

- `r_c = 15 m` is only a **neighbor counting radius** for the CHScore connectivity term.
- `r_tx = 50 m` is the hard **member-to-CH transmission limit**.
- Nodes farther than `r_tx` from every CH are stranded and their packets count as lost in the PDR denominator.

### PDR Reporting

The project reports three complementary PDR windows:

- **All T rounds**: `mean(PDR)`
- **FND-truncated**: `mean(PDR(1:t_death))`
- **Zero-PDR round count**: `sum(PDR == 0)`

This avoids ambiguity around whether PDR is measured over the full lifecycle or only the operational window.

---

## Current Best Results

Current default evaluation is the 20-seed run from `run_multiseed.m`.

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | **603.0 ± 35.1** | 726.6 ± 30.4 |
| PDR all rounds (%) | **71.24 ± 1.58** | 62.20 ± 0.85 |
| PDR FND-trunc (%) | **82.03 ± 1.28** | 72.81 ± 2.91 |
| Zero-PDR rounds | **1.2 ± 5.6** | 164.5 ± 15.7 |
| Energy @ round 300 (J) | 31.61 ± 0.43 | **32.39 ± 1.25** |

Interpretation:
- Proposed is clearly better on delivered data and blackout avoidance.
- LEACH still lasts longer to first node death and keeps slightly more residual energy because the proposed multi-hop layer concentrates forwarding load on relay CHs.
- The remaining proposed zero-PDR rounds are late end-of-life events, typically when one CH remains and almost no reachable members are left.

---

## Recent Changes Completed

- Added emergency CH re-election in `run_proposed.m` when a round begins with no alive CHs.
- Updated `diag_proposed_zeros.m` to mirror the emergency re-election rule.
- Fixed `elect_ch_proposed.m` so `p_CH` is now read from `core/config.m` through function inputs instead of being hardcoded internally.
- Expanded `run_multiseed.m` from the original 5-seed default to `1:20`.
- Updated `docs/SIMULATION_LOG.md` and `docs/proposed_model_for_paper_search.md` to match the current implementation.

## Next Steps

- Add residual-energy awareness to `route_dijkstra.m` so overloaded relay CHs are penalized during path selection.
- Run a small tuning sweep over `alpha`, `beta`, `gamma_`, `delta` to improve average PDR without collapsing lifetime.
- Run a routing-weight sweep over `phi1` and `phi3` to reduce unnecessary relays and average hop delay.
- Consider adaptive CH density only in late rounds, rather than increasing `p_CH` globally.
- Keep zero-PDR rounds as a secondary issue for now; the main optimization target is average PDR and the lifetime-delay trade-off.

---

## File Structure

| File | Role |
|---|---|
| `core/config.m` | Central simulation parameters |
| `core/init_network.m` | Node deployment and initial state |
| `core/uav_trajectory.m` | Precomputed jammer trajectory |
| `core/compute_packet_success.m` | Packet success probability model |
| `core/compute_energy.m` | LEACH-style energy model |
| `layer1/update_jamming_risk.m` | Instantaneous PDR, EWMA, JR update |
| `layer1/elect_ch_proposed.m` | CHScore-based election with spatial suppression |
| `layer2/route_dijkstra.m` | JR-aware inter-cluster routing |
| `schemes/run_proposed.m` | Proposed scheme round loop |
| `schemes/run_leach.m` | LEACH baseline round loop |
| `run_multiseed.m` | Current multi-seed evaluation entry point |
| `run_lambda_sensitivity.m` | Lambda sensitivity sweep |
| `plotting/plot_multiseed.m` | Mean ± std plots |
| `plotting/visualize_snapshot.m` | Round snapshot visualization |
| `diag_proposed_zeros.m` | Proposed zero-PDR diagnostics |
| `diag_leach_zeros.m` | LEACH zero-PDR diagnostics |
| `docs/SIMULATION_LOG.md` | Permanent record of simulation runs |

---

## Common Debug Entry Points

- If PDR is always 0: check `compute_packet_success.m` and `update_jamming_risk.m`
- If energy drains too fast: check `compute_energy.m` units and path-counting logic
- If no CHs are elected: check `elect_ch_proposed.m` and whether `r_exc` is too large for current `N_alive`
- If many nodes become stranded: inspect `r_tx`, CH spacing, and end-of-life topology
- If plots are flat or zero: confirm the result struct is being filled correctly in the scheme runner

---

## Logging Simulation Results

Every major run should be added to `docs/SIMULATION_LOG.md` with:
- date and who ran it
- code changes since the previous run
- full parameter snapshot
- results table
- takeaways
- next steps
