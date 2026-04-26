# WSN Jamming-Aware Simulation - README

## Project Overview

MATLAB simulation for the course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

The code evaluates a proposed heuristic that integrates jamming risk (JR) into cluster head (CH) selection and inter-cluster routing under a moving UAV jammer, compared against two baselines:

- **TBC** — flat multi-hop topology with instantaneous threshold-based jamming suppression (Babitha B.S. et al., IEEE MRIE 2025)
- **FCPA** — clustered topology with IPN-gated CH election and cooperative relay for jammed members (López-Vilos et al., Sensors 2023)

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

% Diagnostics
testing/diag_proposed_zeros.m      % per-round zero-PDR cause breakdown (proposed)
testing/diag_leach_zeros.m         % per-round zero-PDR cause breakdown (LEACH)
testing/diag_scale.m               % zero-PDR cause breakdown for 200x200m deployment
```

**Network snapshot visualization:**
```matlab
plotting/visualize_snapshot.m      % set snapshot_round and seed at top of file
```

**Export publication figures (PDF + PNG for Overleaf):**
```matlab
plotting/export_figures.m          % runs 20-seed sim internally, saves to figures/
```
Output: `figures/fig_combined.pdf` (3-panel PDR/Energy/Alive), plus individual `fig_pdr.pdf`, `fig_energy.pdf`, `fig_alive.pdf`.

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

Each alive non-CH node uses packet trials per round for PDR estimation. The trial count adapts to jamming risk:

```
M_eff(i) = max(M_min, round(M * (1 - JR(i))))   % M=10, M_min=2
```

Instantaneous PDR is EWMA-smoothed to obtain JR:

```
PDR_ewma = lambda * PDR_inst + (1 - lambda) * PDR_ewma
JR = 1 - PDR_ewma
```

Heavily jammed nodes (high JR) reduce transmissions to conserve energy, extending network lifetime without changing the PDR ratio since they deliver near-zero packets regardless. Energy deduction scales proportionally: `scale = M_eff/M`.

### 2. CH Election (core contribution)

Every `K_elec = 5` rounds, alive nodes are scored and CHs elected greedily with spatial suppression:

```
CHScore = alpha*(E/E0) + beta*(|N_i|/N_max) - gamma_*JR - delta*(d_BS/d_max)
```

Weights: `alpha=0.35, beta=0.20, gamma_=0.35, delta=0.10` (sum to 1.0).
Spatial exclusion radius `r_exc=25m` ensures field-wide CH coverage.

### 3. Emergency CH Re-election

Proactive emergency re-election triggers in two cases:
- A CH died in the previous round (proactive replacement before coverage gap forms)
- No alive CHs remain at the start of a round (reactive fallback)

Both cases use the same CHScore election logic and pay normal control overhead.

### 4. Inter-Cluster Routing

CHs route to the BS via Dijkstra on a fully connected CH graph:

```
C(i,j) = phi1 + phi2 * E_amp * L * d(i,j)^2 + phi3 * JR_j
```

> **Note:** In this compact geometry (BS at center, 100x100m), many CH-to-BS paths are short because much of the field is within r_tx=50m of the BS. Dijkstra is retained as the proposed framework's inter-cluster routing component.

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
- `r_tx = 50m` — hard member-to-CH transmission limit. Nodes farther than this from every CH are stranded and their packet trials count as lost in the PDR denominator.

### PDR Reporting (two windows)

| Window | Formula | Purpose |
|---|---|---|
| All T rounds | `mean(PDR)` | Full lifecycle average |
| FND-truncated | `mean(PDR(1:t_death))` | Operational period only |

---

## Current Best Results (Run 021, 20 seeds)

| Metric | Proposed | TBC | FCPA |
|---|---|---|---|
| First node death (round) | **704.7 ± 33.1** | 459.1 ± 41.3 | 542.1 ± 19.2 |
| PDR all rounds (%) | **85.11 ± 2.02** | 53.20 ± 4.41 | 63.35 ± 0.89 |
| PDR FND-trunc (%) | **88.77 ± 1.42** | 82.42 ± 0.58 | 75.07 ± 1.44 |
| Energy @ round 300 (J) | **33.95 ± 0.41** | 26.80 ± 1.12 | 30.20 ± 0.26 |

Proposed wins on every metric. Key findings:

- **TBC** (flat multi-hop, no clustering) dies at ~round 459 from relay overload — nodes near the BS exhaust their energy forwarding packets for the entire field. Validates the clustering premise.
- **FCPA** (clustered, exact jammer geometry) dies at ~round 542 from cooperative relay overhead — relay nodes accumulate forwarding costs from all jammed members they serve. Despite having perfect jammer position knowledge, FCPA cannot match the proposed scheme's EWMA-based temporal memory and adaptive burst size.
- **Proposed** achieves 0 communication blackout rounds; both baselines have significant blackout periods.

---

## File Structure

```
main.m                          single-seed entry point (Proposed + TBC + FCPA)
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
  run_leach.m                   standard LEACH (reference only, not in active comparison)
  run_tbc.m                     TBC baseline (flat multi-hop, threshold suppress)
  run_fcpa.m                    FCPA baseline (IPN-gated election, cooperative relay)

plotting/
  plot_results.m                single-seed plots
  plot_multiseed.m              mean +/- std band plots
  visualize_snapshot.m          2D round snapshot with JR heatmap
  export_figures.m              publication-quality figure export for Overleaf

figures/                        generated by export_figures.m — upload to Overleaf
  fig_combined.pdf              3-panel figure (paper Fig. 2): PDR + Energy + Alive
  fig_pdr.pdf / fig_pdr.png     PDR vs Round (individual)
  fig_energy.pdf / fig_energy.png  Residual Energy vs Round (individual)
  fig_alive.pdf / fig_alive.png    Alive Nodes vs Round (individual)

paper.tex                       complete IEEE-format paper (all sections finalized)
references.bib                  BibTeX entries for baseline references [1][2][3]

testing/
  run_lambda_sensitivity.m      EWMA lambda sweep
  run_phi1_sweep.m              phi1 per-hop penalty sweep
  diag_proposed_zeros.m         proposed zero-PDR round diagnostics
  diag_leach_zeros.m            LEACH zero-PDR round diagnostics
  visualize_tbc_routing.m       TBC routing snapshot: paths, relay load, jammed nodes

docs/
  README.md                     this file
  SIMULATION_LOG.md             permanent record of all simulation runs
  tbc_baseline.md               TBC implementation and adaptation notes
  fcpa_baseline.md              FCPA implementation and adaptation notes
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

## Paper Submission Checklist

Upload these four files to Overleaf:
1. `paper.tex` — complete IEEE conference paper
2. `references.bib` — BibTeX entries
3. `figures/fig_combined.pdf` — 3-panel simulation results figure
4. `system_model.pdf` — system model diagram (Fig. 1)

To regenerate figures (e.g. after a parameter change), re-run:
```matlab
plotting/export_figures.m
```
This takes ~2 minutes for 20 seeds and overwrites `figures/`.

---

## Logging Simulation Results

Every major run should be added to `docs/SIMULATION_LOG.md` with:
- date and who ran it
- code changes since the previous run
- full parameter snapshot
- results table
- takeaways and next steps
