# CLAUDE.md - Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes in a `100 x 100 m` field, BS at `[50,50]`, and a UAV jammer moving on a circular orbit. The proposed scheme uses jamming risk (JR) in CH election and inter-cluster routing, compared against TBC and FCPA baselines.

---

## Current State (as of 2026-04-19, Paper Complete)

### Implemented
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing + proactive emergency CH re-election + adaptive burst size (M_eff)
- `schemes/run_leach.m` — standard LEACH (kept as reference; removed from active comparison in Run 020)
- `schemes/run_tbc.m` — TBC baseline: flat multi-hop topology, instantaneous PDR detection, energy-aware Dijkstra, threshold suppression (Run 019 energy fix)
- `schemes/run_fcpa.m` — FCPA baseline: IPN-gated CH election + cooperative relay for jammed members (Run 020, adapted from López-Vilos et al. Sensors 2023)
- `run_multiseed.m` — main evaluation: seeds 1:20, Proposed + TBC + FCPA, 2-window PDR + energy@r300
- `plotting/visualize_snapshot.m` — 2D network map with JR heatmap and routing paths
- `plotting/export_figures.m` — runs full 20-seed sim and exports publication-quality PDFs/PNGs to `figures/`
- `testing/visualize_tbc_routing.m` — TBC routing snapshot: paths, relay load, jammed/isolated nodes
- `testing/` — all sensitivity sweeps, routing experiments, and diagnostics (run from project root)
- `paper.tex` — complete IEEE-format paper (all sections written and finalized)
- `references.bib` — BibTeX entries for all three cited baselines
- `figures/` — exported figure PDFs and PNGs ready for Overleaf upload

### Entry Points
- `main.m` — single seed quick check (Proposed + TBC + FCPA)
- `run_multiseed.m` — canonical comparative evaluation
- `plotting/export_figures.m` — regenerate all paper figures (runs 20-seed sim internally)

### Active Model
- `kappa = 10`
- `K_elec = 5`
- `lambda = 0.6`
- `phi1 = 5e-4` (updated in Run 014)
- `p_CH = 0.05`
- `r_tx = 50m`
- Emergency CH re-election fires when a CH died last round OR no alive CHs remain (proactive)
- Adaptive burst size: `M_eff(i) = max(M_min, round(M*(1-JR(i))))`, `M_min=2` — jammed nodes conserve energy by reducing transmissions proportionally to their EWMA jamming risk (Run 017)

### Current Best Comparative Results (Run 021, 20 seeds)

| Metric | Proposed | TBC | FCPA |
|---|---|---|---|
| First node death (round) | **704.7 +/- 33.1** | 459.1 +/- 41.3 | 542.1 +/- 19.2 |
| PDR all rounds (%) | **85.11 +/- 2.02** | 53.20 +/- 4.41 | 63.35 +/- 0.89 |
| PDR FND-trunc (%) | **88.77 +/- 1.42** | 82.42 +/- 0.58 | 75.07 +/- 1.44 |
| Energy @ round 300 (J) | **33.95 +/- 0.41** | 26.80 +/- 1.12 | 30.20 +/- 0.26 |

Proposed wins on every metric — including against FCPA which has exact jammer position each round. TBC dies at ~round 459 from relay overload (flat topology). FCPA dies at ~round 542 from cooperative relay energy overhead. The proposed scheme's EWMA JR + adaptive M_eff + JR-aware election outperforms both.

---

## Important Gotchas

**`lambda = 0.6` is the best tested value.**
Sweep over {0.6, 0.7, 0.8}: no PDR gain from higher lambda, worse blackouts from noise amplification with M=10 bursts.

**Adaptive M_eff uses EWMA JR (not instantaneous p).**
`M_eff(i) = max(2, round(10*(1-JR(i))))`. With lambda=0.6, recovery after jammer leaves takes ~3 rounds — negligible vs 50-round orbit period. All three loop-replaying scripts (visualize_snapshot, diag_proposed_zeros, diag_scale) are kept in sync with run_proposed.m.

**`p_CH` flows from `core/config.m` through function signatures.**
`elect_ch_proposed.m` no longer hardcodes p_CH. Any sweep is real.

**`r_c` is not a radio limit.**
`r_c=15m` is only used in the CHScore beta term for neighbor counting. Hard radio limit is `r_tx=50m`.

**PDR is reported in two windows.**
All-rounds PDR and FND-truncated PDR. Zero-PDR round count was removed from `run_multiseed.m` output in Run 021 (proposed always has 0; it added noise without insight for the other two schemes).

**`gamma_` not `gamma`.**
`gamma` conflicts with a MATLAB builtin. The CHScore jamming-risk weight is `gamma_` everywhere.

**Exact jammer position does not beat EWMA JR (Run 020).**
FCPA has omniscient jammer geometry; proposed only estimates JR from experienced packet loss. Proposed still wins by +21.76pp all-rounds PDR and +162.6 rounds FND. EWMA temporal memory + adaptive M_eff outweighs the information advantage of exact geometry.

**TBC energy convention: recv_packets/M not recv_packets (Run 019).**
In `run_tbc.m`, per-hop TX/RX energy scales by `recv_packets/M` — a fraction ≤ 1. L=4000 bits is the total round payload represented by the M packet trials, same as all other schemes. Using raw `recv_packets` would charge 10× too much per hop.

---

## Repo Structure

```text
main.m
run_multiseed.m
CLAUDE.md
core/
  config.m
  init_network.m
  uav_trajectory.m
  compute_packet_success.m
  compute_energy.m
layer1/
  update_jamming_risk.m
  elect_ch_proposed.m
layer2/
  route_dijkstra.m
schemes/
  run_proposed.m
  run_leach.m                  (reference only — not in active comparison)
  run_tbc.m
  run_fcpa.m
plotting/
  plot_results.m
  plot_multiseed.m
  visualize_snapshot.m
  export_figures.m           (NEW) publication figure export for Overleaf
figures/
  fig_combined.pdf           3-panel PDR + Energy + Alive (paper Fig. 2)
  fig_pdr.pdf
  fig_energy.pdf
  fig_alive.pdf
  *.png                      300 DPI PNG copies of each figure
paper.tex                    complete IEEE-format paper
references.bib               BibTeX entries for [1][2][3]
testing/
  run_lambda_sensitivity.m
  run_phi1_sweep.m
  diag_proposed_zeros.m
  diag_leach_zeros.m
  diag_scale.m
  visualize_tbc_routing.m
docs/
  README.md
  SIMULATION_LOG.md
  fcpa_baseline.md
  tbc_baseline.md
  proposed_model_for_paper_search.md
reference/
  LEACH.m
```

---

## Project Status: COMPLETE (2026-04-19)

The simulation, evaluation, and paper write-up are finished. Everything needed for submission is in the repo.

### Done
1. ~~Paper write-up~~ — `paper.tex` complete. All sections written: Abstract, Introduction, Related Work, Contributions, System Model, Proposed Scheme (including M_eff subsection), Simulation Results (Table I parameters, Table II results, Figure, Discussion), Conclusion. `references.bib` created.
2. ~~Figure export~~ — `plotting/export_figures.m` written and run. `figures/fig_combined.pdf` generated and ready for Overleaf.
3. ~~FCPA baseline~~ — done in Run 020. IPN-gated election + cooperative relay.
4. ~~TBC energy fix~~ — done in Run 019. recv_packets/M scaling corrected.
5. ~~TBC baseline~~ — done in Run 018/019. Flat multi-hop, threshold suppression.
6. ~~Adaptive burst size~~ — done in Run 017. M_eff = max(2, round(M*(1-JR))).
7. ~~Scaled-up geometry test~~ — done in Run 016. Dijkstra confirmed net-negative at 200x200m.
8. ~~phi1 sweep~~ — done in Run 014. phi1=5e-4 is canonical.
9. ~~Zero-PDR rounds~~ — solved (0.0 +/- 0.0 for proposed).

### To Submit
Upload to Overleaf: `paper.tex`, `references.bib`, `figures/fig_combined.pdf`, `system_model.pdf`

---

## How To Add a New Baseline

1. Create `schemes/run_<name>.m` returning a struct with `PDR`, `energy`, `delay`, `alive`, `t_death`, `label`.
2. Use the same `r_tx` stranded-node accounting as the current schemes.
3. Use `compute_energy`, `compute_packet_success`, `update_jamming_risk` from `core/` and `layer1/`.
4. Use fixed M=10 packet-trial count (no adaptive M_eff) for fair comparison vs proposed.
5. Wire into `main.m` and `run_multiseed.m`.
6. Log results in `docs/SIMULATION_LOG.md`.
