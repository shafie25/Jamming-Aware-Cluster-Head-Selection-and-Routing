# CLAUDE.md - Context for Claude Code Sessions

## What This Project Is

MATLAB simulation for a graduate wireless networks course project:
**"Jamming-Aware Cluster Head Selection and Routing in Wireless Sensor Networks under UAV-Based Interference"**

100 sensor nodes in a `100 x 100 m` field, BS at `[50,50]`, and a UAV jammer moving on a circular orbit. The proposed scheme uses jamming risk (JR) in CH election and inter-cluster routing, compared against TBC and FCPA baselines.

---

## Current State (as of 2026-04-28, Run 028)

### Implemented
- `schemes/run_proposed.m` — proposed scheme: JR-aware CHScore election + Dijkstra routing + proactive emergency CH re-election + adaptive burst size (M_eff) + post-FND stranded node relay (Run 028)
- `schemes/run_leach.m` — standard LEACH (kept as reference; removed from active comparison in Run 020)
- `schemes/run_tbc.m` — TBC baseline: flat multi-hop topology, instantaneous PDR detection, energy-aware Dijkstra, threshold suppression (Run 019 energy fix)
- `schemes/run_fcpa.m` — FCPA baseline: K_elec-gated election (Run 024), IPN-gated CH election + cooperative relay for jammed members
- `run_multiseed.m` — main evaluation: seeds 1:100, Proposed + TBC + FCPA; metrics: FND, HND, PDR all rounds, PDR FND-trunc, PDR@r300, Energy@r300, total delivered packets (Run 025)
- `plotting/plot_multiseed.m` — 4-panel figure: PDR, Energy, Cumulative Delivered Packets, Alive Nodes (Run 025: delay panel replaced)
- `plotting/visualize_snapshot.m` — 2D network map with JR heatmap and routing paths
- `plotting/export_figures.m` — runs 20-seed sim internally and exports publication-quality PDFs/PNGs to `figures/` (uses its own 20-seed loop, separate from run_multiseed.m)
- `testing/visualize_tbc_routing.m` — TBC routing snapshot: paths, relay load, jammed/isolated nodes
- `testing/` — all sensitivity sweeps, routing experiments, and diagnostics (run from project root)
- `paper.tex` — IEEE-format paper (needs update for Run 028 numbers and relay mechanism)
- `references.bib` — BibTeX entries for all three cited baselines
- `figures/` — exported figure PDFs and PNGs (need regeneration after paper update)

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
- Adaptive burst size: `M_eff(i) = round(M*(1-JR(i)))`, floor=0 — jammed nodes conserve energy by reducing transmissions proportionally to their EWMA jamming risk (Run 017, floor lowered in Run 022)
- Sleep timer: nodes at M_eff=0 sleep for `N_sleep=5` rounds (EWMA frozen, JR reset to 0 on wakeup) — covers the jammer arc tail so nodes don't over-penalise post-jam JR
- PDR measured end-to-end at the BS: packets must survive every routing hop via Dijkstra, not just reach the CH (fixed in Run 022)
- Dijkstra enforces `r_tx=50m` radio range — edges beyond range pruned from cost matrix (fixed in Run 022)
- Post-FND stranded node relay (Run 028): after first node death, isolated nodes (no CH within r_tx) relay via highest-energy non-CH member within r_tx. Only fires post-FND; JR < 0.5 filter; one relay node per stranded node per round; only stranded→relay hop energy charged (relay→CH amortised).

### Current Best Comparative Results (Run 028, 100 seeds)

| Metric | Proposed | TBC | FCPA |
|---|---|---|---|
| FND (rnd) | **702.2 +/- 34.9** | 469.2 +/- 49.0 | 571.9 +/- 42.4 |
| HND (rnd) | **912.7 +/- 20.5** | 634.0 +/- 50.6 | 828.0 +/- 11.2 |
| PDR all rounds (%) | **79.44 +/- 1.70** | 52.54 +/- 4.16 | 58.25 +/- 2.88 |
| PDR FND-trunc (%) | 80.96 +/- 1.48 | **82.49 +/- 0.56** | 59.87 +/- 3.13 |
| PDR@r300 (%) | **84.41 +/- 4.54** | 83.07 +/- 3.28 | 50.53 +/- 8.82 |
| Energy@r300 (J) | **34.17 +/- 0.37** | 26.68 +/- 1.23 | 31.95 +/- 0.24 |
| Total del. pkts (k) | **731.7 +/- 17.8** | 511.2 +/- 37.4 | 495.7 +/- 25.9 |

Proposed wins on FND (+233 rounds vs TBC, +130 rounds vs FCPA), HND (+279 vs TBC, +85 vs FCPA), all-rounds PDR (+26.90pp vs TBC, +21.19pp vs FCPA), and total delivered packets (+220k vs TBC, +236k vs FCPA). TBC's slightly higher FND-trunc PDR (82.49% vs 80.96%) is a window artefact from its shorter lifetime. FCPA's PDR@r300 (50.53%) confirms its cooperative relay has a structural per-round PDR deficit even when the network is fully healthy — not just a lifetime-window effect.

---

## Important Gotchas

**`lambda = 0.6` is the best tested value.**
Sweep over {0.6, 0.7, 0.8}: no PDR gain from higher lambda, worse blackouts from noise amplification with M=10 bursts.

**Adaptive M_eff uses EWMA JR (not instantaneous p).**
`M_eff(i) = round(10*(1-JR(i)))`, floor=0. Nodes at M_eff=0 sleep for N_sleep=5 rounds (EWMA frozen, JR reset on wakeup). With lambda=0.6, JR recovery after jammer leaves takes ~3 rounds — negligible vs 50-round orbit period. All three loop-replaying scripts (visualize_snapshot, diag_proposed_zeros, diag_scale) must be kept in sync with run_proposed.m if the sleep timer logic changes.

**`p_CH` flows from `core/config.m` through function signatures.**
`elect_ch_proposed.m` no longer hardcodes p_CH. Any sweep is real.

**`r_c` is not a radio limit.**
`r_c=15m` is only used in the CHScore beta term for neighbor counting. Hard radio limit is `r_tx=50m`.

**PDR is reported in two windows.**
All-rounds PDR and FND-truncated PDR. Zero-PDR round count was removed from `run_multiseed.m` output in Run 021 (proposed always has 0; it added noise without insight for the other two schemes).

**`gamma_` not `gamma`.**
`gamma` conflicts with a MATLAB builtin. The CHScore jamming-risk weight is `gamma_` everywhere.

**Exact jammer position does not beat EWMA JR (Run 020/024/028).**
FCPA has omniscient jammer geometry; proposed only estimates JR from experienced packet loss. Proposed wins by +21.19pp all-rounds PDR and +130 rounds FND (Run 028, 100-seed numbers). EWMA temporal memory + adaptive M_eff + sleep timer + post-FND relay outweighs the information advantage of exact geometry even against a fairer FCPA baseline.

**Post-FND relay must stay gated on `~isnan(t_death)`.**
Runs 026–027 showed that relay active in healthy rounds drains relay nodes faster than it delivers packets, causing FND to drop below FCPA (551/561 vs 572). Gate is load-bearing — do not remove it. All three efficiency constraints (JR < 0.5 filter, one relay per relay node per round, amortised relay→CH energy) must be retained together.

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

## Project Status: IN PROGRESS (2026-04-28, Run 028)

The simulation, evaluation, and paper write-up are finished. Everything needed for submission is in the repo. Run 022 corrected baseline modelling (end-to-end PDR, FCPA overhead, r_tx in Dijkstra) and added the sleep timer; paper numbers from Run 021 still hold structurally.

### Done
1. ~~Paper write-up~~ — `paper.tex` complete. All sections written: Abstract, Introduction, Related Work, Contributions, System Model, Proposed Scheme (including M_eff subsection), Simulation Results (Table I parameters, Table II results, Figure, Discussion), Conclusion. `references.bib` created.
2. ~~Figure export~~ — `plotting/export_figures.m` written and run. `figures/fig_combined.pdf` generated and ready for Overleaf.
3. ~~Baseline correctness review~~ — done in Run 022. FCPA overhead, end-to-end PDR, r_tx in Dijkstra, FCPA range gate, sleep timer.
4. ~~FCPA baseline~~ — done in Run 020. IPN-gated election + cooperative relay.
5. ~~TBC energy fix~~ — done in Run 019. recv_packets/M scaling corrected.
6. ~~TBC baseline~~ — done in Run 018/019. Flat multi-hop, threshold suppression.
7. ~~Adaptive burst size~~ — done in Run 017. M_eff = round(M*(1-JR)), floor lowered to 0 in Run 022 with sleep timer.
8. ~~Scaled-up geometry test~~ — done in Run 016. Dijkstra confirmed net-negative at 200x200m.
9. ~~phi1 sweep~~ — done in Run 014. phi1=5e-4 is canonical.
10. ~~Zero-PDR rounds~~ — solved (0.0 +/- 0.0 for proposed).

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
