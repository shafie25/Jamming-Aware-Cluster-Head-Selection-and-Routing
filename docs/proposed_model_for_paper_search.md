# Proposed Model: Jamming-Aware Cluster Head Selection and Routing in WSNs under UAV-Based Interference

This document summarizes the current MATLAB implementation so it can be mapped cleanly to prior work and candidate baselines.

---

## Current Problem Setup

- `N = 100` static sensor nodes
- `100 x 100 m` deployment field
- BS at `[50, 50]`
- Initial node energy `E0 = 0.5 J`
- Simulation horizon `T = 1000` rounds
- UAV jammer follows a circular orbit centered at `[50, 50]` with radius `35 m`
- One full jammer orbit every `50` rounds
- Jamming radius `r_j = 20 m`
- Baseline packet success `p_base = 0.95`
- Jamming decay constant `kappa = 10`
- Hard member-to-CH transmission limit `r_tx = 50 m`

Packet success model:

```text
p_i(t) = p_base                               if d_iJ(t) > r_j
p_i(t) = p_base * exp(-kappa * (1 - d_iJ/r_j)) if d_iJ(t) <= r_j
```

At the jammer center, `p` is effectively zero (`p_base * e^{-10} ≈ 0.00004`).

---

## Layer 1: Jamming Risk Estimation + Adaptive Burst Size

Every alive non-CH node sends a burst of `M_eff` packets per round, where:

```text
M_eff(i) = max(M_min, round(M * (1 - JR_i)))     M=10, M_min=2
```

1. Instantaneous PDR is measured from the burst.
2. EWMA smoothing is applied with `lambda = 0.6`:

```text
PDR_ewma_i(t) = lambda * PDR_inst_i(t) + (1 - lambda) * PDR_ewma_i(t-1)
JR_i(t) = 1 - PDR_ewma_i(t)
```

3. Adaptive burst: heavily jammed nodes (high JR) transmit fewer packets, conserving energy without changing the PDR ratio (near-zero recv / reduced sent ≈ same zero). Energy deduction scales by `M_eff/M`.

This is a decentralized link-quality / interference estimate derived only from local packet outcomes.

---

## Layer 1: CH Election

Scheduled CH re-election occurs every `K_elec = 5` rounds.

Each alive node is scored by:

```text
CHScore_i = alpha*(E_i/E0)
          + beta*(|N_i|/N_max)
          - gamma_*JR_i
          - delta*(d_i,BS/d_max)
```

with:
- `alpha = 0.35`
- `beta = 0.20`
- `gamma_ = 0.35`
- `delta = 0.10`

Additional election settings:
- Dynamic CH count: `K = round(0.05 * N_alive)`
- Neighbor counting radius: `r_c = 15 m`
- Spatial exclusion radius: `r_exc = 25 m`

### Emergency CH Re-election

Two triggers (proactive + reactive):
- **Proactive:** a CH died in the previous round — election fires immediately to replace coverage before the gap affects the current round
- **Reactive:** no alive CHs remain at the start of a round

Both cases reuse the standard CHScore election and pay normal control overhead.

Alive non-CH nodes are assigned to the nearest CH within `r_tx`; otherwise they are stranded and their M_eff packets count as lost in the PDR denominator.

---

## Layer 2: Inter-Cluster Routing

CHs route to the BS using Dijkstra on a fully connected CH graph.

Per-edge cost:

```text
C(i,j,t) = phi1 + phi2*E_amp*L*d_ij^2 + phi3*JR_j
```

with:
- `phi1 = 5e-4`
- `phi2 = 1`
- `phi3 = 5e-4`

> **Known result (Runs 015–016):** In this compact geometry (BS at center, r_tx=50m), Dijkstra degenerates to direct CH-to-BS for ~78% of rounds. `run_proposed_direct.m` (same election, no routing) matches or beats Dijkstra on every metric. The full PDR advantage over LEACH comes from the election layer. Even at 200x200m with r_tx=75m, Dijkstra is net-negative (-6.56pp PDR) vs direct due to relay overload at scale. Dijkstra is retained as a framework component.

---

## Energy Model

LEACH-style radio model, scaled by M_eff for member transmissions:

```text
E_tx = (M_eff/M) * (L*E_elec + L*E_amp*d^2)    % member to CH
E_rx = (M_eff/M) * L*E_elec                      % CH receiving from member
E_agg = n_members*L*E_da                          % CH aggregation
E_tx_route = L*E_elec + L*E_amp*d^2              % CH to next hop (full packet)
```

with:
- `E_elec = 50 nJ/bit`
- `E_amp = 100 pJ/bit/m^2`
- `E_da = 5 nJ/bit`
- `L = 4000 bits`
- `M = 10`, `M_min = 2`

---

## Metrics Reported

- PDR over all `T` rounds
- PDR truncated at first node death (`FND-trunc`)
- Zero-PDR round count
- First node death round
- Residual energy at round 300
- Mean hop-count delay

The three-window PDR reporting is deliberate: all-round PDR, FND-truncated PDR, and zero-PDR count capture different aspects of protocol behavior under jamming and late-stage collapse.

---

## Current Implemented Baselines

| Scheme | Description |
|---|---|
| Proposed | JR-aware CH election + JR-aware inter-cluster routing + proactive/reactive emergency CH recovery + adaptive burst size M_eff |
| Standard LEACH | Probabilistic CH election, direct CH-to-BS, no jamming awareness, fixed M=10 burst |

---

## Current Best Comparative Results (Run 017, 20 seeds)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | **704.7 +/- 33.1** | 723.2 +/- 29.3 |
| PDR all rounds (%) | **85.11 +/- 2.02** | 62.46 +/- 0.82 |
| PDR FND-trunc (%) | **88.77 +/- 1.42** | 72.87 +/- 2.70 |
| Zero-PDR rounds | **0.0 +/- 0.0** | 158.4 +/- 13.7 |
| Energy @ round 300 (J) | **33.95 +/- 0.41** | 32.25 +/- 1.02 |

Proposed wins on every metric. +22.6pp all-rounds PDR, +15.9pp FND-truncated PDR, zero communication blackouts vs LEACH's 158 per seed. Adaptive M_eff was the largest single improvement: +13.5pp all-rounds PDR and +99 rounds first-node-death vs Run 014.

---

## Baselines To Implement Next

| Candidate | Design | What it isolates |
|---|---|---|
| EWMA-Detect | LEACH + JR tracking but no action taken | detection alone (no benefit expected — validates that action is needed) |
| Threshold-JR | LEACH + suppress member transmission when JR > 0.5 (fixed M, not adaptive) | threshold suppression vs proportional M_eff |
| Reactive-CH | LEACH + re-elect CH when its JR > 0.5 | reactive CH replacement vs proactive JR-aware election |
| Proposed-Direct | Proposed election + direct CH-to-BS (no Dijkstra) | election layer in isolation (already tested in Run 015) |

All new baselines must use:
- `r_tx = 50m` stranded-node accounting
- 3-window PDR reporting (all-rounds, FND-trunc, zero-PDR count)
- Same energy constants from `core/compute_energy.m`
- Same `compute_packet_success` and `update_jamming_risk` from `core/` and `layer1/`

---

## Literature Search Targets

The most relevant prior-work categories:

1. Jamming-aware clustering in WSNs
2. Interference-aware or anti-jamming routing in clustered WSNs
3. UAV-based jamming threat models for terrestrial sensor networks
4. EWMA-based link-quality / packet-delivery estimation in WSNs
5. Multi-criteria CH scoring methods that combine energy, connectivity, and interference / reliability
6. Adaptive transmission rate / power control under jamming in WSNs
7. Protocols with reactive or emergency CH reselection after CH failure

Specific questions worth searching:
- Prior work that combines both jamming-aware CH election and jamming-aware routing
- WSN papers that use packet-success or PDR-based EWMA as an interference proxy
- WSN papers that include explicit CH failure recovery or emergency reselection
- UAV-jammer models with distance-based or exponential packet-success attenuation
- Papers that reduce transmission rate or power when jamming is detected
- Good ablation baselines: CH-aware only, routing-aware only, recovery-aware only, rate-adaptive only
