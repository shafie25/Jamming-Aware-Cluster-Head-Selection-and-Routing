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

At the jammer center, `p` is effectively zero.

---

## Layer 1: Jamming Risk Estimation

Every alive non-CH node sends `M = 10` packets per round.

1. Instantaneous PDR is measured from the burst.
2. EWMA smoothing is applied with `lambda = 0.6`.
3. Jamming risk is computed as:

```text
JR_i(t) = 1 - PDR_ewma_i(t)
```

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

The current implementation includes a recovery rule in `run_proposed.m`:
- If a round starts with **no alive CHs**, the protocol triggers an immediate emergency CH re-election.
- The emergency step reuses the standard CH election logic and pays the normal control overhead.
- This is intended only to remove the artificial blackout window created when all CHs die between scheduled elections.

Alive non-CH nodes are assigned to the nearest CH within `r_tx`; otherwise they are stranded and their packets count as lost in the PDR denominator.

---

## Layer 2: Inter-Cluster Routing

CHs route to the BS using Dijkstra on a fully connected CH graph.

Per-edge cost:

```text
C(i,j,t) = phi1 + phi2*E_amp*L*d_ij^2 + phi3*JR_j
```

with:
- `phi1 = 1e-4`
- `phi2 = 1`
- `phi3 = 5e-4`

Interpretation:
- penalize extra hops (`phi1`)
- penalize long links (`d^2` energy term)
- avoid relays that are currently jammed (`JR_j` term)

---

## Energy Model

LEACH-style radio model:

```text
E_tx = L*E_elec + L*E_amp*d^2
E_rx = L*E_elec
E_agg = n_members*L*E_da
```

with:
- `E_elec = 50 nJ/bit`
- `E_amp = 100 pJ/bit/m^2`
- `E_da = 5 nJ/bit`
- `L = 4000 bits`

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
| Proposed | JR-aware CH election + JR-aware inter-cluster routing + emergency CH recovery when no CH remains |
| Standard LEACH | Probabilistic CH election, direct CH-to-BS, no jamming awareness |

---

## Current Best Comparative Results

Current default comparison uses `run_multiseed.m` with seeds `1:20`.

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 603.0 ± 35.1 | **726.6 ± 30.4** |
| PDR all rounds (%) | **71.24 ± 1.58** | 62.20 ± 0.85 |
| PDR FND-trunc (%) | **82.03 ± 1.28** | 72.81 ± 2.91 |
| Zero-PDR rounds | **1.2 ± 5.6** | 164.5 ± 15.7 |
| Energy @ round 300 (J) | 31.61 ± 0.43 | **32.39 ± 1.25** |

Headline interpretation:
- The proposed scheme clearly improves delivery and strongly suppresses blackout rounds.
- LEACH still lasts longer and retains slightly more energy because relay load is concentrated in the proposed multi-hop design.
- Remaining proposed zero-PDR rounds occur only in extreme late-stage collapse.

---

## Literature Search Targets

The most relevant prior-work categories now are:

1. Jamming-aware clustering in WSNs
2. Interference-aware or anti-jamming routing in clustered WSNs
3. UAV-based jamming threat models for terrestrial sensor networks
4. EWMA-based link-quality / packet-delivery estimation in WSNs
5. Multi-criteria CH scoring methods that combine energy, connectivity, and interference / reliability
6. Protocols with reactive or emergency CH reselection after CH failure

Specific questions worth searching:
- Prior work that combines both jamming-aware CH election and jamming-aware routing
- WSN papers that use packet-success or PDR-based EWMA as an interference proxy
- WSN papers that include explicit CH failure recovery or emergency reselection
- UAV-jammer models with distance-based or exponential packet-success attenuation
- Good ablation baselines: CH-aware only, routing-aware only, recovery-aware only
