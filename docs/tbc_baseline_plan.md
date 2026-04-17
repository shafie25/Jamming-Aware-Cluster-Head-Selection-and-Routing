# TBC Baseline Implementation Plan
# "Threshold-Based Jamming Attack Detection and Cure Mechanism for WSNs"
# Babitha B.S. et al., IEEE MRIE 2025

---

## What the Paper Actually Does

TBC is a **flat-topology** protocol. There is no clustering. Every node
transmits directly to the Base Station. The paper's methodology has five
stages (Section III):

### A. Data Collection and Preprocessing
> "Data streams are collected from any and every node containing sensors
> that is linked to the internet at the very beginning of the architecture"

Each node reports its packet send/receive counts to the BS. The BS
normalizes the metrics (Eq. 1: min-max normalization) before computing
thresholds. In our implementation we skip normalization — we compute PDR
directly, which is already in [0,1].

### B. Threshold Value Calculation (Eq. 2)
```
T_i = Packets_Received_i / Packets_Sent_i
```
> "A threshold quantity is established by taking the average of the entire
> number of packets which are sent through each node."

This is PDR per node, computed at the BS from reported counts. In our
framework we compute this as the instantaneous PDR each round:
```
T_i(t) = sum(rand(M,1) <= p_i(t)) / M
```
where `p_i(t)` is our existing packet success model. We use **instantaneous**
PDR (not EWMA) to stay faithful to the paper — TBC does not mention any
smoothing filter.

### C. Malicious Node Detection (Eq. 3)
```
M_i = 1   if T_i < T_threshold
M_i = 0   otherwise
```
> "A node with malicious intent is detected as one that exceeds its
> permitted parameters and is isolated from the remaining components
> of the network."

Note: the paper flags nodes as malicious when PDR drops below the
threshold — in the original context this means the node is being
jammed/disrupted. We apply the same logic: `T_i < T_threshold` → node
is jammed → flag it.

**Threshold value:** The paper does not specify a fixed threshold number.
It describes a per-node dynamic threshold derived from baseline traffic.
For our simulation we use **T_threshold = 0.5** (PDR < 50% → jammed).
This is consistent with the paper's description of detecting significant
degradation from normal operation.

### D. Key Management for Secure Communication (Eqs. 4–5)
> "A method of controlling keys that is based on a passing of time is
> implemented in order to ensure that nodes communicate in a trustworthy
> manner with one another."

This step (TKM — time-based key management) is specific to the paper's
replay attack threat model. It is **not applicable** to our UAV jamming
scenario and is omitted. We cite this omission explicitly in the paper.

### E. Path Reconfiguration (Eq. 6)
```
P_new = P - P_malicious
```
> "After detecting malicious nodes, the infrastructure will reorganize the
> communication channels so as to avoid those nodes which were penetrated
> recently."
>
> "Upon identifying the attack, the program identifies the jammer node,
> shares data pertaining to it with surrounding nodes, and alters all
> pathways from it."

In the paper's multi-hop flat network, path reconfiguration means rerouting
around flagged nodes. In our flat direct-to-BS topology, there is no
alternate route — every node's only path IS the direct path to BS.
Therefore path reconfiguration = **suppress the flagged node's transmission
that round**. The node waits until its PDR recovers above the threshold in
a future round before resuming.

This is the faithful adaptation: the paper's "remove flagged node from
path" becomes "flagged node does not transmit" when the only path is
direct-to-BS.

---

## Implementation Design for `schemes/run_tbc.m`

### Topology
- Flat — **no clustering, no CH election**
- Every alive node attempts to send M packets directly to BS each round
- Unless flagged as jammed that round

### Per-Round Logic
```
1. Compute instantaneous PDR per node:
      p_i = compute_packet_success(...)     % existing function
      T_i = (1/M) * sum(rand(M,1) <= p_i)  % instantaneous PDR from M-packet burst

2. Detect jammed nodes:
      jammed_i = (T_i < T_threshold)        % binary flag, Eq. 3

3. Non-jammed alive nodes transmit:
      energy(i) -= compute_energy('tx', L, E_elec, E_amp, E_da, d_BS_i, 0)
      [BS receives packet, counts toward PDR numerator]

4. Jammed nodes suppress — no transmission, no energy deducted for tx
      [their M packets count as lost in PDR denominator]

5. PDR this round:
      recv = sum of successful packets from non-jammed nodes
      sent = M * n_alive                    % all alive nodes had data to send
      PDR(t) = recv / sent

6. Node death: energy(i) <= 0 → alive(i) = false
```

### Energy Model
Direct-to-BS only — no aggregation, no relay:
```
E_tx_i = L*E_elec + L*E_amp * d_BS_i^2    % per non-jammed node per round
```
Same constants as all other schemes: E_elec=50nJ/bit, E_amp=100pJ/bit/m^2, L=4000 bits.

No receive energy at nodes (BS is infrastructure, not energy-constrained).
No aggregation energy (no CH role).

### Output Struct
Same fields as run_proposed.m and run_leach.m:
```matlab
results.PDR    = PDR;        % vector length T
results.energy = energy_total; % total residual energy per round
results.delay  = delay;      % avg hops — always 1 (direct to BS)
results.alive  = alive_count;
results.t_death = first_node_death_round;
results.label  = 'TBC (flat, threshold-suppress)';
```

---

## Final Implementation Decisions (post-exploration)

1. **Detection: instantaneous PDR (Eq. 2), not EWMA.**
   EWMA was tried (lambda=0.6, same as proposed scheme) but had negligible
   effect on results — energy-aware routing was the dominant factor. Reverted
   to pure instantaneous PDR to stay faithful to the paper's Eq. 2-3 with
   no added assumptions.

2. **Routing: multi-hop Dijkstra with energy-aware edge weights.**
   Initial implementation used direct-to-BS (simplest flat topology).
   Updated to multi-hop after recognizing the paper's path reconfiguration
   (Eq. 6) only makes sense when nodes route through each other. Edge weight:
   `dist(u,v) × (E0 / max(energy(u), 1e-6))` — penalises low-energy relay
   nodes so Dijkstra distributes relay burden as nodes deplete.
   This adaptation goes beyond the paper spec (which gives no routing
   formula) but is necessary to prevent trivial collapse from relay overload.

3. **Threshold value = 0.5** (unchanged). Paper gives no formula.

4. **Delay metric: average hops per routed source per round.**

---

## What This Comparison Isolates

| Scheme   | Topology      | Election        | Jamming response                      |
|----------|---------------|-----------------|---------------------------------------|
| LEACH    | Clustered     | Random prob.    | None                                  |
| TBC      | **Flat multi-hop** | None       | Binary suppress when PDR < 0.5        |
| Proposed | Clustered     | JR-aware score  | Adaptive M_eff + JR-aware CH election |

TBC tests the hypothesis: *does flat topology with threshold suppression
outperform clustered protocols under UAV jamming?*

---

## Actual Results (Run 018, 20 seeds)

| Metric | Proposed | LEACH | TBC |
|---|---|---|---|
| First node death (round) | **704.7 ± 33.1** | 723.2 ± 29.3 | 46.6 ± 4.2 |
| PDR all rounds (%) | **85.11 ± 2.02** | 62.46 ± 0.82 | 5.20 ± 0.35 |
| PDR FND-trunc (%) | **88.77 ± 1.42** | 72.87 ± 2.70 | 81.42 ± 0.53 |
| Zero-PDR rounds | **0.0 ± 0.0** | 158.4 ± 13.7 | 932.5 ± 3.4 |
| Energy @ round 300 (J) | **33.95 ± 0.41** | 32.25 ± 1.02 | 0.88 ± 0.55 |

**Interpretation:** TBC's FND-truncated PDR (81.42%) is competitive —
threshold suppression works correctly during the network's ~47-round life.
The catastrophic all-rounds PDR (5.2%) and 932 zero-PDR rounds are entirely
due to relay overload killing the flat network before round 50. This is not
a detection failure; it is a structural energy failure that clustering (LEACH,
Proposed) avoids by design. The comparison validates the clustering premise.

---

## Citation Summary for Paper

- Architecture and stages: Section III, Figure 1
- Threshold computation: Eq. (2), Section III-B
- Detection rule: Eq. (3), Section III-C
- Path reconfiguration: Eq. (6), Section III-E
- TKM omission justification: Section III-D — "time-dependent administration
  of keys" is specific to replay attack mitigation; not applicable under
  continuous UAV jamming
- Performance context: Table II/III — TBC achieves 95.1% PDR in their NS-2
  replay attack scenario; our scenario (mobile UAV jammer) is harder and
  different in threat model
