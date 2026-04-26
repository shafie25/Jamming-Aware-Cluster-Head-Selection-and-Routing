# TBC Baseline Implementation
# "Threshold-Based Jamming Attack Detection and Cure Mechanism for WSNs"
# Babitha B.S. et al., IEEE MRIE 2025

---

## What the Paper Does

TBC is a **flat-topology** protocol — no clustering, no CH election. Every node
transmits to the BS via multi-hop paths through alive neighbors. The paper's
methodology has five stages (Section III):

### A. Data Collection and Preprocessing
Each node reports packet send/receive counts. The paper normalizes metrics
(Eq. 1: min-max) before computing thresholds. We skip normalization — PDR
is already in [0,1] and normalization has no effect on the binary detection
outcome.

### B. Threshold Value Calculation (Eq. 2)
```
T_i = Packets_Received_i / Packets_Sent_i
```
Instantaneous PDR per node, computed each round:
```matlab
T_i(i) = sum(rand(M, 1) <= p(i)) / M
```
`p(i)` comes from `compute_packet_success` — our existing channel model.
**No EWMA smoothing** — TBC uses instantaneous PDR, which is a deliberate
contrast with the proposed scheme's EWMA-based JR.

### C. Malicious Node Detection (Eq. 3)
```
jammed_i = 1   if T_i < T_threshold (0.5)
jammed_i = 0   otherwise
```
`T_threshold = 0.5` — the paper gives no fixed value, describing a dynamic
baseline threshold. We use 0.5 as a reasonable midpoint for detecting
significant channel degradation.

### D. Key Management (Eqs. 4–5) — Omitted
TKM is specific to the paper's replay-attack threat model. Not applicable
under continuous UAV jamming. Cited explicitly as an omission.

### E. Path Reconfiguration (Eq. 6)
```
P_new = P - P_malicious
```
Jammed nodes are removed from the routing graph entirely. No traffic
originates from or routes through them that round. They resume when their
instantaneous PDR recovers above 0.5 in a future round.

---

## Implementation: `schemes/run_tbc.m`

### Topology
Flat multi-hop — no clustering. Every alive, non-jammed node originates
M packet trials toward BS each round via the Dijkstra-computed next-hop chain.

### Per-Round Algorithm

**Step 1 — Channel quality**
```matlab
p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j)
```
`p(i)` = probability a single packet from node i survives this round.
Drops exponentially to near-zero when the UAV jammer is within `r_j = 20m`.

**Step 2 — Jamming detection**
```matlab
T_i(i) = sum(rand(M, 1) <= p(i)) / M    % instantaneous PDR from M-packet probe
jammed  = alive & (T_i < 0.5)
```
Each alive node independently probes its channel using M Bernoulli trials.
Detection is purely local and instantaneous — no memory of prior rounds.

**Step 3 — Build routing graph and run Dijkstra**
```matlab
routable = alive & ~jammed
```
Only routable nodes participate in the graph. Dijkstra runs **from BS outward**,
expanding the shortest-path tree toward all sensor nodes in one pass.
`next_hop(i)` stores the next node toward BS for every node.

Edge weights are energy-penalised distances:
```
cost(u → v) = dist(u, v) × (E0 / max(energy(u), 1e-6))
```
A relay node `u` at half energy appears twice as far away, so Dijkstra
routes around depleted nodes in favor of fresher ones. This distributes
relay burden across the field as the network ages.

**Step 4 — Packet forwarding**

For every alive node `i`:
- Add M to `total_sent` (PDR denominator — all alive nodes had data to send)
- If jammed → skip (packets lost, no energy cost)
- If `D(i) == inf` (isolated, no path exists) → skip (packets lost)
- Otherwise trace the path hop by hop:

At each hop from `cur` to `next_hop(cur)`:
1. Charge TX energy at `cur`:
   ```
   (recv_packets / M) × (L·E_elec + L·E_amp·d²)
   ```
2. Charge RX energy at the next node (unless next is BS):
   ```
   (recv_packets / M) × L·E_elec
   ```
3. Apply channel loss — surviving packets after this hop:
   ```matlab
   recv_packets = sum(rand(recv_packets, 1) <= p(cur))
   ```
4. Advance `cur = next_hop(cur)`, increment `hops`.

When `cur == BS_idx`, add `recv_packets` to `total_recv`.

**Energy convention:** `recv_packets/M` is the key scaling factor. When
all M packet trials survive (`recv_packets == M`), the fraction is 1 and the node pays
exactly `L·E_elec + L·E_amp·d²` — identical to what `compute_energy('tx')`
charges in `run_proposed` and `run_leach`, where L = 4000 bits is the total
round payload represented by M packet trials, not a per-trial size. As trials
are lost along the path, the relay pays proportionally less because it is
forwarding less represented data.

All energy deltas are accumulated in a temporary array and applied at once
after all nodes are processed. This prevents a node dying mid-round from
affecting traffic it already agreed to relay.

**Step 5 — Metrics and node death**
```matlab
PDR(t)    = total_recv / total_sent
energy(t) = sum(energy(alive))
delay(t)  = mean hops per successfully routed source
```
Nodes with `energy ≤ 0` are killed. First death round recorded as `t_death`.

---

## What TBC Does NOT Have (vs. Proposed)

| Feature | Proposed | TBC |
|---|---|---|
| Cluster heads | Yes — JR-aware CHScore election | No — flat topology |
| Jamming memory | EWMA JR smoothed over rounds | Instantaneous only |
| Adaptive burst | M_eff reduced proportional to JR | Always M or full suppression |
| Data aggregation | CH aggregates before routing | Every node routes individually |
| Channel loss model | Counted once (member → CH) | Counted at every hop |

---

## What This Comparison Isolates

| Scheme | Topology | Election | Jamming Response |
|---|---|---|---|
| LEACH | Clustered | Random prob. | None |
| TBC | Flat multi-hop | None | Binary suppress when PDR < 0.5 |
| Proposed | Clustered | JR-aware CHScore | Adaptive M_eff + JR-aware election |

TBC tests: *does flat topology with threshold suppression outperform
clustered protocols under UAV jamming?*

---

## Citation Summary for Paper

- Architecture and stages: Section III, Figure 1
- Threshold computation: Eq. (2), Section III-B
- Detection rule: Eq. (3), Section III-C
- Path reconfiguration: Eq. (6), Section III-E
- TKM omission: Section III-D — time-based key management is replay-attack
  specific; not applicable under continuous UAV jamming
- Performance context: Table II/III — TBC achieves 95.1% PDR in their NS-2
  replay-attack scenario; our scenario (mobile UAV jammer) is a harder and
  structurally different threat model
