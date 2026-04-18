# FCPA Baseline Implementation
# "Clustering-Based Energy-Efficient Self-Healing Strategy for WSNs under Jamming Attacks"
# López-Vilos et al., Sensors 2023

---

## What the Paper Does

FCPA is a **clustered WSN protocol** that adapts CH selection and routing in response to jamming by exploiting knowledge of the jammer's geometric position. The paper's core contributions are:

1. **IPN-aware CH election** — nodes near the jammer are penalised in CH candidacy using an Interference-Plus-Noise (IPN) metric computed from estimated jammer location.
2. **Cooperative relay for isolated/jammed members** — cluster members that cannot reach their CH due to jamming identify relay nodes and use load-balanced 2-hop paths (member → relay → CH).
3. **Load-balanced relay selection** — relay weight is `w_ij = (1/e_coop_ij) / Σ_k(1/e_coop_ik)`, spreading relay traffic across low-cost candidates rather than always overloading the nearest node.

The paper evaluates against a fixed-power LEACH baseline in an NS-2 simulation with a stationary jammer. Our adaptation applies these mechanisms to a mobile UAV jammer on a circular orbit.

---

## How FCPA Was Adapted to Our System Model

### A. IPN → Jammer Distance Gate

The paper computes IPN as a function of received interference power at each node, which depends on jammer location. In our model, the UAV jammer position `(J_x(t), J_y(t))` is known each round (it is modelled as a system-level observable). We replace the paper's IPN SNR computation with a direct geometric gate:

```
d_jammer(i) = sqrt((x(i) - J_x(t))^2 + (y(i) - J_y(t))^2)
ipn_jammed(i) = d_jammer(i) < r_j     % r_j = 20m jamming radius
```

This is equivalent to the paper's IPN threshold — nodes inside the jamming radius experience near-zero packet success probability (`p(i) ≈ 0`) and are treated as IPN-jammed. The gate is **binary and instantaneous** — it resets every round with the UAV's new position, with no memory of prior rounds.

### B. K-Medoids → LEACH Epoch Mechanism + IPN Gate

The paper uses K-medoids clustering where cluster centers are chosen to minimise intra-cluster distance AND have low IPN. We do not implement K-medoids (it requires an iterative assignment step incompatible with the round-by-round LEACH energy budget). Instead:

- CH candidacy uses the LEACH epoch threshold `T_thresh = p_CH / (1 - p_CH × mod(t-1, 1/p_CH))` to control the expected CH fraction over time.
- The IPN gate is applied as an **eligibility filter**: nodes with `ipn_jammed = true` cannot be CHs this round, regardless of their LEACH threshold draw.
- Fallback: if all eligible nodes are jammed (rare, near-total jamming), the fallback elects from any alive non-jammed node, then any alive node as a last resort.

This preserves the paper's core intent — CHs are elected from geometrically safe regions — while keeping the energy budget and epoch structure compatible with the proposed and TBC schemes.

### C. Fixed Power Only

The paper discusses both fixed-power and variable-power variants. We implement **fixed-power only**, consistent with our energy model where TX cost is `L*E_elec + L*E_amp*d²` (no power control). Variable power is not applicable to our LEACH-inherited constant-amplitude model.

### D. No Adaptive Burst Size

Unlike the proposed scheme, FCPA uses **fixed M = 10 packets** throughout. This is a deliberate modelling choice to isolate the effect of CH election strategy and cooperative relay from the energy savings of adaptive burst size.

---

## Implementation: `schemes/run_fcpa.m`

### Per-Round Algorithm

**Step 1 — Channel quality and IPN gate**
```matlab
p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
d_jammer   = sqrt((x - J_x(t)).^2 + (y - J_y(t)).^2);
ipn_jammed = d_jammer < r_j;
```
`p(i)` is the packet success probability from our standard exponential decay model. `ipn_jammed` is the binary IPN gate derived from jammer geometry.

**Step 2 — CH Election (LEACH epoch + IPN eligibility)**
```matlab
T_thresh(G & alive & ~ipn_jammed) = p_CH / denom;
is_CH = (rand(1, N) < T_thresh) & alive;
```
Only alive, epoch-eligible, non-IPN-jammed nodes can be elected as CH. Fallback ensures at least one CH exists per round.

**Step 3 — Cluster Formation**
Each alive non-CH node joins the nearest CH within `r_tx = 50m`. Nodes with no CH within range are **stranded** — their `M` packets count as lost in the PDR denominator, no energy cost.

**Step 4 — Direct member transmission**
Non-jammed members (`ipn_jammed = false`) transmit `M` packets directly to their CH:
```matlab
energy(i) -= compute_energy('tx', L, ..., d_ic)
energy(c) -= compute_energy('rx', L, ...)
recv = sum(rand(M, 1) <= p(i))      % stochastic channel loss
```

**Step 5 — Cooperative relay for jammed members (FCPA Eq. 7)**

Jammed members (`ipn_jammed = true`) cannot transmit directly to their CH. For each jammed member `i`:

1. Find **relay candidates**: alive, non-jammed, non-CH nodes within `r_tx` of both `i` and the CH `c`.

2. Compute cooperative energy cost for each candidate relay `j`:
```
e_coop(j) = TX(i→j) + RX(j) + TX(j→c)
           = [L·E_elec + L·E_amp·d(i,j)²]
           + [L·E_elec]
           + [L·E_elec + L·E_amp·d(j,c)²]
```

3. Compute normalised weight (FCPA Eq. 7):
```
w_raw(j) = 1 / e_coop(j)
w(j)     = w_raw(j) / sum_k(w_raw(k))
```

4. Select relay: `j* = argmax(w) = argmin(e_coop)` — the relay that minimises the total 2-hop energy overhead.

5. Charge energy:
```matlab
energy(i)  -= compute_energy('tx', L, ..., d(i, j*))   % i transmits
energy(j*) -= compute_energy('rx', L, ...)              % j* receives from i
energy(j*) -= compute_energy('tx', L, ..., d(j*, c))   % j* forwards to CH
energy(c)  -= compute_energy('rx', L, ...)              % CH receives from j*
```

6. Apply channel loss at each hop:
```matlab
recv_at_relay = sum(rand(M, 1) <= p(i))                     % hop 1: i → j*
recv_at_ch    = sum(rand(recv_at_relay, 1) <= p(j*))        % hop 2: j* → c
```

If no relay exists (all non-jammed nodes unreachable): packets lost, no energy charged.

**Step 6 — CH aggregation + direct TX to BS**
```matlab
energy(c) -= compute_energy('agg', L, ..., n_members_c)
energy(c) -= compute_energy('tx',  L, ..., dist_to_BS(c))
```
Same direct CH→BS path as LEACH. No inter-cluster Dijkstra routing.

**Step 7 — Deferred energy application and node death**
All energy deltas are accumulated in a temporary array and applied at once after all nodes are processed, preventing mid-round death cascades (same pattern as TBC and proposed).

**Step 8 — Metrics**
```matlab
PDR(t)    = total_recv / total_sent
energy(t) = sum(energy(alive))
delay(t)  = mean hops per routed source (1 for direct, 2 for relay)
```
`total_sent` = M × (alive nodes), including stranded (counted as lost). Consistent with LEACH and TBC accounting.

---

## What FCPA Does NOT Have (vs Proposed)

| Feature | Proposed | FCPA |
|---|---|---|
| Jamming signal | EWMA JR (estimated from observed loss) | Geometric IPN (exact jammer position) |
| Memory | EWMA smoothing, ~3-round lag | None — resets each round |
| CH ineligibility | JR > threshold (continuous score) | d_jammer < r_j (binary gate) |
| Burst size | Adaptive M_eff = max(2, round(M×(1−JR))) | Fixed M = 10 |
| Relay routing | CH→BS only (Dijkstra confirms direct at this geometry) | Intra-cluster cooperative relay for jammed members |
| Election score | CHScore (energy + connectivity + JR + BS-dist) | Epoch threshold gated by IPN |

---

## What This Comparison Isolates

| Scheme | Jamming Awareness | Memory | Relay Strategy |
|---|---|---|---|
| TBC | Instantaneous PDR threshold | None | Flat multi-hop Dijkstra |
| FCPA | Exact jammer geometry (IPN gate) | None | Cooperative intra-cluster relay |
| Proposed | EWMA JR from experienced loss | Yes (lambda=0.6) | Direct CH→BS after JR-aware election |

FCPA vs Proposed tests two specific questions:
1. Does **exact jammer position** knowledge outperform **learned temporal estimates** (EWMA JR)?
2. Does **intra-cluster cooperative relay** for jammed members offset the energy overhead it introduces?

FCPA has a structural advantage (omniscient jammer geometry) but a structural disadvantage (no temporal memory, relay overhead, fixed burst size). The result indicates which factor dominates.

---

## Adaptation Omissions and Justifications

| Paper Feature | Status | Reason |
|---|---|---|
| K-medoids clustering | Replaced with LEACH epoch + IPN gate | K-medoids incompatible with round-by-round LEACH energy accounting |
| Variable-power mode | Omitted | Our energy model uses fixed amplifier constant E_amp×d²; no power control |
| Multi-jammer scenario | N/A | Single UAV jammer by design |
| Static jammer evaluation | N/A | Our UAV jammer is mobile (harder threat model) |
| IPN from received power | Replaced with d_jammer < r_j gate | Jammer position available directly as J_x(t)/J_y(t); avoids introducing a separate IPN parameter |

---

## Citation Summary for Paper

- Architecture overview: Section II, Figure 1
- IPN metric and CH election: Section III-B, Eq. (4)–(5)
- Cooperative relay mechanism: Section III-C
- Load-balanced relay weight: Eq. (7)
- Fixed vs variable power comparison: Section IV-A
- Performance context: Table II — FCPA achieves ~92% PDR vs ~74% LEACH in their static-jammer NS-2 scenario; our mobile UAV scenario is a harder and structurally different threat model
