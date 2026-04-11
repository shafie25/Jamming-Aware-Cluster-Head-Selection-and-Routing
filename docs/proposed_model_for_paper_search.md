# Proposed Model: Jamming-Aware Cluster Head Selection and Routing in WSNs under UAV-Based Interference

This document describes the full proposed model as implemented in MATLAB. It is intended to help identify relevant prior work and suitable comparison baselines from the literature.

---

## Problem Statement

We simulate a Wireless Sensor Network (WSN) of **100 static sensor nodes** randomly deployed in a **100×100 m field**, with a Base Station (BS) at the center [50, 50]. A **UAV-mounted jammer** follows a circular orbit over the field and actively degrades packet delivery for nodes within its jamming radius. The goal is to maximize packet delivery ratio (PDR) and network lifetime despite this interference, by integrating jamming awareness into both cluster head (CH) election and inter-cluster routing.

---

## Network and Jammer Model

### Network
- N = 100 nodes, uniform random deployment, static positions
- BS at field center [50, 50] m
- Initial energy per node: E_0 = 0.5 J
- Simulation runs for T = 1000 rounds
- Energy model: LEACH radio model (Heinzelman et al.)
  - Tx: `E_tx = L·E_elec + L·E_amp·d²`
  - Rx: `E_rx = L·E_elec`
  - Aggregation: `E_agg = n_members · L · E_da`
  - E_elec = 50 nJ/bit, E_amp = 100 pJ/bit/m², E_da = 5 nJ/bit, L = 4000 bits

### UAV Jammer
- Circular orbit centered at [50, 50] m with radius 35 m
- Angular speed ω = 2π/50 rad/round → one full orbit every 50 rounds
- Jamming radius r_j = 20 m

### Packet Success Probability (per node per round)
For node i at distance d_i from the jammer at round t:

```
         p_base                            if d_i > r_j
p_i(t) = 
         p_base · exp(−κ · (1 − d_i/r_j)) if d_i ≤ r_j
```

- p_base = 0.95 (baseline success rate outside jamming zone)
- κ = 3 (decay constant; at jammer center, p drops to ~0.047)

This is an exponential decay model: packet success degrades smoothly from p_base at the boundary to near-zero at the jammer's exact position.

---

## Layer 1: Jamming Risk Estimation (EWMA)

Each round, every alive **member node** (non-CH) attempts a burst of M = 10 packets. Jamming risk is estimated via three steps:

**Step 1 — Instantaneous PDR (Eq. 3):**
```
PDR_inst_i(t) = (1/M) · Σ Bernoulli(p_i(t))   [sum over M packet draws]
```

**Step 2 — EWMA-smoothed PDR (Eq. 4):**
```
PDR_ewma_i(t) = λ · PDR_inst_i(t) + (1−λ) · PDR_ewma_i(t−1)
```
- λ = 0.6 (recent observations weighted more heavily; tracks UAV movement while smoothing noise)

**Step 3 — Jamming Risk Score (Eq. 5):**
```
JR_i(t) = 1 − PDR_ewma_i(t)    ∈ [0, 1]
```

JR = 0 means no detected jamming; JR = 1 means complete packet loss. This is a **distributed, decentralized** estimate — each node computes its own JR from its own transmission outcomes without any explicit message from the jammer.

---

## Layer 1: CH Election — CHScore (Eq. 6)

CH re-election occurs every K_elec = 10 rounds. The number of CHs is K = round(0.05 · N_alive), consistent with LEACH's 5% target.

### CHScore Formula
Each alive node computes:

```
CHScore_i = α · (E_i / E_0)  +  β · (|N_i| / N_max)  −  γ · JR_i  −  δ · (d_i_BS / d_max)
```

| Term | Weight | Meaning |
|---|---|---|
| E_i / E_0 | α = 0.35 | Residual energy fraction — prefer energy-rich nodes |
| \|N_i\| / N_max | β = 0.20 | Normalized neighbor count within r_c = 15 m — prefer well-connected nodes |
| JR_i | γ = 0.35 | Jamming risk — **penalize** nodes currently being jammed |
| d_i_BS / d_max | δ = 0.10 | Distance to BS normalized by field diagonal — prefer closer nodes |

Weights sum to 1.0. The **jamming risk penalty (γ)** and energy term (α) carry equal weight and are the dominant factors, reflecting the paper's core claim.

### Greedy Spatial Election with Exclusion Radius
After scoring all alive nodes:
1. Sort nodes by CHScore descending
2. Elect the top-scoring eligible node as CH
3. **Suppress all nodes within r_exc = 25 m** of the new CH (exclusion radius derived from expected CH territory area: r ≈ sqrt(field_area / (π·K)) ≈ 25 m)
4. Repeat until K CHs are elected

This ensures **spatial spread** of CHs — avoids electing multiple CHs in the same region while leaving other regions uncovered.

### Cluster Assignment
Each alive non-CH node joins its **nearest CH** by Euclidean distance.

---

## Layer 2: Inter-Cluster Routing — Dijkstra (Eq. 7, 8)

Instead of each CH transmitting directly to the BS (as in LEACH), CHs form a **multi-hop relay graph**. Dijkstra's algorithm finds the minimum-cost path from each CH to the BS every round.

### Edge Cost (Eq. 7)
For a link from CH_i to relay node CH_j (or directly to BS):

```
C(i → j) = φ1  +  φ2 · E_amp · L · d_ij²  +  φ3 · JR_j
```

| Term | Value | Meaning |
|---|---|---|
| φ1 | 1×10⁻⁴ J | Fixed per-hop penalty — discourages unnecessary relay hops |
| φ2 · E_amp · L · d_ij² | φ2=1 | Propagation energy cost, quadratic in distance |
| φ3 · JR_j | φ3=5×10⁻⁴ J | **Avoids routing through jammed relay nodes** |

The JR penalty on the destination node means the algorithm naturally steers traffic away from relays currently in the jammer's path.

### Dijkstra Execution
- Graph nodes: all current CHs + BS (BS treated as a super-sink with JR = 0)
- Run once per round for each CH
- Output: ordered relay path from CH to BS; energy deducted per hop

---

## Round Loop (Full Execution Flow)

Each of the T = 1000 rounds executes the following sequence:

```
1. [Every K_elec rounds] Run CHScore election → assign clusters → pay control overhead energy
2. Compute p_i(t) for all nodes (jammer position J_x(t), J_y(t))
3. Member nodes attempt M packets → update PDR_ewma and JR
4. Deduct Tx/Rx energy: member → CH links
5. Run Dijkstra routing for all CHs
6. CH aggregates member data → deducts E_agg
7. CH forwards along relay path to BS → deducts Tx/Rx per hop
8. Record metrics: PDR, total residual energy, average hop count, alive node count
9. Kill nodes with energy ≤ 0; record first-death round
```

---

## Key Metrics Reported

| Metric | Definition |
|---|---|
| PDR (%) | (packets received at BS) / (packets sent by all member nodes), per round |
| Network lifetime | Round of first node death |
| Residual energy at round 300 | Sum of all alive node energies (J) |
| Zero-PDR rounds | Number of rounds where PDR = 0 (complete delivery failure) |

---

## Current Baselines (Already Implemented)

| Scheme | Description |
|---|---|
| **LEACH** | Standard probabilistic CH election, direct CH→BS transmission, no jamming awareness |
| **EWMA-Detect** | LEACH + EWMA jamming detection (detection computed but never acted upon — confirms that detection alone without adaptation is useless) |
| **Threshold-JR** | LEACH + member transmission suppression when a member's JR > 0.70 (reduces wasted energy on jammed packets, does not change CH election or routing) |
| **Reactive-CH** | LEACH + reactive CH re-election within a cluster when the current CH's JR > 0.50 (replaces jammed CH with a member, but the replacement is typically still inside the same jammed zone) |

---

## Summary of Final Results (5-seed average, 1000 rounds)

| Metric | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| PDR mean (%) | **81.63 ± 3.46** | 58.99 ± 1.02 | 58.96 ± 1.87 | 60.25 ± 1.90 | 58.70 ± 1.56 |
| First death (round) | 394.0 ± 86.6 | 432.0 ± 34.0 | 413.8 ± 23.6 | 414.4 ± 22.4 | 401.8 ± 25.4 |
| Energy @ round 300 (J) | **30.42 ± 0.73** | 26.92 ± 1.03 | 26.06 ± 1.07 | 27.50 ± 0.99 | 26.29 ± 0.43 |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 | 330.2 ± 19.3 | 308.6 ± 20.3 | 330.6 ± 17.4 |

**Headline result:** +21.4 percentage point PDR improvement over the best baseline (Threshold-JR), with 4× fewer zero-PDR rounds, and better energy efficiency.

---

## What to Search for in the Literature

Based on the model above, the following areas are relevant for finding baseline papers or closely related prior work:

1. **LEACH and energy-efficient clustering in WSNs** — original energy model and probabilistic CH election
2. **Jamming-aware or interference-aware clustering** — CHScore-like formulations that include jamming/interference in CH election
3. **UAV-based jamming or eavesdropping in WSNs** — threat models where an aerial adversary moves over the network
4. **EWMA-based link quality estimation in WSNs** — using exponential moving average to track PDR or LQI
5. **Multi-hop inter-cluster routing in WSNs with energy/interference cost** — Dijkstra or shortest-path routing where edge cost combines energy and link quality
6. **Anti-jamming routing in wireless networks** — graph-based approaches that route around jammed regions
7. **Jamming-resilient WSN protocols** — any scheme that modifies MAC or routing behavior based on detected jamming
8. **Threshold-based or reactive CH re-election in WSNs** — papers that replace CHs during a round based on energy or link quality thresholds

### Specific questions for the literature search:
- Is there a prior work that combines **both** CH election **and** routing with jamming awareness (as opposed to treating them separately)?
- Are there UAV-jammer threat model papers that use a similar exponential decay or distance-based packet success model?
- Is there a standard EWMA-based jamming risk formulation (JR = 1 − PDR_ewma) used in prior WSN security papers?
- Are there papers that use a CHScore-style multi-criteria weighted sum for CH election (energy + connectivity + interference)?
- Are there more suitable baselines than the four above — particularly any scheme that uses jamming information for routing but not CH election, or CH election but not routing?
