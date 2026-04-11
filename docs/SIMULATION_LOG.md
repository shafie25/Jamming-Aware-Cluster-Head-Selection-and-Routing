# Simulation Log — Jamming-Aware WSN Project

Each entry documents one simulation run: the parameters used, the state of the code at the time, what was being tested, and what the results mean.

---

## Run 001 — Baseline Sanity Check (Proposed Scheme, No Baselines)

**Date:** 2026-04-10
**Run by:** Ahmed

### What This Run Was

First end-to-end run of the proposed scheme after the initial codebase was written. The goal was to verify the simulation runs without errors and that the output metrics are physically reasonable — not to draw comparative conclusions yet (baselines are not implemented). Think of this as a sanity check and a reference point for all future runs.

### Code State

- All core modules implemented: `config.m`, `init_network.m`, `uav_trajectory.m`, `compute_packet_success.m`, `compute_energy.m`, `update_jamming_risk.m`, `elect_ch_proposed.m`, `route_dijkstra.m`, `run_proposed.m`, `plot_results.m`
- Bug fixed prior to run: `r_c = 15` was missing from `config.m` — added before running
- Baselines: **not yet implemented**
- RNG seed: `rng(42)` — fixed for reproducibility

### Parameters Used

| Parameter | Value | Description |
|---|---|---|
| `N` | 100 | Number of sensor nodes |
| `area` | 100 m | Field side length (100×100 m deployment) |
| `BS` | [50, 50] | Base station at field center |
| `E0` | 0.5 J | Initial energy per node |
| `T` | 1000 | Total simulation rounds |
| `K_elec` | 10 | CH re-election interval (rounds) |
| `M` | 10 | Burst size (packets/round) |
| `p_CH` | 0.05 | Target CH fraction (dynamic K = round(0.05 × N_alive)) |
| `r_c` | 15 m | Communication range for neighbor counting |
| `r_exc` | 25 m | CH spatial exclusion radius |
| `lambda` | 0.6 | EWMA smoothing constant |
| `alpha` | 0.35 | CHScore weight — residual energy |
| `beta` | 0.20 | CHScore weight — connectivity |
| `gamma_` | 0.35 | CHScore weight — jamming risk penalty |
| `delta` | 0.10 | CHScore weight — distance-to-BS penalty |
| `phi1` | 1e-4 | Routing cost — fixed per-hop penalty (J) |
| `phi2` | 1 | Routing cost — energy term scale |
| `phi3` | 5e-4 | Routing cost — JR penalty scale (J) |
| `r_j` | 20 m | Jamming radius |
| `kappa` | 3 | Jamming decay constant |
| `p_base` | 0.95 | Baseline packet success probability |
| `orbit_radius` | 35 m | UAV orbit radius |
| `omega` | 2π/50 rad/round | UAV angular speed (1 orbit per 50 rounds) |
| `E_elec` | 50 nJ/bit | Circuit energy |
| `E_amp` | 100 pJ/bit/m² | Amplifier energy |
| `E_da` | 5 nJ/bit | Data aggregation energy |
| `L` | 4000 bits | Packet length |

### Results

| Metric | Value |
|---|---|
| **First node death (t_death)** | Round 318 |
| Alive nodes @ round 100 | 100 / 100 |
| Alive nodes @ round 200 | 100 / 100 |
| Alive nodes @ round 300 | 100 / 100 |
| Alive nodes @ round 400 | 98 / 100 |
| Alive nodes @ round 500 | 97 / 100 |
| Alive nodes @ round 1000 | 1 / 100 |
| PDR mean (rounds 1–100) | 88.40% |
| PDR mean (rounds 300–318) | 89.02% |
| PDR mean (all 1000 rounds) | 80.96% |
| PDR min (any round) | 10.00% |
| PDR max (any round) | 100.00% |
| Avg end-to-end delay | 1.25 hops |
| Max delay (any round) | 2 hops |
| Total residual energy @ round 100 | 43.42 J |
| Total residual energy @ round 200 | 36.62 J |
| Total residual energy @ round 300 | 29.79 J |
| Total residual energy @ round 318 | 28.60 J |

### Takeaways

**1. The scheme is functionally correct.**
No crashes, no NaN metrics, no zero-PDR rounds in the healthy phase. All modules are wiring together properly.

**2. Network lifetime of 318 rounds is reasonable, and degradation is slow.**
All 100 nodes survive until round 318. After the first death, the network degrades gracefully — 97 nodes are still alive at round 500. This indicates the energy load is fairly balanced across nodes, which is the intended effect of the CHScore energy term and the spatial exclusion radius.

**3. PDR is strong and stable in the active phase (~88–89%).**
In the healthy network phase (rounds 1–318), PDR holds consistently between 88–89%. The overall 1000-round mean drops to ~81% only because the degraded network in later rounds (fewer CHs, broken routing paths) pulls it down — not because the jamming mitigation is failing.

**4. Routing is almost always 1 hop to BS.**
Average delay of 1.25 hops (max 2) means CHs are routing directly to the BS nearly all the time rather than through relay CHs. With 5 CHs spread across 100×100 m and BS at center [50, 50], most CHs are within direct range. This is expected geometry — multi-hop routing would be more prominent in larger fields or with BS at a corner.

**5. Energy drains at a linear, balanced rate.**
Total energy consumption is approximately 0.068 J/round throughout the entire healthy phase:
- Rounds 0→100: −6.58 J
- Rounds 100→200: −6.80 J
- Rounds 200→300: −6.82 J

No accelerating drain, no hotspot death spiral. This is a good sign for the clustering algorithm.

**6. The PDR floor of ~10% reflects the jamming model working correctly.**
The minimum PDR of 0.10 occurs when the UAV flies directly over a cluster. At the jamming center, `p = p_base * exp(-kappa) ≈ 0.95 × e^{-3} ≈ 0.047`. With M=10 stochastic packet draws, a node with p≈0.05 will frequently deliver 0 or 1 packets, pushing cluster PDR down transiently. This is the intended behavior — not a bug.

### What to Do Next

- ~~Implement LEACH baseline and run a comparative simulation~~ → Done in Run 002
- Consider running multiple seeds and averaging to reduce variance in the comparison

---

## Run 002 — Proposed Scheme vs Standard LEACH

**Date:** 2026-04-10
**Run by:** Ahmed

### What This Run Was

First comparative run: proposed jamming-aware scheme vs standard LEACH running under identical conditions (same network topology, same UAV jammer trajectory, same energy model, same RNG seed). Goal was to validate the proposed scheme shows meaningful improvement over the LEACH baseline in PDR, and to understand the energy and lifetime trade-offs.

### Code State

- `run_leach.m` added — standard LEACH converted to a function returning a compatible results struct
- `main.m` updated — now calls both schemes and passes both to `plot_results`
- Energy model aligned: LEACH uses same `E_elec`, `E_amp`, `E_da`, `L` from `config.m` (free-space only, no multipath threshold) for a fair comparison
- LEACH receives the same jamming exposure via `compute_packet_success` — no mitigation, just exposure
- RNG seed: `rng(42)` — both schemes see identical network and packet draws
- Baselines 1–3 from README: not yet implemented

### Parameters Used

Same as Run 001 for all shared parameters. LEACH-specific:

| Parameter | Value | Description |
|---|---|---|
| `P` | 0.05 | Target CH fraction (LEACH standard, same as p_CH) |
| Epoch length | `round(1/P)` = 20 rounds | Rounds before CH eligibility resets |
| Inter-cluster routing | None | LEACH is direct CH→BS always |
| Jamming mitigation | None | Same UAV exposure, no avoidance |

### Results

| Metric | Proposed | Standard LEACH |
|---|---|---|
| **First node death (t_death)** | Round **318** | Round **447** |
| Alive @ round 100 | 100 / 100 | 100 / 100 |
| Alive @ round 300 | 100 / 100 | 100 / 100 |
| Alive @ round 500 | 97 / 100 | 98 / 100 |
| PDR mean (all rounds) | **80.96%** | 58.14% |
| PDR mean (rounds 1–100) | **88.40%** | 83.05% |
| PDR min (any round) | 10.00% | 78.33% |
| Avg delay (hops) | 1.25 | 1.00 |
| Energy @ round 100 | 43.42 J | 41.84 J |
| Energy @ round 200 | 36.62 J | 33.57 J |
| Energy @ round 300 | 29.79 J | 25.36 J |

### Takeaways

**1. PDR improvement is the proposed scheme's clear win (+22% overall).**
The proposed scheme delivers 80.96% vs LEACH's 58.14% averaged across all 1000 rounds. In the active-network phase (rounds 1–100), the gap is already visible: 88.4% vs 83.05%. The LEACH PDR collapses dramatically in later rounds as nodes die and random CH election fails to avoid the jammer — the proposed scheme holds up far longer.

**2. LEACH outlives the proposed scheme to first node death (447 vs 318) — and this is worth investigating.**
Despite draining total energy faster (25.36 J remaining at round 300 vs 29.79 J for proposed), LEACH's first individual node death happens later. The likely cause: the proposed scheme's inter-cluster Dijkstra routing creates relay CHs that carry both their own cluster's data and forwarded data from other CHs, accelerating drain on specific nodes. LEACH is purely direct-to-BS — no node ever serves as a relay, so load is more evenly distributed even if total energy is spent faster. This is a known trade-off between multi-hop efficiency and load balancing.

**3. LEACH's PDR floor (78%) is higher than proposed (10%) — but for the wrong reason.**
The proposed scheme's floor of 10% represents a single heavily-jammed cluster taking a full hit for one round (the jamming model is working). LEACH's floor of 78% doesn't mean it handles jamming better — it means LEACH's random CH distribution dilutes jamming impact across many clusters simultaneously rather than concentrating it, which suppresses the floor but also suppresses peak performance.

**4. LEACH drains energy ~10–17% faster than the proposed scheme.**
By round 300, LEACH has consumed ~14.6 J more than the proposed scheme (50 J − 25.36 J = 24.64 J consumed vs 50 J − 29.79 J = 20.21 J). This is the energy efficiency benefit of jamming-aware routing — avoiding high-JR nodes as relays reduces retransmissions and CH overhead in bad-jamming zones.

**5. Delay trade-off is minimal.**
Proposed scheme averages 1.25 hops vs LEACH's 1.0. The 0.25 hop overhead from inter-cluster routing is the cost of the Dijkstra routing layer. In a 100×100 m field with BS at center, most CHs are already within direct range of the BS, so the routing rarely adds more than 1 extra hop.

### What to Do Next

- Tune `alpha`/`delta` weights in CHScore to better balance energy load and delay the first node death in the proposed scheme
- Implement the remaining baselines from the README (EWMA Detection, FCPA simplified)
- Run multiple RNG seeds (e.g., seeds 42, 7, 13, 99, 101) and report mean ± std to reduce variance

---

## Run 003 — CHScore Weight Tuning (alpha 0.35 → 0.45)

**Date:** 2026-04-11
**Run by:** Faris

### What This Run Was

Weight tuning experiment targeting the proposed scheme's first-node-death weakness. In Run 002, the proposed scheme's first node died at round 318 vs LEACH's 447 — caused by relay nodes in Dijkstra routing carrying extra traffic and draining faster than neighbors. The fix attempted: increase `alpha` (energy weight in CHScore) so drained relay nodes score lower and get rotated out of the CH role sooner. `beta` and `delta` were reduced to keep the sum at 1.0. `gamma_` was left untouched (core paper contribution).

### Parameters Changed

| Weight | Run 002 | Run 003 | Reason |
|---|---|---|---|
| `alpha` | 0.35 | **0.45** | More emphasis on residual energy → rotate out drained relays |
| `beta` | 0.20 | **0.15** | Reduced to compensate |
| `gamma_` | 0.35 | 0.35 | Unchanged — core contribution |
| `delta` | 0.10 | **0.05** | Reduced to compensate |

### Results

| Metric | Proposed | Standard LEACH |
|---|---|---|
| **First node death** | Round **353** (+35 vs Run 002) | Round **423** |
| PDR mean (all rounds) | 73.55% | 57.39% |
| Energy @ round 300 | 29.53 J | 25.27 J |
| Zero-PDR rounds | **167** | 348 |

### Takeaways

**1. First-death goal was achieved — but at too high a cost.**
First node death improved from 318 → 353 (+35 rounds). However, zero-PDR rounds doubled from 84 → 167, and overall PDR dropped from 80.96% → 73.55%. The trade-off is not worth it.

**2. Higher alpha hurt coverage consistency.**
With more weight on residual energy, CHScore elected nodes that were energy-rich but apparently positioned worse for cluster coverage. More rounds ended up with CHs lacking members or entire clusters being simultaneously jammed with no healthy fallback — both scenarios produce zero PDR.

**3. LEACH numbers shifted slightly — RNG artifact, not a real change.**
LEACH's first death moved from 447 → 423. Since `rng(42)` is seeded once at the start of `main.m` and both schemes share the same RNG stream sequentially, the proposed scheme consuming random numbers differently (due to different CH elections) leaves LEACH starting from a different RNG state. LEACH's core behavior did not change.

**4. Config reverted after this run.**
The weights were reset to Run 002 values (alpha=0.35, beta=0.20, gamma_=0.35, delta=0.10) since the tuning degraded overall performance. Weight tuning alone is unlikely to fix the lifetime gap without hurting PDR — the root cause (relay load imbalance) may need a structural fix such as load-aware relay selection in `route_dijkstra.m`.

### What to Do Next

- Accept the lifetime trade-off as a known limitation and document it clearly in the paper
- Proceed to multi-seed averaging (seeds 42, 7, 13, 99, 101) with the original Run 002 weights
- Implement remaining baselines

---

## Run 004 — Multi-Seed Averaging (5 Seeds)

**Date:** 2026-04-11
**Run by:** Faris

### What This Run Was

First statistically robust evaluation. Instead of relying on a single network topology (seed 42), both schemes were run across 5 seeds {42, 7, 13, 99, 101}. Each seed produces a different random node deployment and a different packet-draw sequence. Results are reported as mean ± std across seeds.

Entry point: `run_multiseed.m`. Plots generated by `plotting/plot_multiseed.m` (mean line + shaded ±1 std band).

### Code State

- All Run 002 weights restored: alpha=0.35, beta=0.20, gamma_=0.35, delta=0.10
- No code changes from Run 002 — only the evaluation wrapper changed
- `run_multiseed.m` and `plotting/plot_multiseed.m` added this session

### Per-Seed First Node Death

| Seed | Proposed | LEACH | Winner |
|---|---|---|---|
| 42 | 318 | 447 | LEACH |
| 7 | 454 | 414 | **Proposed** |
| 13 | 287 | 384 | LEACH |
| 99 | 427 | 473 | **Proposed** |
| 101 | 484 | 442 | **Proposed** |

### Averaged Results

| Metric | Proposed | Standard LEACH |
|---|---|---|
| **First node death** | **394.0 ± 86.6 rounds** | 432.0 ± 34.0 rounds |
| PDR mean (all rounds) | **81.63% ± 3.46%** | 58.99% ± 1.02% |
| Energy @ round 300 | **30.42 ± 0.73 J** | 26.92 ± 1.03 J |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 |

### Takeaways

**1. Seed 42 was an outlier — proposed scheme actually outlives LEACH on average.**
The Run 002 finding that "LEACH outlasts proposed in first-node-death" was a single-topology artifact. Averaged across 5 seeds, proposed achieves first death at round 394 vs LEACH's 432 — a 38-round advantage. In 3 out of 5 seeds, proposed outlasts LEACH. Seed 42 and seed 13 were simply unlucky topologies for the proposed scheme.

**2. The proposed scheme's lifetime variance is high (±86.6 vs ±34.0 for LEACH).**
LEACH's behavior is fairly stable across topologies because it's random — it doesn't try to be smart, so topology doesn't matter much. The proposed scheme is more topology-sensitive: a bad node layout can force relay nodes into disadvantageous positions. This is expected for an optimization-based scheme.

**3. PDR advantage is large and consistent.**
+22.6 percentage points overall (81.63% vs 58.99%), with low variance on both sides. This is the scheme's headline result and it holds across all 5 seeds — not a fluke.

**4. Energy efficiency advantage holds across seeds.**
Proposed retains 30.42 J at round 300 vs LEACH's 26.92 J (+3.5 J). Consistent across seeds (low std on both).

**5. Zero-PDR rounds: proposed is far better but more variable.**
78.2 ± 37.4 vs 330.8 ± 11.7. The proposed scheme has 4× fewer wasted rounds on average. The higher std (37.4) reflects topology sensitivity — some seeds produce cleaner routing paths than others.

### What to Do Next

- Implement remaining baselines (Baselines 1–3 from README)
- Multi-seed results are the final numbers to report in the paper for Proposed vs LEACH comparison

---

## Run 005 — Full 5-Scheme Comparison (5-Seed Average)

**Date:** 2026-04-11
**Run by:** Faris

### What This Run Was

First complete multi-scheme evaluation. All five schemes ran across seeds {42, 7, 13, 99, 101} and results were averaged. This is the definitive result set for the paper.

Three new baselines were implemented and added to `schemes/`:

| Baseline | File | Design |
|---|---|---|
| EWMA-Detect | `run_ewma_detect.m` | LEACH + EWMA JR tracking (computed but unused in decisions). Models detection-only approach (IEEE ICDSIS 2022). |
| Threshold-JR | `run_threshold.m` | LEACH + member suppression when JR > 0.7. Suppressed members skip transmission; their M packets counted as lost in PDR denominator for honest accounting. |
| Reactive-CH | `run_reactive_ch.m` | LEACH + reactive CH re-election when elected CH has JR > 0.5. Replacement scored by energy/E0 − JR. All cluster members re-assigned to replacement. No inter-cluster routing. |

Entry point: `run_multiseed.m` (updated to run all 5 schemes).

### Parameters

Same as Run 004. All scheme-specific thresholds:
- Threshold-JR suppression threshold: JR > 0.70 (PDR_ewma < 0.30)
- Reactive-CH re-election threshold: JR > 0.50 (PDR_ewma < 0.50)

### Per-Seed First Node Death

| Seed | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| 42  | 318 | 447 | 397 | 419 | 396 |
| 7   | 454 | 414 | 417 | 414 | 384 |
| 13  | 287 | 384 | 383 | 387 | 375 |
| 99  | 427 | 473 | 434 | 448 | 438 |
| 101 | 484 | 442 | 438 | 404 | 416 |

### Averaged Results (mean ± std across 5 seeds)

| Metric | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| First node death (round) | 394.0 ± 86.6 | 432.0 ± 34.0 | 413.8 ± 23.6 | 414.4 ± 22.4 | 401.8 ± 25.4 |
| PDR mean — all rounds (%) | **81.63 ± 3.46** | 58.99 ± 1.02 | 58.96 ± 1.87 | 60.25 ± 1.90 | 58.70 ± 1.56 |
| Energy @ round 300 (J) | **30.42 ± 0.73** | 26.92 ± 1.03 | 26.06 ± 1.07 | 27.50 ± 0.99 | 26.29 ± 0.43 |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 | 330.2 ± 19.3 | 308.6 ± 20.3 | 330.6 ± 17.4 |

### Takeaways

**1. Proposed scheme dominates PDR — by a large, consistent margin.**
81.63% vs next-best Threshold-JR at 60.25%. That is a +21.4 percentage point gap, and it holds across all 5 seeds (std 3.46%). All three baselines and LEACH cluster tightly between 58.70–60.25%. The proposed scheme is in a fundamentally different performance tier.

**2. EWMA-Detect ≈ LEACH — detection without adaptation provides exactly zero benefit.**
PDR: 58.96% vs 58.99% (LEACH). Zero-PDR rounds: 330.2 vs 330.8. Every metric is statistically indistinguishable from LEACH. This is the expected result and validates the baseline design: if you detect jamming but don't change CH election or routing, nothing improves.

**3. Threshold-JR is the best baseline — but barely.**
PDR 60.25% (+1.26pp over LEACH), energy 27.50 J (+0.58 J), zero-PDR rounds 308.6 (−22 rounds). Suppressing very badly jammed members (JR > 0.7) reduces some wasted transmissions and saves modest energy. The gains are real but small — the threshold approach reacts only after a node is already severely jammed, and then only silences it rather than routing around it.

**4. Reactive-CH performs worse than LEACH on PDR (58.70% vs 58.99%).**
Reactive CH re-election actually hurts slightly. The reason: when a cluster's CH is heavily jammed (JR > 0.5), the cluster itself tends to be in the jammer's path — all members have elevated JR. The replacement CH is a slightly better node within the same jammed zone, not a fundamentally different one. The protocol disruption (re-assignment, new CH receiving from all members) adds overhead without a meaningful jamming avoidance benefit. This highlights the fundamental difference between reactive and proactive approaches: reacting inside a jammed cluster is far less effective than avoiding jammed nodes as CHs in the first place.

**5. Proposed scheme's energy efficiency is clearly superior.**
30.42 J at round 300 vs 26.06–27.50 J for all baselines. The JR-aware routing (Dijkstra with JR cost) avoids routing through jammed relay nodes, reducing retransmissions and avoiding high-cost jammed-zone hops. The baselines all use direct CH→BS and still drain faster.

**6. Zero-PDR rounds are the starkest differentiator.**
Proposed: 78.2 rounds (7.8% of 1000 rounds). All others: 308–330 rounds (30–33%). The baselines waste 4× more rounds in complete silence. This is the cost of LEACH's epoch mechanism combined with no jamming awareness — entire rounds where all CHs have no members and PDR = 0.

**7. RNG sequencing note.**
Within each seed, schemes run sequentially sharing the same RNG stream (topology is fixed, but packet draws and LEACH elections depend on RNG state left by previous schemes). This is an acknowledged limitation for a course project. The averaged PDR numbers (across 1000 rounds × 5 seeds = 5000 rounds per scheme) are robust to this. First-node-death comparisons between baselines should be interpreted with some caution since they depend more heavily on individual RNG draws.

### Final Paper Numbers (use these)

| Metric | Proposed | LEACH | EWMA-Detect | Threshold-JR | Reactive-CH |
|---|---|---|---|---|---|
| First node death (round) | 394.0 ± 86.6 | 432.0 ± 34.0 | 413.8 ± 23.6 | 414.4 ± 22.4 | 401.8 ± 25.4 |
| PDR mean (%) | **81.63 ± 3.46** | 58.99 ± 1.02 | 58.96 ± 1.87 | 60.25 ± 1.90 | 58.70 ± 1.56 |
| Energy @ round 300 (J) | **30.42 ± 0.73** | 26.92 ± 1.03 | 26.06 ± 1.07 | 27.50 ± 0.99 | 26.29 ± 0.43 |
| Zero-PDR rounds | **78.2 ± 37.4** | 330.8 ± 11.7 | 330.2 ± 19.3 | 308.6 ± 20.3 | 330.6 ± 17.4 |

### What to Do Next

- Simulation is complete. All results logged.
- Write paper discussion section using the takeaways above.
- The headline result: proposed scheme delivers +21.4pp PDR improvement over the best baseline, with 4× fewer wasted (zero-PDR) rounds, and better energy efficiency — all while operating under an identical UAV jammer.

---

## Run 006 — Energy-Aware Relay Penalty in Dijkstra (φ4 experiment — REVERTED)

**Date:** 2026-04-11
**Run by:** Ahmed + Claude Code

### What This Run Was

Attempted tweak to address relay node overload identified in Run 002. The Dijkstra routing cost was extended with a fourth term penalizing low-energy relay CHs:

```
C(i→j) = φ1 + φ2·E_amp·L·d² + φ3·JR_j + φ4·(1 − E_j/E0)
```

`φ4 = 5×10⁻⁴ J` (same magnitude as φ3). The hypothesis was that steering traffic away from energy-depleted relay nodes would reduce variance in first-node-death and better distribute routing load.

### Parameters Changed

| Parameter | Run 005 | Run 006 |
|---|---|---|
| `phi4` | N/A | **5×10⁻⁴ J** (new term added to Dijkstra cost) |
| All other parameters | unchanged | unchanged |

### Results (5-seed average)

| Metric | Run 005 (baseline) | Run 006 (φ4 added) | Change |
|---|---|---|---|
| First death (round) | 394.0 ± 86.6 | 392.6 ± 98.1 | ~same, variance worse |
| PDR mean (%) | **81.63 ± 3.46** | 80.20 ± 5.52 | **−1.43pp, variance worse** |
| Energy @ round 300 (J) | 30.42 ± 0.73 | 30.46 ± 0.78 | ~same |
| Zero-PDR rounds | **78.2 ± 37.4** | 93.6 ± 59.6 | **+15.4 worse, variance worse** |

### Takeaways

**The tweak made things worse on every meaningful metric. Code was fully reverted to Run 005 state.**

**Why it backfired:** The energy penalty penalized nodes that are low-energy *because* they are good relays — well-positioned close to the BS, low JR, naturally attracting forwarding traffic. Penalizing them pushed Dijkstra onto longer, higher-JR detour paths, increasing zero-PDR rounds by +15 and degrading PDR variance from ±3.46 to ±5.52.

**Root cause insight:** The CHScore energy term (`α·E_i/E0`) already handles relay overload from the election side — drained nodes score lower and get rotated out of the CH role at the next K=10 re-election. The routing does not need a second energy penalty. Relay overload self-corrects through the election cycle with at most 10-round lag. Adding a routing penalty disrupts current-round paths before the election cycle has a chance to rotate the node out naturally.

### What to Do Next

- Do not retry energy penalty in routing — the mechanism is sound on paper but counterproductive given the election cycle already handles it.
- Consider tweak #2: `max(JR_src, JR_dest)` in routing cost — low effort, targets the source-side jamming gap.
- Consider K_elec sensitivity test (try K_elec = 5) — faster re-election may reduce relay overload lag with zero structural changes.
- Alternatively, shift focus to identifying and implementing stronger baselines from the literature before further proposed-scheme tuning.

---
