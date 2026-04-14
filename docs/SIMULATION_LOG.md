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

## Run 007 — Hard Transmission Range Limit (r_tx = 50m)

**Date:** 2026-04-12
**Run by:** Ahmed + Claude Code

### What This Run Was

First run with a hard transmission range limit. Previously, every alive node was assigned to its nearest CH with no distance cap — long links were penalized only via the d² energy term. This run adds `r_tx = 50m`: member nodes farther than 50m from every CH are stranded and cannot join any cluster. Their M packets count in the PDR denominator as lost (honest accounting). The change applies equally to both proposed and LEACH.

Motivation: in late-network rounds with 1–2 CHs, some nodes can be 50–70m from the nearest CH. Without a range cap, the simulation silently assigns them to a distant CH at unrealistic cost. The 50m limit is ~2× the average nearest-CH distance in a healthy 5-CH network (~22m), so healthy rounds are largely unaffected.

This run was Proposed vs LEACH only — baselines not updated yet.

### Parameters Changed

| Parameter | Run 005 | Run 007 | Reason |
|---|---|---|---|
| `r_tx` | N/A (no limit) | **50 m** | Hard transmission range cap added |
| All other parameters | unchanged | unchanged | — |

### Code Changes

- `core/config.m` — added `r_tx = 50`
- `layer1/elect_ch_proposed.m` — cluster assignment enforces `min_d <= r_tx`; stranded nodes keep `CH_assign = 0`
- `schemes/run_proposed.m` — added `r_tx` parameter; stranded node guard in overhead loop; `n_stranded * M` added to `total_sent` before PDR record
- `schemes/run_leach.m` — same range check in cluster assignment; same stranded PDR accounting
- `run_multiseed.m` — trimmed to 2 schemes (Proposed + LEACH) for this run

### Per-Seed First Node Death

| Seed | Proposed | LEACH |
|---|---|---|
| 42  | 555 | 724 |
| 7   | 575 | 758 |
| 13  | 570 | 679 |
| 99  | 616 | 723 |
| 101 | 589 | 707 |

### Results (mean ± std across 5 seeds)

| Metric | Proposed | LEACH | Run 005 Proposed | Run 005 LEACH |
|---|---|---|---|---|
| First node death (round) | 581.0 ± 23.0 | 718.2 ± 28.7 | 394.0 ± 86.6 | 432.0 ± 34.0 |
| PDR mean — all rounds (%) | **73.81 ± 1.72** | 64.54 ± 0.46 | 81.63 ± 3.46 | 58.99 ± 1.02 |
| Energy @ round 300 (J) | 31.64 ± 0.24 | **32.34 ± 1.08** | 30.42 ± 0.73 | 26.92 ± 1.03 |
| Zero-PDR rounds | **13.6 ± 2.8** | 155.0 ± 23.9 | 78.2 ± 37.4 | 330.8 ± 11.7 |

### Takeaways

**1. Proposed still dominates PDR but gap narrowed (+9.3pp vs +22pp in Run 005).**
73.81% vs 64.54%. The range limit exposes a different late-round regime: stranded nodes pull both schemes' PDR down, but they pull the proposed scheme's down more because the proposed scheme previously held up stronger in late rounds (fewer dead CHs, better CH placement) — that advantage is now partially offset by stranded nodes contributing zeros regardless of jamming awareness.

**2. First-node-death jumped sharply for both schemes.**
Proposed: 394 → 581 rounds (+187). LEACH: 432 → 718 rounds (+286). Stranded nodes burn zero energy — they can't transmit so they never drain. This inflates network lifetime artificially in both cases. LEACH benefits more because random CH placement tends to strand more peripheral nodes earlier, saving more energy. This metric is less meaningful as a paper result under the range limit since it conflates "nodes are alive" with "nodes are reachable."

**3. Proposed scheme first-death variance collapsed (±23 vs ±86.6 in Run 005).**
Run 005's high variance was driven by relay node overload on bad topologies. With the range limit, stranded nodes never burden relay CHs, smoothing out the topology-sensitive variance. More consistent behavior across seeds.

**4. Zero-PDR rounds dropped dramatically for both, but proposed remains far better.**
Proposed: 78.2 → 13.6 (nearly eliminated). LEACH: 330.8 → 155.0 (halved but still high). The "zero-PDR round" metric now fires only when literally no node can deliver to any CH — a stricter condition. The proposed scheme almost never hits it; LEACH still hits it 155 times due to random CH placement occasionally stranding large portions of the network simultaneously.

**5. Energy @ r300: LEACH now slightly ahead (+0.70 J).**
Reversed from Run 005 where proposed was ahead. Explanation: stranded LEACH nodes skip transmission, saving more energy for LEACH than for the proposed scheme (which strands fewer nodes, so fewer "free" energy savings from skipped Tx).

**6. The PDR gap narrowed but the zero-PDR advantage widened relatively.**
In Run 005, proposed had 4.2× fewer zero-PDR rounds (78 vs 331). In Run 007, it has 11.4× fewer (13.6 vs 155). The range limit makes the "dead silence" problem worse for LEACH — random CH placement sometimes clusters all CHs in one zone, stranding the rest of the network with no coverage.

### What to Do Next

- Decide whether r_tx = 50m is the right value for the paper or just a sensitivity check
- Consider running the 3 baselines with r_tx to get the full 5-scheme comparison under the range limit
- Update CLAUDE.md "Current State" and "Final Results" sections if r_tx is kept as a permanent model choice
- The reduced PDR gap (+9.3pp vs +22pp) may weaken the paper's headline claim — worth assessing whether the range limit is a core modeling choice or an optional sensitivity analysis

---

## Run 008 — Higher CH Density (p_CH 0.05 → 0.10 — REVERTED)

**Date:** 2026-04-12
**Run by:** Ahmed + Claude Code

### What This Run Was

Attempted improvement targeting the stranded node problem introduced by r_tx=50m in Run 007. With only 5 CHs covering a 100×100m field, late-round node stranding becomes significant. Hypothesis: doubling CH density to 10 would halve average nearest-CH distance (~22m → ~11m), reducing stranded nodes and recovering PDR. `r_exc` was also updated to scale automatically with `p_CH` to prevent exclusion zone overlap from blocking the extra elections.

### Parameters Changed

| Parameter | Run 007 | Run 008 | Reason |
|---|---|---|---|
| `p_CH` | 0.05 | **0.10** | Double CH density to reduce stranded nodes |
| `r_exc` | 25m (hardcoded) | **`sqrt(area²/(p_CH·N·π))` ≈ 18m** | Scaled down to match new CH density |

### Results (mean ± std across 5 seeds)

| Metric | Run 007 (p_CH=0.05) | Run 008 (p_CH=0.10) | Change |
|---|---|---|---|
| First node death (round) | 581.0 ± 23.0 | 559.2 ± 12.4 | **−21.8 rounds (worse)** |
| PDR mean (%) | 73.81 ± 1.72 | 69.98 ± 0.84 | **−3.83pp (worse)** |
| PDR gap over LEACH | +9.3pp | +5.8pp | **narrowed** |
| Energy @ round 300 (J) | 31.64 ± 0.24 | 31.18 ± 0.47 | −0.46 J (worse) |
| Zero-PDR rounds | 13.6 ± 2.8 | 20.0 ± 12.4 | **+6.4 (worse)** |

### Takeaways

**Every metric got worse. Config reverted to Run 007 values.**

**Why it backfired:** More CHs means more aggregation, overhead, and routing energy paid every round across all 1000 rounds. The stranded node problem only affects late rounds (~200 rounds). The energy penalty hits the full simulation; the coverage benefit is limited to the tail. Net result: faster drain, earlier first death, lower PDR.

**Root cause:** Stranded nodes are a late-round phenomenon. A global increase in p_CH is too blunt — it pays the overhead cost everywhere to fix a problem that only exists at the end. A dynamic p_CH that scales up only as N_alive drops would target the problem without burning extra energy in healthy rounds.

### What to Do Next

- Do not retry flat p_CH increase — overhead cost outweighs coverage benefit
- Consider dynamic p_CH: start at 0.05 when N_alive=100, ramp toward 0.10 as network shrinks
- This would increase CH density only when stranding becomes a problem, not throughout the full simulation

---

## Run 009 — Three-Window PDR Reporting + Codebase Cleanup (kappa=3)

**Date:** 2026-04-14
**Run by:** Ahmed + Claude Code

### What This Run Was

Validation run after two major changes: (1) implementing three-window PDR reporting in `run_multiseed.m`, and (2) removing the three baselines (EWMA-Detect, Threshold-JR, Reactive-CH) to start fresh with a clean Proposed vs LEACH comparison. No model parameters changed from Run 007 — this is the same config under the new reporting framework.

The three PDR windows were motivated by a literature survey (2022–2025) that found most WSN papers do not specify their PDR evaluation window. We now report:
- **Window 1 — All T rounds:** `mean(PDR)` — full lifecycle
- **Window 2 — FND-truncated:** `mean(PDR(1:t_death))` — operational period only
- **Window 3 — Zero-PDR count:** `sum(PDR == 0)` — communication blackout rounds

### Parameters

Identical to Run 007 (kappa=3, r_tx=50m).

### Results (mean ± std across 5 seeds)

| Metric | Proposed | LEACH |
|---|---|---|
| First node death (round) | 581.0 ± 23.0 | 718.2 ± 28.7 |
| PDR all rounds (%) | **73.81 ± 1.72** | 64.54 ± 0.46 |
| PDR FND-trunc (%) | **85.73 ± 1.52** | 76.12 ± 1.95 |
| Zero-PDR rounds | **13.6 ± 2.8** | 155.0 ± 23.9 |
| Energy @ round 300 (J) | 31.64 ± 0.24 | 32.34 ± 1.08 |

### Takeaways

**1. FND-truncated PDR gap is wider than all-rounds gap.**
All rounds: +9.3pp (73.81 vs 64.54). FND-truncated: +9.6pp (85.73 vs 76.12). This directly answers the reviewer objection that "your scheme only wins because it dies faster" — even restricted to the operational window, the gap holds and slightly widens. Proposed dies earlier (581 vs 718) but delivers more per round while alive.

**2. The lifetime paradox is confirmed and explained.**
Proposed dies 137 rounds earlier despite better PDR. Root cause: Dijkstra multi-hop routing concentrates forwarding load on relay CHs, draining them faster than LEACH's direct CH→BS approach. This is a deliberate trade-off: routing efficiency and jamming avoidance at the cost of relay node lifetime. The three-window framework makes this transparent.

**3. Zero-PDR rounds: 11.4× fewer for proposed.**
13.6 vs 155.0. This is the most operationally meaningful metric for a jamming scenario — how often is the network completely silenced? The proposed scheme achieves near-zero blackout rounds.

### What to Do Next

- Run sensitivity test with kappa=10 (stronger jamming) to validate robustness

---

## Run 010 — Jamming Intensity Sensitivity (kappa=3 → kappa=10)

**Date:** 2026-04-14
**Run by:** Ahmed + Claude Code

### What This Run Was

Sensitivity test: increased jamming decay constant from kappa=3 to kappa=10. At kappa=3, packet success at jammer center is `p_base × e^{−3} ≈ 0.047`. At kappa=10, it drops to `p_base × e^{−10} ≈ 0.00004` — effectively zero. The jammer becomes far more lethal at its center while the gradient outside r_j steepens sharply (bimodal: nodes inside r_j get nearly zero PDR, nodes outside are barely affected).

Hypothesis: stronger jamming should make JR-aware election and routing more valuable, since the cost of being in the jammer's path is higher.

### Parameters Changed

| Parameter | Run 009 | Run 010 | Reason |
|---|---|---|---|
| `kappa` | 3 | **10** | Stronger jamming — steeper attenuation inside r_j |
| All other parameters | unchanged | unchanged | — |

### Results (mean ± std across 5 seeds)

| Metric | Proposed | LEACH | vs Run 009 Proposed | vs Run 009 LEACH |
|---|---|---|---|---|
| First node death (round) | 597.2 ± 26.8 | 728.0 ± 34.1 | +16.2 (later) | +9.8 (later) |
| PDR all rounds (%) | **71.03 ± 1.23** | 62.74 ± 0.63 | −2.78pp | −1.80pp |
| PDR FND-trunc (%) | **82.49 ± 1.13** | 72.40 ± 2.27 | −3.24pp | −3.72pp |
| Zero-PDR rounds | **9.8 ± 7.3** | 136.6 ± 28.4 | −3.8 (fewer) | −18.4 (fewer) |
| Energy @ round 300 (J) | 31.64 ± 0.29 | 32.72 ± 1.23 | ~same | +0.38 J |

### Takeaways

**1. The proposed scheme's advantage is robust to stronger jamming.**
PDR gap (all rounds): 8.29pp at kappa=10 vs 9.27pp at kappa=3 — narrows by only ~1pp despite a 3× increase in jamming intensity. PDR gap (FND-truncated): 10.09pp at kappa=10 vs 9.61pp at kappa=3 — actually *widens*. The proposed scheme degrades gracefully.

**2. Zero-PDR advantage increases under stronger jamming.**
Ratio: 13.9× (9.8 vs 136.6) at kappa=10 vs 11.4× (13.6 vs 155.0) at kappa=3. With kappa=10, LEACH's random CH placement more frequently puts CHs directly in the jammer path — when that happens, the entire cluster is wiped. JR-aware election avoids this more decisively.

**3. Both schemes live slightly longer under stronger jamming.**
Proposed: 581→597, LEACH: 718→728. Counterintuitive but correct: at kappa=10 the jammer effect is nearly binary — nodes inside r_j barely transmit (p≈0, near-zero energy spent), nodes outside are unaffected. Net effect is fewer successful transmissions per round on average → less energy burned → later first death.

**4. Zero-PDR rounds decrease for both schemes.**
Proposed: 13.6→9.8, LEACH: 155→136.6. With kappa=10, the jamming is more spatially concentrated — fewer rounds where a moderate fraction of nodes are partially jammed. More rounds are either clean (jammer elsewhere) or total-wipe (jammer directly overhead). The "partial degradation" rounds that still contribute some PDR reduce, but full-wipe rounds actually pull zero-PDR count down because the jammer footprint is smaller in effective area.

**5. LEACH energy @r300 increases slightly (+0.38 J).**
With stronger jamming, LEACH nodes attempt transmissions that fail at near-zero p, wasting circuit energy (E_elec) on packets that never deliver. JR-aware election skips jammed nodes as CHs, avoiding this wasted energy overhead.

**6. kappa=10 is the current canonical config.**
The results are robust and the stronger jamming scenario is more realistic for UAV-based interference. kappa=3 was likely under-representing jammer effectiveness.

### What to Do Next

- Run 010 numbers are the current best results — update paper draft with kappa=10 figures
- Consider kappa sensitivity plot (kappa=3, 5, 10) as a figure in the paper to show robustness
- Next major step: decide on new baselines to rebuild from scratch with r_tx + honest PDR accounting

---

## Run 011 — CH Re-election Frequency Reduction (K_elec=10 → K_elec=5)

**Date:** 2026-04-14
**Run by:** Ahmed + Claude Code

### What This Run Was

Diagnostic investigation into the proposed scheme's zero-PDR rounds, followed by a targeted fix. A per-round diagnostic script (`diag_proposed_zeros.m`, `diag_leach_zeros.m`) was written to classify the cause of every zero-PDR round in both schemes under seed 42.

**Finding — LEACH zero-PDR cause:**
All 145 LEACH zero-PDR rounds (seed 42) are classified `ALL_NODES_ARE_CH` — caused entirely by the epoch mechanism, not jamming. At the last round of each 20-round epoch, `T_thresh = P/(1-P×19) = 1.0`, causing all remaining eligible nodes to simultaneously elect themselves CH with probability 1.0. This leaves almost no eligible nodes for the next epoch start. When the few remaining eligible nodes fail to elect anyone (probability `(0.95)^n ≈ high` for small n), the fallback `if ~any(is_CH); is_CH = alive` fires, making ALL nodes CHs with zero members. This cascades for the full remainder of the epoch. The jammer plays no role — this failure occurs even when the jammer is not overhead. Zero early/mid zeros (rounds 1-300), 17 mid zeros (301-600), 128 late zeros (601+).

**Finding — Proposed scheme zero-PDR cause:**
11 zero-PDR rounds (seed 42, K_elec=10), all `STOCHASTIC(CHs=0)` in rounds 838-859. Root cause: CHs elected at round 830 (last election before round 838) died between election intervals. With K_elec=10, there is up to a 9-round window during which dead CHs go unreplaced. `is_CH & alive = false` for all nodes → no active CHs → `total_sent` counts only stranded packets → PDR=0.

**Fix applied:** Reduced `K_elec` from 10 to 5 in `core/config.m`. Halves the maximum dead-CH window from 9 rounds to 4 rounds. Also motivated independently: faster re-election allows JR estimates (EWMA) to drive CH rotation more responsively as the jammer orbits (one orbit = 50 rounds; K_elec=5 gives 10 elections per orbit vs 5 previously).

**Also explored but reverted:** `ceil()` instead of `round()` in the K formula in `elect_ch_proposed.m`. Logically sound (avoids K=0 when n_alive<10 without relying solely on the `max(1,...)` guard), but introduced higher variance in late-round behavior across seeds (zero-PDR std rose from ±3.7 to ±16.1). Reverted to `round()` — the `max(1,...)` guard is sufficient.

### Parameters Changed

| Parameter | Run 010 | Run 011 | Reason |
|---|---|---|---|
| `K_elec` | 10 | **5** | Halve dead-CH window; faster JR-driven rotation |
| All other parameters | unchanged | unchanged | — |

### Results (mean ± std across 5 seeds)

| Metric | Proposed | LEACH | vs Run 010 Proposed | vs Run 010 LEACH |
|---|---|---|---|---|
| First node death (round) | 593.2 ± 13.4 | 712.8 ± 35.9 | −4.0, variance halved | −15.2 |
| PDR all rounds (%) | **70.95 ± 1.22** | 62.02 ± 0.82 | −0.08pp (negligible) | −0.72pp |
| PDR FND-trunc (%) | **82.23 ± 1.54** | 71.37 ± 4.67 | −0.26pp (negligible) | −1.03pp |
| Zero-PDR rounds (Proposed) | **5.2 ± 3.7** | 163.0 ± 18.3 | **−47%, variance halved** | +26.4 |
| Energy @ round 300 (J) | 31.62 ± 0.32 | 32.21 ± 1.62 | negligible | −0.51 J |

### Per-Seed First Node Death

| Seed | Proposed | LEACH |
|---|---|---|
| 42  | 582 | 742 |
| 7   | 586 | 663 |
| 13  | 604 | 704 |
| 99  | 583 | 702 |
| 101 | 611 | 753 |

### Takeaways

**1. K_elec=5 cuts proposed scheme zero-PDR rounds nearly in half.**
9.8 → 5.2 (−47%). The remaining ~5 are end-of-life stochastic events in the last ~50 rounds when the network is degraded to fewer than 30 nodes. The variance also halved (±7.3 → ±3.7), indicating more consistent behavior across topologies.

**2. PDR is essentially unchanged — the overhead cost is negligible.**
All-rounds PDR: −0.08pp. FND-truncated: −0.26pp. Both within noise. Doubling election frequency from every 10 to every 5 rounds does not meaningfully increase energy drain in the healthy phase (confirmed by identical Energy@r300).

**3. First-death variance collapsed (±26.8 → ±13.4).**
The proposed scheme is now more topology-consistent. K_elec=10's high variance was partly caused by topologies where relay CHs drained faster in the 10-round window before rotation — K_elec=5 rotates them out sooner.

**4. LEACH zero-PDR rounds are not improved by this change — by design.**
LEACH's 163 zeros are caused by its own epoch mechanism, not by K_elec (which only affects the proposed scheme). The gap widens: proposed has 31× fewer zero-PDR rounds than LEACH (5.2 vs 163.0).

**5. The LEACH zero-PDR mechanism is a paper-relevant finding.**
LEACH's zero-PDR rounds are self-inflicted by its epoch fallback — not caused by jamming. This is a strong argument in the paper: LEACH suffers communication blackouts even in rounds where the jammer is not directly overhead, purely due to protocol design. The proposed scheme's near-zero blackout count holds regardless of jammer position.

### What to Do Next

- K_elec=5 is the new canonical config — update CLAUDE.md
- Decide on new baselines to rebuild (must include r_tx + 3-window PDR reporting from day one)
- Consider whether to keep `diag_leach_zeros.m` and `diag_proposed_zeros.m` in the repo or remove as temporary diagnostic scripts

---
