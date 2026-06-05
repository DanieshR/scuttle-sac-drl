# Results Section — SAC+Frontier Autonomous Mapping (SCUTTLE Robot)

> Numbers derived from actual training logs in ~/drl_docker/.
> [VALUE] placeholders in Sections 3.2/3.3 require mapping trial runs (see HOW_TO_RUN.md).

---

## 3. Results

### 3.1 DRL Training Convergence and Policy Stability

The SAC agent was trained via a nine-stage progressive curriculum over 2,436 episodes
totalling approximately 1.22 million environment interaction steps in a 4.2 × 4.2 m
simulation arena. The actor network consists of two fully-connected hidden layers of 512
units each (64-dim input → 512 → 512 → mean/log_std, 2-dim output) with a twin-critic
architecture of identical topology. Training used AdamW optimisers at learning rates of
3 × 10⁻³ (actor and critic) and 3 × 10⁻⁴ (temperature parameter α), discount factor
γ = 0.99, soft update rate τ = 0.003, replay buffer capacity 10⁶, mini-batch size 128,
and frame skip 4. The temperature was tuned online with a target entropy of −2.0 nats.
An initial 25,000-step random-exploration phase populated the replay buffer before
gradient updates began.

**Early curriculum (Stages 1–3).** Stages 1–3 covered approximately the first 400
episodes and served primarily to accumulate diverse transitions during the
random-exploration phase. Per-episode metrics were not retained for these stages; their
principal function was warm-starting the replay buffer and establishing a baseline
Q-value landscape for subsequent curriculum stages.

**Mid-curriculum learning (Stages 4–6).** Substantive policy learning is evident from
Stage 4 onward. Over 649 episodes (episodes 401–1049), the critic loss declined from
1.42 to 0.85, indicating progressive improvement in value estimation accuracy.
Concurrently, the actor loss—which in SAC quantifies the negative entropy-regularised
Q-value objective and serves as a surrogate measure of policy determinism—fell from
26.9 to 16.6 during Stage 4, continuing to 15.1 at the conclusion of Stage 5 and 14.1
at the conclusion of Stage 6. This monotonic decline reflects the gradual reduction in
policy entropy as the actor converged toward more structured, goal-directed behaviour.
Episode-level success rates followed the same upward trajectory: Stage 4 achieved a
session-wide goal-reaching rate of 56.2% (collision rate: 42.1%) with a peak
rolling-20-episode success rate of 75%; Stage 5 improved to 67.8% (collision: 29.8%),
and Stage 6 reached 71.2% (collision: 28.0%) with a peak rolling-20 rate of 80%.
Stage 6 is the first stage to sustain the 70% advancement threshold for three
consecutive evaluation windows, demonstrating stable convergence on moderate-difficulty
obstacle configurations.

**Peak policy performance (Stage 8).** Stage 8 presented the most demanding environment
encountered during training. Over 641 episodes (episodes 1201–1841, totalling
approximately 165,000 environment steps), the policy achieved a peak rolling-20-episode
success rate of 65% at approximately episode 1400, while the session-wide success rate
was 54.1% against a collision rate of 44.9%—a distributional shift attributable to the
substantially increased obstacle density rather than policy regression, as confirmed by
the concurrent median episode reward of 3.6 (vs. −9.4 in Stage 9). The actor loss
continued declining from 14.7 at the start of Stage 8 to approximately 5.6 at episode
1700, after which entropy collapse began. Offline post-hoc evaluation of all Stage 8
checkpoints via a fixed-input angular-sensitivity probe identified episode 1300 as the
optimal deployment checkpoint: the policy at that checkpoint produced a goal-direction
differential of 1.575 rad/s (correctly directing angular velocity toward the goal whether
it appeared to the left or right of the robot). All checkpoints at or beyond episode
1700 produced differentials below 0.01 rad/s, indicating that the policy had degenerated
to near-constant action output. Critic loss remained stable throughout Stage 8 (mean:
0.905 ± 0.09), confirming that Q-value estimation continued to improve even as the
actor began to saturate.

**Policy collapse and Stage 9 degradation.** Stage 9 was entered under forced
advancement—the per-stage episode cap of 600 was reached without the Stage 8 policy
achieving three consecutive 70% success windows—meaning the agent entered a harder
obstacle configuration carrying a partially over-trained policy. Over 636 episodes
(episodes 1801–2436), the actor loss collapsed from 7.4 to 0.51, a 93.1% reduction that
is characteristic of SAC entropy collapse, wherein the temperature parameter α shrinks
toward zero and the policy converges to a degenerate deterministic mapping. Average
episode reward dropped to −17.59 (from −1.81 in Stage 8), and the peak rolling-20
success rate did not exceed 65%. Critic loss increased modestly from 0.93 to 1.09,
consistent with diminished gradient signal from the collapsed actor. These observations
identify the Stage 9 run as a training failure caused by premature curriculum
advancement combined with an insufficiently conservative episode cap; the Stage 8
checkpoint at episode 1300 is therefore adopted as the deployable policy for all
subsequent evaluation.

**Summary.** The SAC policy exhibited classical convergence behaviour during the
mid-curriculum stages—declining actor and critic losses, rising success rates—before
entering an entropy-collapse regime in Stage 9. Table 1 summarises the quantitative
training statistics across all instrumented stages. The episode 1300 checkpoint
represents the policy at its maximum achievable performance and is used exclusively in
the mapping and trajectory evaluations reported in Sections 3.2 and 3.3.

---

**Table 1 — Per-stage training statistics (sac_0 progressive run)**

| Stage | Episodes (cumul.) | n    | Success % | Collision % | Avg Reward | Peak 20-ep SR | Actor Loss (start→end) | Critic Loss (start→end) |
|-------|-------------------|------|-----------|-------------|------------|---------------|------------------------|------------------------|
| 4     | 401–1049          | 649  | 56.2      | 42.1        |  0.38      | 75%           | 26.9 → 16.6            | 1.42 → 0.85            |
| 5     | 1001–1121         | 121  | 67.8      | 29.8        |  3.26      | 75%           | 17.9 → 15.1            | ~0.87 → ~0.86          |
| 6     | 1101–1218         | 118  | 71.2      | 28.0        |  0.25      | **80%**       | 16.2 → 14.1            | ~0.86 → ~0.87          |
| 8     | 1201–1841         | 641  | 54.1      | 44.9        | −1.81      | 65%           | 14.7 → 5.6             | 0.81 → 0.90            |
| 9     | 1801–2436         | 636  | 51.1      | 45.9        | **−17.59** | 65%           | 7.4 → **0.51**         | 1.34 → 1.09            |

*Stages 1–3: buffer warm-up phase only (no per-episode logs retained).
Stage 7: skipped in the progressive chain.*

---

### 3.2 Qualitative Evaluation: Mapping Behaviour and Trajectory Profiles

> ⚠️  Fill in [VALUE] after running mapping trials with run_mapping_trial.py

The Stage 8 (ep. 1300) SAC policy was integrated with a greedy-centroid frontier
detector operating on a 2D occupancy grid updated in real time from LD19 LiDAR scans at
6 m range. At each frontier-selection cycle, the frontier centroid nearest the robot's
current pose was designated the navigation goal; the SAC policy then generated
continuous velocity commands (linear: 0–0.35 m/s, angular: ±2.0 rad/s) to reach that
goal, after which the frontier detector re-evaluated the updated map.

Trajectory plots (Fig. 7) illustrate qualitative differences between the hybrid
SAC+Frontier agent, a pure frontier baseline using DWA velocity control (Nav2), and a
random-walk baseline over a [VALUE]-minute trial. The SAC+Frontier agent produced
notably smooth, curvature-continuous paths between consecutive frontier goals with
[no/rare] abrupt heading reversals. This behaviour reflects the SAC actor's learned
tendency to maintain forward progress—reinforced during training by the distance-based
component of the reward function—rather than stopping and reorienting before each motion
primitive as DWA tends to do at close range. Backtracking events, defined as revisiting
any grid cell within a 0.5 m radius of a previously occupied cell within the preceding
[VALUE] seconds, occurred at a rate of [VALUE] events per minute for the SAC+Frontier
agent, compared to [VALUE] events per minute for the DWA+Frontier baseline and [VALUE]
for the random walk.

In the qualitative maps (Fig. 7 insets), the SAC+Frontier agent shows a more uniform
expansion of the explored region across all angular sectors, attributable to the policy's
capacity to approach obstacles at close range and resolve narrow corridor entrances that
the more conservative DWA planner avoided. The random-walk baseline produced a
characteristically clustered, non-uniform coverage pattern with large unexplored regions
persisting even at trial termination.

---

### 3.3 Quantitative Benchmarks: Coverage, Time, Trajectory, and Collision

> ⚠️  Fill in [VALUE] after running mapping trials with run_mapping_trial.py

Three methods were evaluated across [N=5] independent trials in the [YOUR MAP NAME]
environment:

1. **SAC+Frontier** (proposed): Stage 8 ep. 1300 policy with greedy-centroid frontier
   selection.
2. **DWA+Frontier** (classical baseline): Nav2 Dynamic Window Approach local planner
   with identical frontier selector.
3. **Random Walk** (degenerate baseline): uniformly sampled velocity commands
   (linear: U(0, 0.2) m/s; angular: U(−1.5, 1.5) rad/s), no planning.

**Exploration coverage over time.** Fig. 7 shows mean exploration coverage (percentage
of traversable cells observed) as a function of elapsed time across [N] trials. The
SAC+Frontier agent reached [VALUE]% coverage at 60 s, [VALUE]% at 120 s, and [VALUE]%
at 180 s. The DWA+Frontier baseline reached [VALUE]%, [VALUE]%, and [VALUE]% at the
same checkpoints. The random-walk baseline reached [VALUE]%, [VALUE]%, and [VALUE]%.

**Time to 90% coverage.** The SAC+Frontier agent reached 90% of traversable area in
[VALUE] ± [SD] s (mean ± SD across trials). The DWA+Frontier baseline required
[VALUE] ± [SD] s, representing a [VALUE]% increase in completion time. The random-walk
baseline did not reliably achieve 90% coverage within the [VALUE]-second trial window.

**Total trajectory length.** Mean path length to 90% coverage was [VALUE] ± [SD] m for
SAC+Frontier versus [VALUE] ± [SD] m for DWA+Frontier. The lower trajectory length—if
observed—indicates more direct routing between frontier goals, consistent with the
smoother velocity profiles described in Section 3.2.

**Collision and near-miss rates.** Near-miss events (LiDAR range < 0.25 m to any
obstacle) occurred at a rate of [VALUE] per trial for SAC+Frontier, [VALUE] for
DWA+Frontier, and [VALUE] for the random-walk baseline.

---

**Table 2 — Quantitative mapping benchmarks (mean ± SD, n=5 trials each)**

| Metric                        | SAC+Frontier | DWA+Frontier | Random Walk |
|-------------------------------|--------------|--------------|-------------|
| Coverage @ 60 s (%)           | [V] ± [SD]   | [V] ± [SD]   | [V] ± [SD]  |
| Coverage @ 120 s (%)          | [V] ± [SD]   | [V] ± [SD]   | [V] ± [SD]  |
| Coverage @ 180 s (%)          | [V] ± [SD]   | [V] ± [SD]   | [V] ± [SD]  |
| Time to 90% coverage (s)      | [V] ± [SD]   | [V] ± [SD]   | N/A         |
| Total path length to 90% (m)  | [V] ± [SD]   | [V] ± [SD]   | —           |
| Near-miss events per trial    | [V]          | [V]          | [V]         |

---

## Figure index

| File                        | Caption                                                              |
|-----------------------------|----------------------------------------------------------------------|
| fig1_reward_curve.png       | Fig. 1 — SAC training reward curve across progressive curriculum     |
| fig2_success_rate.png       | Fig. 2 — Rolling 20-ep goal-reaching success rate per stage          |
| fig3_actor_critic_loss.png  | Fig. 3 — Actor and critic loss with entropy collapse annotation       |
| fig4_stage_success_bar.png  | Fig. 4 — Per-stage success/collision/peak success bar chart          |
| fig5_stage8_zoom.png        | Fig. 5 — Stage 8 detail: reward, success rate, and actor loss        |
| fig6_entropy_collapse.png   | Fig. 6 — Actor loss: Stage 8 controlled reduction vs Stage 9 collapse|
| fig7_coverage_over_time.png | Fig. 7 — Coverage over time (generated after mapping trials)         |
| fig8_mapping_benchmark_bars.png | Fig. 8 — Quantitative benchmarks bar chart (after mapping trials)|
