"""
Generate all training convergence figures for the SAC+Frontier paper.
Reads per-episode logs from drl_docker, saves PNGs to ~/Desktop/results/
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# ── paths ──────────────────────────────────────────────────────────────────
BASE = os.path.expanduser(
    "~/drl_docker/src/turtlebot3_drl/model/daniesh-Legion-Pro-5-16ARX8"
)
OUT = os.path.expanduser("~/Desktop/results")
os.makedirs(OUT, exist_ok=True)

STAGE_FILES = {
    4: f"{BASE}/sac_0_stage_4/_train_stage4_20260531-134559.txt",
    5: f"{BASE}/sac_0_stage_5/_train_stage5_20260531-145145.txt",
    6: f"{BASE}/sac_0_stage_6/_train_stage6_20260531-150428.txt",
    8: f"{BASE}/sac_0_stage_8/_train_stage8_20260531-151921.txt",
    9: f"{BASE}/sac_0_stage_9/_train_stage9_20260531-161605.txt",
}

STAGE_COLORS = {4: "#4C72B0", 5: "#55A868", 6: "#C44E52", 8: "#8172B2", 9: "#CCB974"}
STAGE_LABELS = {4: "Stage 4", 5: "Stage 5", 6: "Stage 6",
                8: "Stage 8 (peak)", 9: "Stage 9 (collapse)"}

# ── loader ─────────────────────────────────────────────────────────────────
def load_stage(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("episode"):
                continue
            p = [x.strip() for x in line.split(",")]
            if len(p) < 9:
                continue
            try:
                rows.append({
                    "ep":      int(p[0]),
                    "reward":  float(p[1]),
                    "success": int(p[2]),
                    "steps":   int(p[4]),
                    "total":   int(p[5]),
                    "critic":  float(p[7]),
                    "actor":   float(p[8]),
                })
            except (ValueError, IndexError):
                continue
    return rows


def rolling(arr, w):
    return np.convolve(arr, np.ones(w) / w, mode="valid")


all_data = {s: load_stage(p) for s, p in STAGE_FILES.items()}

# ── shared style ────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "figure.dpi": 150,
})

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 1 – Reward curve (full training timeline, rolling mean)
# ═══════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(12, 4.5))

WINDOW = 50
for stage, rows in all_data.items():
    eps     = [r["ep"] for r in rows]
    rewards = [r["reward"] for r in rows]
    ax.plot(eps, rewards, alpha=0.12, color=STAGE_COLORS[stage], linewidth=0.6)
    if len(rewards) >= WINDOW:
        rm = rolling(rewards, WINDOW)
        ax.plot(eps[WINDOW-1:], rm, color=STAGE_COLORS[stage],
                linewidth=2.0, label=STAGE_LABELS[stage])

# mark best checkpoint
ax.axvline(1300, color="red", linestyle="--", linewidth=1.4, alpha=0.8)
ax.text(1305, ax.get_ylim()[1]*0.88, "Best ckpt\n(ep 1300)", color="red",
        fontsize=9, va="top")

# stage boundary shading
boundaries = [(401, 1049, 4), (1001, 1121, 5), (1101, 1218, 6),
              (1201, 1841, 8), (1801, 2436, 9)]
for s_ep, e_ep, stage in boundaries:
    ax.axvspan(s_ep, e_ep, alpha=0.04, color=STAGE_COLORS[stage])

ax.set_xlabel("Cumulative episode")
ax.set_ylabel("Episode reward")
ax.set_title("Fig. 1 — SAC Training Reward Curve (progressive curriculum, 9 stages)")
ax.legend(loc="lower left", fontsize=9, framealpha=0.7)
fig.tight_layout()
fig.savefig(f"{OUT}/fig1_reward_curve.png")
plt.close(fig)
print("Saved fig1_reward_curve.png")

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 2 – Rolling 20-ep success rate (goal-reaching %)
# ═══════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(12, 4.5))

WIN_SR = 20
for stage, rows in all_data.items():
    eps     = [r["ep"] for r in rows]
    success = [1 if r["success"] == 1 else 0 for r in rows]
    rm      = rolling(success, WIN_SR)
    ax.plot(eps[WIN_SR-1:], rm * 100, color=STAGE_COLORS[stage],
            linewidth=2.0, label=STAGE_LABELS[stage])

ax.axhline(70, color="grey", linestyle=":", linewidth=1.2, alpha=0.8)
ax.text(415, 71, "Advancement threshold (70%)", color="grey", fontsize=8)
ax.axvline(1300, color="red", linestyle="--", linewidth=1.4, alpha=0.8)

for s_ep, e_ep, stage in boundaries:
    ax.axvspan(s_ep, e_ep, alpha=0.04, color=STAGE_COLORS[stage])

ax.set_ylim(0, 100)
ax.set_xlabel("Cumulative episode")
ax.set_ylabel("Rolling 20-ep success rate (%)")
ax.set_title("Fig. 2 — Goal-reaching success rate across the progressive curriculum")
ax.legend(loc="lower left", fontsize=9, framealpha=0.7)
fig.tight_layout()
fig.savefig(f"{OUT}/fig2_success_rate.png")
plt.close(fig)
print("Saved fig2_success_rate.png")

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 3 – Actor loss and critic loss (dual panel, policy entropy narrative)
# ═══════════════════════════════════════════════════════════════════════════
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

WIN_LOSS = 50
for stage, rows in all_data.items():
    eps    = [r["ep"] for r in rows]
    actor  = [r["actor"] for r in rows]
    critic = [r["critic"] for r in rows]

    # raw (very transparent)
    ax1.plot(eps, actor,  alpha=0.08, color=STAGE_COLORS[stage], linewidth=0.5)
    ax2.plot(eps, critic, alpha=0.08, color=STAGE_COLORS[stage], linewidth=0.5)

    if len(actor) >= WIN_LOSS:
        ax1.plot(eps[WIN_LOSS-1:], rolling(actor,  WIN_LOSS),
                 color=STAGE_COLORS[stage], linewidth=2.0, label=STAGE_LABELS[stage])
        ax2.plot(eps[WIN_LOSS-1:], rolling(critic, WIN_LOSS),
                 color=STAGE_COLORS[stage], linewidth=2.0)

for axx in (ax1, ax2):
    axx.axvline(1300, color="red", linestyle="--", linewidth=1.4, alpha=0.8)
    for s_ep, e_ep, stage in boundaries:
        axx.axvspan(s_ep, e_ep, alpha=0.04, color=STAGE_COLORS[stage])

ax1.set_ylabel("Actor loss (entropy-reg. Q-objective)")
ax1.set_title("Fig. 3 — Actor and Critic Loss: policy entropy collapse at Stage 9")
ax1.legend(loc="upper right", fontsize=9, framealpha=0.7)

# annotate collapse
ax1.annotate("Entropy\ncollapse", xy=(2100, 2.5), xytext=(1950, 8),
             arrowprops=dict(arrowstyle="->", color="black", lw=1.2),
             fontsize=9, color="black")

ax2.set_xlabel("Cumulative episode")
ax2.set_ylabel("Critic loss (Huber/smooth-L1)")

fig.tight_layout()
fig.savefig(f"{OUT}/fig3_actor_critic_loss.png")
plt.close(fig)
print("Saved fig3_actor_critic_loss.png")

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 4 – Per-stage bar chart: success rate & collision rate
# ═══════════════════════════════════════════════════════════════════════════
stages_ord = [4, 5, 6, 8, 9]
sr   = []
cr   = []
peak = []

for s in stages_ord:
    rows = all_data[s]
    n  = len(rows)
    sc = sum(1 for r in rows if r["success"] == 1)
    co = sum(1 for r in rows if r["success"] == 2)
    sr.append(100 * sc / n)
    cr.append(100 * co / n)
    # peak rolling-20
    success_bin = [1 if r["success"] == 1 else 0 for r in rows]
    best = max(sum(success_bin[i:i+20]) for i in range(len(success_bin)-19)) if len(success_bin) >= 20 else 0
    peak.append(100 * best / 20)

x     = np.arange(len(stages_ord))
width = 0.28

fig, ax = plt.subplots(figsize=(9, 5))
b1 = ax.bar(x - width, sr,   width, label="Session-wide success rate",  color="#4C72B0", alpha=0.85)
b2 = ax.bar(x,          peak, width, label="Peak rolling-20 success rate", color="#55A868", alpha=0.85)
b3 = ax.bar(x + width, cr,   width, label="Collision rate",               color="#C44E52", alpha=0.85)

ax.axhline(70, color="grey", linestyle=":", linewidth=1.2, alpha=0.7,
           label="Advancement threshold (70%)")

for bar_grp in (b1, b2, b3):
    for bar in bar_grp:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + 0.8,
                f"{h:.0f}%", ha="center", va="bottom", fontsize=8)

ax.set_xticks(x)
ax.set_xticklabels([f"Stage {s}" for s in stages_ord])
ax.set_ylabel("Rate (%)")
ax.set_ylim(0, 95)
ax.set_title("Fig. 4 — Per-stage success and collision rates across the curriculum")
ax.legend(fontsize=9, framealpha=0.7, loc="upper right")
fig.tight_layout()
fig.savefig(f"{OUT}/fig4_stage_success_bar.png")
plt.close(fig)
print("Saved fig4_stage_success_bar.png")

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 5 – Stage 8 zoom: reward + rolling success + actor loss (3-panel)
# ═══════════════════════════════════════════════════════════════════════════
rows8  = all_data[8]
eps8   = [r["ep"] for r in rows8]
rew8   = [r["reward"] for r in rows8]
suc8   = [1 if r["success"] == 1 else 0 for r in rows8]
act8   = [r["actor"] for r in rows8]
crit8  = [r["critic"] for r in rows8]

fig = plt.figure(figsize=(12, 9))
gs  = GridSpec(3, 1, figure=fig, hspace=0.35)
ax_r = fig.add_subplot(gs[0])
ax_s = fig.add_subplot(gs[1], sharex=ax_r)
ax_a = fig.add_subplot(gs[2], sharex=ax_r)

WIN8 = 50
# reward
ax_r.plot(eps8, rew8, alpha=0.12, color=STAGE_COLORS[8], linewidth=0.5)
if len(rew8) >= WIN8:
    ax_r.plot(eps8[WIN8-1:], rolling(rew8, WIN8),
              color=STAGE_COLORS[8], linewidth=2.0)
ax_r.set_ylabel("Episode reward")
ax_r.set_title("Fig. 5 — Stage 8 detail: reward, success rate, and actor loss")

# success
rm_s = rolling(suc8, 20)
ax_s.plot(eps8[19:], rm_s * 100, color=STAGE_COLORS[8], linewidth=2.0)
ax_s.axhline(70, color="grey", linestyle=":", linewidth=1.0, alpha=0.8)
ax_s.set_ylabel("Rolling 20-ep\nsuccess rate (%)")
ax_s.set_ylim(0, 100)

# actor loss
ax_a.plot(eps8, act8, alpha=0.12, color="#8172B2", linewidth=0.5)
if len(act8) >= WIN8:
    ax_a.plot(eps8[WIN8-1:], rolling(act8, WIN8), color="#8172B2", linewidth=2.0,
              label="Actor loss")
ax_a.set_ylabel("Actor loss")
ax_a.set_xlabel("Cumulative episode")

# mark best checkpoint and collapse onset
for axx in (ax_r, ax_s, ax_a):
    axx.axvline(1300, color="red", linestyle="--", linewidth=1.4, alpha=0.9)
    axx.axvline(1700, color="orange", linestyle="--", linewidth=1.2, alpha=0.8)

ax_r.text(1302, ax_r.get_ylim()[1]*0.9, "Best ckpt\nep 1300",
          color="red", fontsize=8, va="top")
ax_r.text(1702, ax_r.get_ylim()[1]*0.9, "Collapse\nonset",
          color="orange", fontsize=8, va="top")

fig.savefig(f"{OUT}/fig5_stage8_zoom.png")
plt.close(fig)
print("Saved fig5_stage8_zoom.png")

# ═══════════════════════════════════════════════════════════════════════════
# FIGURE 6 – Actor loss collapse: Stage 8 vs Stage 9 side-by-side
# ═══════════════════════════════════════════════════════════════════════════
rows9 = all_data[9]
eps9  = [r["ep"] for r in rows9]
act9  = [r["actor"] for r in rows9]

fig, (ax_l, ax_r_) = plt.subplots(1, 2, figsize=(12, 4.5))

WIN_AL = 30
ax_l.plot(eps8, act8, alpha=0.15, color=STAGE_COLORS[8], linewidth=0.5)
if len(act8) >= WIN_AL:
    ax_l.plot(eps8[WIN_AL-1:], rolling(act8, WIN_AL),
              color=STAGE_COLORS[8], linewidth=2.2, label="Stage 8")
ax_l.axvline(1300, color="red", linestyle="--", linewidth=1.3)
ax_l.axvline(1700, color="orange", linestyle="--", linewidth=1.2)
ax_l.set_xlabel("Episode"); ax_l.set_ylabel("Actor loss")
ax_l.set_title("Stage 8: gradual entropy reduction")
ax_l.legend(fontsize=9)
ax_l.text(1302, max(act8)*0.92, "Best ckpt", color="red", fontsize=8)
ax_l.text(1702, max(act8)*0.92, "Onset", color="orange", fontsize=8)

ax_r_.plot(eps9, act9, alpha=0.15, color=STAGE_COLORS[9], linewidth=0.5)
if len(act9) >= WIN_AL:
    ax_r_.plot(eps9[WIN_AL-1:], rolling(act9, WIN_AL),
               color=STAGE_COLORS[9], linewidth=2.2, label="Stage 9")
ax_r_.set_xlabel("Episode")
ax_r_.set_title("Stage 9: catastrophic entropy collapse → 0.51")
ax_r_.legend(fontsize=9)

# shared y range for visual comparison
y_max = max(max(act8), max(act9)) * 1.05
for axx in (ax_l, ax_r_):
    axx.set_ylim(0, y_max)

fig.suptitle("Fig. 6 — Actor loss: controlled reduction (Stage 8) vs. entropy collapse (Stage 9)",
             fontsize=11)
fig.tight_layout()
fig.savefig(f"{OUT}/fig6_entropy_collapse.png")
plt.close(fig)
print("Saved fig6_entropy_collapse.png")

# ═══════════════════════════════════════════════════════════════════════════
# Save numeric summary CSV
# ═══════════════════════════════════════════════════════════════════════════
import csv

summary_rows = []
for stage in stages_ord:
    rows = all_data[stage]
    n  = len(rows)
    sc = sum(1 for r in rows if r["success"] == 1)
    co = sum(1 for r in rows if r["success"] == 2)
    to = sum(1 for r in rows if r["success"] == 3)
    rewards = [r["reward"] for r in rows]
    actors  = [r["actor"]  for r in rows]
    critics = [r["critic"] for r in rows]
    suc_bin = [1 if r["success"] == 1 else 0 for r in rows]
    peak20  = max(sum(suc_bin[i:i+20]) for i in range(len(suc_bin)-19)) if len(suc_bin) >= 20 else 0
    summary_rows.append({
        "stage":              stage,
        "ep_start":           rows[0]["ep"],
        "ep_end":             rows[-1]["ep"],
        "n_episodes":         n,
        "total_steps_start":  rows[0]["total"],
        "total_steps_end":    rows[-1]["total"],
        "success_rate_pct":   round(100*sc/n, 2),
        "collision_rate_pct": round(100*co/n, 2),
        "timeout_rate_pct":   round(100*to/n, 2),
        "avg_reward":         round(np.mean(rewards), 3),
        "median_reward":      round(float(np.median(rewards)), 3),
        "max_reward":         round(max(rewards), 3),
        "min_reward":         round(min(rewards), 3),
        "peak_rolling20_sr":  round(100*peak20/20, 1),
        "actor_loss_start":   round(actors[0], 4),
        "actor_loss_end":     round(actors[-1], 4),
        "actor_loss_mean":    round(np.mean(actors), 4),
        "critic_loss_start":  round(critics[0], 4),
        "critic_loss_end":    round(critics[-1], 4),
        "critic_loss_mean":   round(np.mean(critics), 4),
    })

csv_path = f"{OUT}/training_summary.csv"
with open(csv_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=summary_rows[0].keys())
    writer.writeheader()
    writer.writerows(summary_rows)
print(f"Saved training_summary.csv")

# Also dump full per-episode data for all stages into one combined CSV
combined_path = f"{OUT}/all_episodes.csv"
with open(combined_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["stage","ep","reward","success","steps","total_steps","critic_loss","actor_loss"])
    writer.writeheader()
    for stage in stages_ord:
        for r in all_data[stage]:
            writer.writerow({
                "stage": stage, "ep": r["ep"], "reward": r["reward"],
                "success": r["success"], "steps": r["steps"],
                "total_steps": r["total"], "critic_loss": r["critic"],
                "actor_loss": r["actor"],
            })
print(f"Saved all_episodes.csv ({sum(len(all_data[s]) for s in stages_ord)} rows)")

print("\nAll done. Files in ~/Desktop/results/:")
for f in sorted(os.listdir(OUT)):
    path = os.path.join(OUT, f)
    size = os.path.getsize(path)
    print(f"  {f:45s}  {size/1024:7.1f} KB")
