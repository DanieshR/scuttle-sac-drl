"""
After running 5 trials per method with run_mapping_trial.py, run this script to:
  1. Aggregate stats across trials (mean ± SD)
  2. Plot Fig 7 (coverage over time) and Fig 8 (summary bar chart)
  3. Save mapping_summary.csv with the Table 2 numbers for the paper
"""

import csv
import glob
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TRIAL_DIR = os.path.expanduser("~/Desktop/results/mapping_trials")
OUT       = os.path.expanduser("~/Desktop/results")

METHODS = {
    "sac_frontier": ("SAC+Frontier (proposed)", "#4C72B0"),
    "dwa_frontier": ("DWA+Frontier (classical)", "#55A868"),
    "random_walk":  ("Random Walk (baseline)",   "#C44E52"),
}

plt.rcParams.update({
    "font.family": "serif", "font.size": 11,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 150,
})


def load_metas(method):
    pattern = os.path.join(TRIAL_DIR, f"{method}_trial*_meta.csv")
    rows = []
    for path in sorted(glob.glob(pattern)):
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({k: float(v) if v not in ("None", "") else None
                             for k, v in row.items()})
    return rows


def load_coverage_timeseries(method):
    """Returns list of (t_array, coverage_array) one per trial."""
    pattern = os.path.join(TRIAL_DIR, f"{method}_trial*_coverage.csv")
    series = []
    for path in sorted(glob.glob(pattern)):
        ts, cs = [], []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                ts.append(float(row["t_s"]))
                cs.append(float(row["coverage_pct"]))
        if ts:
            series.append((np.array(ts), np.array(cs)))
    return series


def stat(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None
    return round(float(np.mean(vals)), 2), round(float(np.std(vals)), 2)


# ── Figure 7: coverage over time ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 5))

summary_rows = []

for method, (label, color) in METHODS.items():
    metas  = load_metas(method)
    series = load_coverage_timeseries(method)

    if not metas:
        print(f"[WARN] No trial data found for {method} — skipping.")
        continue

    # interpolate all trials onto a common time grid
    t_grid = np.linspace(0, 180, 360)
    interp_covs = []
    for t_arr, c_arr in series:
        interp_covs.append(np.interp(t_grid, t_arr, c_arr))

    if interp_covs:
        mean_cov = np.mean(interp_covs, axis=0)
        std_cov  = np.std(interp_covs, axis=0)
        ax.plot(t_grid, mean_cov, color=color, linewidth=2.2, label=label)
        ax.fill_between(t_grid, mean_cov - std_cov, mean_cov + std_cov,
                        color=color, alpha=0.15)

    # aggregate meta stats
    c60_m,  c60_s  = stat([r["coverage_60s"]  for r in metas])
    c120_m, c120_s = stat([r["coverage_120s"] for r in metas])
    c180_m, c180_s = stat([r["coverage_180s"] for r in metas])
    t50_m,  t50_s  = stat([r["time_to_50pct"] for r in metas])
    t75_m,  t75_s  = stat([r["time_to_75pct"] for r in metas])
    t90_m,  t90_s  = stat([r["time_to_90pct"] for r in metas])
    pl_m,   pl_s   = stat([r["path_length_m"] for r in metas])

    summary_rows.append({
        "method":              method,
        "n_trials":            len(metas),
        "coverage_60s_mean":   c60_m,  "coverage_60s_sd": c60_s,
        "coverage_120s_mean":  c120_m, "coverage_120s_sd": c120_s,
        "coverage_180s_mean":  c180_m, "coverage_180s_sd": c180_s,
        "time_to_50pct_mean":  t50_m,  "time_to_50pct_sd": t50_s,
        "time_to_75pct_mean":  t75_m,  "time_to_75pct_sd": t75_s,
        "time_to_90pct_mean":  t90_m,  "time_to_90pct_sd": t90_s,
        "path_length_mean_m":  pl_m,   "path_length_sd_m": pl_s,
    })

for t_ckpt in (60, 120):
    ax.axvline(t_ckpt, color="grey", linestyle=":", linewidth=1.0, alpha=0.6)

ax.set_xlabel("Elapsed time (s)")
ax.set_ylabel("Exploration coverage (%)")
ax.set_title("Fig. 7 — Coverage over time: SAC+Frontier vs. baselines (mean ± 1 SD, n=5 trials each)")
ax.legend(fontsize=9, loc="lower right", framealpha=0.8)
ax.set_ylim(0, 100)
fig.tight_layout()
fig.savefig(f"{OUT}/fig7_coverage_over_time.png")
plt.close(fig)
print("Saved fig7_coverage_over_time.png")

# ── Figure 8: summary bar (coverage@120s, time to 90%, path length) ─────────
if summary_rows:
    methods_ord = [r["method"] for r in summary_rows]
    labels_ord  = [METHODS[m][0] for m in methods_ord]
    colors_ord  = [METHODS[m][1] for m in methods_ord]
    x = np.arange(len(methods_ord))

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))

    def bar_panel(ax, vals, errs, ylabel, title):
        bars = ax.bar(x, vals, color=colors_ord, alpha=0.85,
                      yerr=errs, capsize=5, error_kw={"linewidth": 1.2})
        for bar, v in zip(bars, vals):
            if v is not None:
                ax.text(bar.get_x() + bar.get_width()/2, v + 0.5,
                        f"{v:.1f}", ha="center", va="bottom", fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels([l.split(" (")[0] for l in labels_ord], fontsize=9)
        ax.set_ylabel(ylabel)
        ax.set_title(title)

    c120_vals = [r["coverage_120s_mean"] for r in summary_rows]
    c120_errs = [r["coverage_120s_sd"]   for r in summary_rows]
    bar_panel(axes[0], c120_vals, c120_errs, "Coverage (%)", "Coverage @ 120 s")

    t90_vals = [r["time_to_90pct_mean"] for r in summary_rows]
    t90_errs = [r["time_to_90pct_sd"]   for r in summary_rows]
    bar_panel(axes[1], t90_vals, t90_errs, "Time (s)", "Time to 90% coverage")

    pl_vals = [r["path_length_mean_m"] for r in summary_rows]
    pl_errs = [r["path_length_sd_m"]   for r in summary_rows]
    bar_panel(axes[2], pl_vals, pl_errs, "Path length (m)", "Total path length to 90%")

    fig.suptitle("Fig. 8 — Quantitative mapping benchmarks: proposed vs. baselines",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig8_mapping_benchmark_bars.png")
    plt.close(fig)
    print("Saved fig8_mapping_benchmark_bars.png")

# ── Save summary CSV ─────────────────────────────────────────────────────────
if summary_rows:
    csv_path = f"{OUT}/mapping_summary.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=summary_rows[0].keys())
        w.writeheader()
        w.writerows(summary_rows)
    print(f"Saved mapping_summary.csv")

print("\nDone.")
