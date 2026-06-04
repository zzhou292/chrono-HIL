#!/usr/bin/env python3
"""Two separate terrain-estimator convergence figures (tall, easy to read).

* `fig_terrain_estimator_closedloop.png` — NMPC tracks the sinusoidal
  reference at v_cmd = 7 m/s; estimator runs inside the loop.
* `fig_terrain_estimator_openloop.png`   — scripted sine steering +
  fixed throttle; estimator sees full-amplitude excitation.

Both panels are the same 3 ID + 4 OOD terrains, the same
`terrain_window_mlp` checkpoint, and the same window/warmup config.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[1]
RUNS = Path(__file__).parent / "runs"
OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)

ID_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}
ID_COLORS = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}
OOD_COLORS = ["#7e57c2", "#26a69a", "#ef6c00", "#c2185b"]


def ood_true_n():
    out = {}
    for i in (1, 2, 3, 4):
        with open(RUNS / "ood_terrains" / f"terrain{i}.yaml") as f:
            cfg = yaml.safe_load(f)
        out[i] = float(cfg["n"])
    return out


OOD_TRUTH = ood_true_n()


def _load_cl(terr_or_idx):
    """Closed-loop diag CSV. `terr_or_idx` is 'clay'/'dirt'/'sand' or int 1..4."""
    tag = f"te_id_{terr_or_idx}" if isinstance(terr_or_idx, str) else f"te_ood_t{terr_or_idx}"
    rd = RUNS / tag
    return pd.read_csv(next(rd.rglob("diag_*.csv")))


def _crop_active(df, lead_in_s=5.0, tail_drop_s=5.0,
                 stop_speed=1.0, stop_run_s=2.0):
    """Crop the diag CSV to the active driving window.

    * Drop the last `tail_drop_s` seconds (vehicle slowing into end-of-path).
    * After the `lead_in_s` warmup, find the FIRST point where the vehicle
      has been below `stop_speed` for `stop_run_s` continuous seconds and
      treat that as the end of active driving. Earlier slow samples
      (during initial acceleration) are ignored.
    The estimator output during the stopped portion is uninformative;
    cropping prevents it from biasing the tail-mean error.
    """
    if "u_meas" in df.columns:
        u = pd.to_numeric(df["u_meas"], errors="coerce").to_numpy()
    elif "u_true" in df.columns:
        u = pd.to_numeric(df["u_true"], errors="coerce").to_numpy()
    else:
        u = None
    t = df["sim_time"].to_numpy()
    t_end = float(t.max())
    cut_t = t_end - tail_drop_s
    if u is not None and len(t) > 1:
        dt = float(np.median(np.diff(t)))
        run_needed = max(1, int(round(stop_run_s / dt)))
        # Only consider samples AFTER lead-in
        first_after_lead = int(np.searchsorted(t, lead_in_s))
        run = 0
        first_stuck = None
        for i in range(first_after_lead, len(u)):
            if u[i] < stop_speed:
                run += 1
                if run >= run_needed:
                    first_stuck = i - run_needed + 1
                    break
            else:
                run = 0
        if first_stuck is not None:
            cut_t = min(cut_t, float(t[first_stuck]))
    return df[df["sim_time"] <= cut_t].reset_index(drop=True)


def _load_ol(terr_or_idx):
    """Open-loop diag CSV from the latest open_loop_terrain_estimator_benchmark."""
    base = ROOT / "benchmarking" / "results"
    bench = sorted(base.glob("open_loop_terrain_estimator_benchmark_*"))[-1]
    raw = bench / "raw"
    key = f"_id_{terr_or_idx}_" if isinstance(terr_or_idx, str) else f"_ood_terrain{terr_or_idx}_"
    hits = sorted(p for p in raw.iterdir() if p.is_dir() and key in p.name)
    if not hits:
        return None
    csvs = list(hits[0].rglob("open_loop_diag.csv"))
    return pd.read_csv(csvs[0]) if csvs else None


def _render(mode, title_suffix, out_name, xlim_max):
    loader = _load_cl if mode == "cl" else _load_ol

    fig, ax = plt.subplots(figsize=(13.0, 8.5))
    # Background band: canonical n range
    ax.axhspan(0.5, 1.1, alpha=0.05, color="#888",
               label="Canonical n range (clay → sand)")
    ax.axvline(4.2, color="#666", lw=0.9, ls="-.", alpha=0.7,
               label="Estimator warm-up complete (~4.2 s)")

    # Tight y-axis around 0.3-1.3 so a 0.05 error is visible.
    ax.set_xlim(0, xlim_max)
    ax.set_ylim(0.30, 1.25)

    # ID terrains — solid bold lines + matching dotted true-n lines
    for terr in ("clay", "dirt", "sand"):
        df = loader(terr)
        if df is None:
            continue
        df = _crop_active(df)
        t = df["sim_time"].to_numpy()
        n = df["n_terrain_est"].to_numpy()
        true_n = ID_N[terr]
        ax.plot(t, n, color=ID_COLORS[terr], lw=2.4,
                label=f"ID {terr}  est (true n={true_n:.2f})", zorder=5)
        ax.axhline(true_n, color=ID_COLORS[terr], lw=1.3, ls=":",
                   alpha=0.85, zorder=3,
                   label=f"ID {terr}  true")

    # OOD terrains — dashed thinner lines + matching dotted true-n lines
    for i in (1, 2, 3, 4):
        df = loader(i)
        if df is None:
            continue
        df = _crop_active(df)
        t = df["sim_time"].to_numpy()
        n = df["n_terrain_est"].to_numpy()
        true_n = OOD_TRUTH[i]
        c = OOD_COLORS[i - 1]
        ax.plot(t, n, color=c, lw=1.8, ls="--",
                label=f"OOD t{i} est (true n={true_n:.2f})", zorder=4)
        ax.axhline(true_n, color=c, lw=0.9, ls=":", alpha=0.55, zorder=2,
                   label=f"OOD t{i} true")

    ax.set_xlabel("Sim time (s)", fontsize=11)
    ax.set_ylabel("Estimated Bekker sinkage exponent  n", fontsize=11)
    ax.set_title(f"Terrain-estimator convergence — {title_suffix}\n"
                 f"3 canonical (ID) + 4 random (OOD) terrains, `terrain_window_mlp` model",
                 fontsize=11)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10),
              ncol=4, fontsize=9, framealpha=0.95)
    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.27, top=0.91)

    out = OUT_DIR / out_name
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


def _tail_stats(df, frac=0.25):
    """Return (mean, std, n_samples) over the last `frac` of the active window."""
    if df is None or df.empty:
        return float("nan"), float("nan"), 0
    t = df["sim_time"].to_numpy()
    t_end = float(t.max())
    t_start = float(t.min())
    cut = t_end - frac * (t_end - t_start)
    tail = df[df["sim_time"] >= cut]
    n = pd.to_numeric(tail["n_terrain_est"], errors="coerce").dropna()
    if n.empty:
        return float("nan"), float("nan"), 0
    return float(n.mean()), float(n.std()), int(len(n))


def _gather_tail(mode, frac=0.25):
    """Return list of dicts with keys: kind, label, true_n, est_mean, est_std,
    err, n_samples — for both ID and OOD terrains under the given mode."""
    loader = _load_cl if mode == "cl" else _load_ol
    rows = []
    for terr in ("clay", "dirt", "sand"):
        df = loader(terr)
        df = _crop_active(df) if df is not None else None
        mean, std, n = _tail_stats(df, frac=frac)
        rows.append(dict(kind="ID", label=terr, true_n=ID_N[terr],
                         est_mean=mean, est_std=std, n_samples=n,
                         err=mean - ID_N[terr] if np.isfinite(mean) else float("nan")))
    for i in (1, 2, 3, 4):
        df = loader(i)
        df = _crop_active(df) if df is not None else None
        mean, std, n = _tail_stats(df, frac=frac)
        rows.append(dict(kind="OOD", label=f"t{i}", true_n=OOD_TRUTH[i],
                         est_mean=mean, est_std=std, n_samples=n,
                         err=mean - OOD_TRUTH[i] if np.isfinite(mean) else float("nan")))
    return rows


def _render_bars(frac=0.25):
    """Steady-state |error| summary on the 103-terrain random sweep.

    Top:    boxplot of |tail-mean error| binned by true n (5 bins
            spanning the deployment range), CL vs OL side-by-side.
    Bottom: mean |error| per terrain class — clay (canonical + random
            soils with n≤0.6), dirt (0.6<n≤0.9), sand (n>0.9), plus
            the canonical clay/dirt/sand presets singled out.
    """
    src = Path(__file__).parent / "runs_v7_random" / "results.csv"
    if not src.exists():
        print(f"[bars] missing {src} — skipping")
        return
    df = pd.read_csv(src)
    df = df[(df["ok"] == 1) &
            pd.to_numeric(df["tail_n_est"], errors="coerce").notna()].copy()
    df["abs_err"] = (df["tail_n_est"] - df["n_true"]).abs()

    # n-bin assignment
    bins = [(0.40, 0.55, "0.40–0.55"),
            (0.55, 0.70, "0.55–0.70"),
            (0.70, 0.85, "0.70–0.85"),
            (0.85, 1.05, "0.85–1.05"),
            (1.05, 1.30, "1.05–1.30")]

    def bin_label(n):
        for lo, hi, lab in bins:
            if lo <= n < hi:
                return lab
        return bins[-1][2] if n >= bins[-1][0] else None

    df["n_bin"] = df["n_true"].apply(bin_label)
    cl = df[df["mode"] == "cl"]
    ol = df[df["mode"] == "ol"]

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(11, 8.5),
                                          gridspec_kw=dict(height_ratios=[1.4, 1]))

    # ---- top: per-n-bin boxplot ----
    bin_labels = [b[2] for b in bins]
    cl_data = [cl[cl.n_bin == lab].abs_err.values for lab in bin_labels]
    ol_data = [ol[ol.n_bin == lab].abs_err.values for lab in bin_labels]

    positions = np.arange(len(bin_labels))
    width = 0.35
    bp_cl = ax_top.boxplot(cl_data, positions=positions - width/2, widths=width,
                            patch_artist=True, showfliers=True,
                            boxprops=dict(facecolor="#1f77b4", alpha=0.7,
                                          edgecolor="#0a3d62"),
                            medianprops=dict(color="white", lw=2),
                            whiskerprops=dict(color="#0a3d62"),
                            capprops=dict(color="#0a3d62"),
                            flierprops=dict(marker="o", markersize=4,
                                            markerfacecolor="#1f77b4",
                                            markeredgecolor="#0a3d62", alpha=0.6))
    bp_ol = ax_top.boxplot(ol_data, positions=positions + width/2, widths=width,
                            patch_artist=True, showfliers=True,
                            boxprops=dict(facecolor="#ff7f0e", alpha=0.7,
                                          edgecolor="#7d3c00"),
                            medianprops=dict(color="white", lw=2),
                            whiskerprops=dict(color="#7d3c00"),
                            capprops=dict(color="#7d3c00"),
                            flierprops=dict(marker="s", markersize=4,
                                            markerfacecolor="#ff7f0e",
                                            markeredgecolor="#7d3c00", alpha=0.6))

    # annotate sample counts and medians on top
    for i, lab in enumerate(bin_labels):
        ax_top.text(positions[i] - width/2,
                    np.median(cl_data[i]) + 0.005,
                    f"n={len(cl_data[i])}",
                    ha="center", va="bottom", fontsize=8, color="#0a3d62")
        ax_top.text(positions[i] + width/2,
                    np.median(ol_data[i]) + 0.005,
                    f"n={len(ol_data[i])}",
                    ha="center", va="bottom", fontsize=8, color="#7d3c00")

    ax_top.axhline(0.05, color="#2ca02c", lw=1.0, ls="--", alpha=0.7,
                   label="|error| = 0.05")
    ax_top.axhline(0.10, color="#d62728", lw=1.0, ls="--", alpha=0.7,
                   label="|error| = 0.10")
    ax_top.set_xticks(positions)
    ax_top.set_xticklabels(bin_labels)
    ax_top.set_xlabel("True n bin", fontsize=11)
    ax_top.set_ylabel("|tail-mean error|", fontsize=11)
    ax_top.set_title(
        "Steady-state estimator |error| binned by true n  "
        "(box = IQR, median = white line, whiskers = 1.5× IQR)\n"
        "103 terrains × 2 excitation modes; "
        "closed-loop NMPC (blue) vs scripted open-loop (orange)",
        fontsize=11)
    ax_top.set_ylim(0, max(0.4, df.abs_err.max() * 1.05))
    # Custom legend (boxplot handles + reference lines)
    from matplotlib.patches import Patch
    ax_top.legend(handles=[
        Patch(facecolor="#1f77b4", alpha=0.7, label="Closed-loop NMPC"),
        Patch(facecolor="#ff7f0e", alpha=0.7, label="Open-loop scripted"),
        plt.Line2D([0], [0], color="#2ca02c", ls="--", label="|error| = 0.05"),
        plt.Line2D([0], [0], color="#d62728", ls="--", label="|error| = 0.10"),
    ], loc="upper right", fontsize=9)
    ax_top.grid(axis="y", alpha=0.3)

    # ---- bottom: canonical clay/dirt/sand summary ----
    canon = df[df.label.str.startswith("canon_")].copy()
    canon["name"] = canon.label.str.replace("canon_", "").str.replace("_cl", "").str.replace("_ol", "")
    # The label was 'canon_clay' for both modes — distinguish by 'mode'
    pivot_canon = canon.pivot_table(index=["name", "n_true"],
                                     columns="mode", values="abs_err",
                                     aggfunc="first").reset_index()
    if pivot_canon.empty:
        print("[bars] no canonical rows found")
    canon_order = ["clay", "dirt", "sand"]
    pivot_canon = pivot_canon.set_index("name").reindex(canon_order).reset_index()

    x = np.arange(len(canon_order))
    w = 0.3
    if "cl" in pivot_canon.columns:
        b1 = ax_bot.bar(x - w/2, pivot_canon["cl"].values, w,
                         color="#1f77b4", alpha=0.85,
                         label="Closed-loop")
        for bi, v in enumerate(pivot_canon["cl"].values):
            if np.isfinite(v):
                ax_bot.text(x[bi] - w/2, v + 0.003, f"{v:.3f}",
                             ha="center", va="bottom", fontsize=9)
    if "ol" in pivot_canon.columns:
        b2 = ax_bot.bar(x + w/2, pivot_canon["ol"].values, w,
                         color="#ff7f0e", alpha=0.85,
                         label="Open-loop")
        for bi, v in enumerate(pivot_canon["ol"].values):
            if np.isfinite(v):
                ax_bot.text(x[bi] + w/2, v + 0.003, f"{v:.3f}",
                             ha="center", va="bottom", fontsize=9)
    ax_bot.axhline(0.05, color="#2ca02c", lw=1.0, ls="--", alpha=0.7)
    ax_bot.axhline(0.10, color="#d62728", lw=1.0, ls="--", alpha=0.7)
    ax_bot.set_xticks(x)
    ax_bot.set_xticklabels(
        [f"{name}\n(n = {float(pivot_canon.iloc[i].n_true):.2f})"
         for i, name in enumerate(canon_order)]
    )
    ax_bot.set_ylabel("|tail-mean error|", fontsize=11)
    ax_bot.set_title("Canonical clay/dirt/sand presets — steady-state |error|",
                      fontsize=11)
    ax_bot.set_ylim(0, max(0.25,
                            float(pivot_canon[["cl", "ol"]].values.flatten().max()) * 1.3))
    ax_bot.legend(loc="upper right", fontsize=9)
    ax_bot.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    out = OUT_DIR / "fig_terrain_estimator_steady_state.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")

    # CSV dump: per-bin stats
    out_csv = OUT_DIR / "fig_terrain_estimator_steady_state.csv"
    rows_out = []
    for lab in bin_labels:
        for m in ("cl", "ol"):
            sub = df[(df.n_bin == lab) & (df["mode"] == m)]
            if sub.empty:
                continue
            rows_out.append(dict(
                n_bin=lab, mode=m, n=len(sub),
                mean_abs_err=sub.abs_err.mean(),
                median_abs_err=sub.abs_err.median(),
                p90_abs_err=sub.abs_err.quantile(0.9),
                max_abs_err=sub.abs_err.max(),
            ))
    pd.DataFrame(rows_out).to_csv(out_csv, index=False)
    print(f"Wrote {out_csv}")


def _gather_te_tail(frac=0.25):
    """Gather (true_n, est_mean, est_std, mode, label) from the te_runs/
    closed-loop scenarios + the latest open-loop benchmark."""
    points = []
    for terr in ("clay", "dirt", "sand"):
        df = _crop_active(_load_cl(terr))
        mean, std, _ = _tail_stats(df, frac=frac)
        points.append(dict(true_n=ID_N[terr], est=mean, std=std,
                           mode="cl", label=f"ID {terr}"))
    for i in (1, 2, 3, 4):
        df = _crop_active(_load_cl(i))
        mean, std, _ = _tail_stats(df, frac=frac)
        points.append(dict(true_n=OOD_TRUTH[i], est=mean, std=std,
                           mode="cl", label=f"OOD t{i}"))
    for terr in ("clay", "dirt", "sand"):
        df = _load_ol(terr)
        if df is not None:
            df = _crop_active(df)
            mean, std, _ = _tail_stats(df, frac=frac)
            points.append(dict(true_n=ID_N[terr], est=mean, std=std,
                               mode="ol", label=f"ID {terr}"))
    for i in (1, 2, 3, 4):
        df = _load_ol(i)
        if df is not None:
            df = _crop_active(df)
            mean, std, _ = _tail_stats(df, frac=frac)
            points.append(dict(true_n=OOD_TRUTH[i], est=mean, std=std,
                               mode="ol", label=f"OOD t{i}"))
    return points


def _gather_eval_sweep():
    """Pull the per-terrain results from the v7 eval sweep
    (deliverables/runs_v7_eval/results.csv)."""
    eval_csv = Path(__file__).parent / "runs_v7_eval" / "results.csv"
    if not eval_csv.exists():
        return []
    df = pd.read_csv(eval_csv)
    df = df[df["ok"] == 1]
    df = df[pd.to_numeric(df["tail_n_est"], errors="coerce").notna()]
    rows = []
    for _, r in df.iterrows():
        rows.append(dict(true_n=float(r["n_true"]),
                         est=float(r["tail_n_est"]),
                         std=0.0,
                         mode=str(r["mode"]),
                         label=str(r["label"])))
    return rows


def _render_scatter():
    """Single 'paper-fair' figure: estimate vs true n across 100 random
    Bekker-jittered terrains spanning n ∈ [0.3, 1.3] + canonical
    clay/dirt/sand. Closed-loop NMPC at v_cmd = 7 m/s on the left,
    Buzhardt-style scripted excitation on the right. One point per
    (terrain, mode); error bars are tail-window std."""
    src = Path(__file__).parent / "runs_v7_random" / "results.csv"
    if not src.exists():
        print(f"[scatter] missing {src} — skipping")
        return
    df = pd.read_csv(src)
    df = df[(df["ok"] == 1) &
            pd.to_numeric(df["tail_n_est"], errors="coerce").notna()]
    points = []
    for _, r in df.iterrows():
        is_canon = str(r["label"]).startswith("canon_")
        points.append(dict(
            true_n=float(r["n_true"]),
            est=float(r["tail_n_est"]),
            std=float(r["tail_n_std"]) if "tail_n_std" in r and pd.notna(r["tail_n_std"]) else 0.0,
            mode=str(r["mode"]),
            label=str(r["label"]),
            is_canon=is_canon,
        ))
    cl = [p for p in points if p["mode"] == "cl" and np.isfinite(p["est"])]
    ol = [p for p in points if p["mode"] == "ol" and np.isfinite(p["est"])]

    fig, ax = plt.subplots(figsize=(9.5, 8.0))
    n_grid = np.linspace(0.25, 1.35, 200)
    ax.fill_between(n_grid, n_grid - 0.10, n_grid + 0.10,
                    color="#888", alpha=0.10, label=r"$\pm$0.10 band")
    ax.fill_between(n_grid, n_grid - 0.05, n_grid + 0.05,
                    color="#888", alpha=0.18, label=r"$\pm$0.05 band")
    ax.plot(n_grid, n_grid, color="#444", lw=1.2, ls="--",
            label="perfect estimation (y = x)", zorder=3)

    def _split(points, is_canon):
        sub = [p for p in points if p.get("is_canon", False) == is_canon]
        if not sub:
            return None, None, None
        return (np.array([p["true_n"] for p in sub]),
                np.array([p["est"]    for p in sub]),
                np.array([p["std"]    for p in sub]))

    # Random points (no error bars to keep the cloud readable)
    x, y, _ = _split(cl, is_canon=False)
    if x is not None:
        ax.scatter(x, y, marker="o", s=36, c="#1f77b4", alpha=0.65,
                   edgecolors="#0a3d62", linewidths=0.5,
                   label=f"Closed-loop NMPC (n={len(cl)})", zorder=5)
    x, y, _ = _split(ol, is_canon=False)
    if x is not None:
        ax.scatter(x, y, marker="s", s=36, c="#ff7f0e", alpha=0.65,
                   edgecolors="#7d3c00", linewidths=0.5,
                   label=f"Open-loop Buzhardt (n={len(ol)})", zorder=5)

    # Canonical clay/dirt/sand emphasized
    x, y, _ = _split(cl, is_canon=True)
    if x is not None:
        ax.scatter(x, y, marker="*", s=240, c="#1f77b4",
                   edgecolors="black", linewidths=1.2,
                   label="Canonical CL (clay/dirt/sand)", zorder=7)
    x, y, _ = _split(ol, is_canon=True)
    if x is not None:
        ax.scatter(x, y, marker="*", s=240, c="#ff7f0e",
                   edgecolors="black", linewidths=1.2,
                   label="Canonical OL (clay/dirt/sand)", zorder=7)

    ax.set_xlim(0.28, 1.32)
    ax.set_ylim(0.28, 1.32)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("True sinkage exponent  n", fontsize=12)
    ax.set_ylabel("Estimated  n  (mean over last 5 s after convergence)",
                  fontsize=12)
    ax.set_title(
        "Online terrain estimator accuracy across 103 random soils\n"
        "(100 randomly sampled terrains + clay, dirt, sand)",
        fontsize=12)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right", fontsize=9, framealpha=0.95)

    out = OUT_DIR / "fig_terrain_estimator_scatter.png"
    fig.tight_layout()
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")

    # CSV dump + summary stats per mode
    out_csv = OUT_DIR / "fig_terrain_estimator_scatter.csv"
    with open(out_csv, "w") as f:
        f.write("label,mode,true_n,est,std,abs_err,is_canon\n")
        for p in points:
            if not np.isfinite(p["est"]):
                continue
            f.write(f"{p['label']},{p['mode']},{p['true_n']:.4f},"
                    f"{p['est']:.4f},{p['std']:.4f},"
                    f"{abs(p['est']-p['true_n']):.4f},"
                    f"{int(p.get('is_canon', False))}\n")
    print(f"Wrote {out_csv}")

    for mode_name, pts in [("Closed-loop", cl), ("Open-loop", ol)]:
        errs = np.array([abs(p["est"] - p["true_n"]) for p in pts])
        print(f"  {mode_name:10s} (n={len(pts)}): "
              f"median |err|={np.median(errs):.3f}  "
              f"mean={errs.mean():.3f}  "
              f"P90={np.percentile(errs, 90):.3f}  "
              f"max={errs.max():.3f}")


def main():
    _render("cl",
            "closed-loop NMPC sinusoidal tracking (v_cmd = 7 m/s)",
            "fig_terrain_estimator_closedloop.png",
            xlim_max=30)
    _render("ol",
            "scripted open-loop excitation (Buzhardt-style)",
            "fig_terrain_estimator_openloop.png",
            xlim_max=25)
    _render_bars(frac=0.25)
    _render_scatter()

    print("\n=== closed-loop tail-mean error (last 5 s of ACTIVE window) ===")
    for terr in ("clay", "dirt", "sand"):
        df = _crop_active(_load_cl(terr))
        tail = df[df["sim_time"] >= df["sim_time"].max() - 5.0]
        est = tail["n_terrain_est"].mean()
        true_n = ID_N[terr]
        print(f"  ID  {terr:5s}  true={true_n:.2f}  est={est:.3f}  err={est-true_n:+.3f}")
    for i in (1, 2, 3, 4):
        df = _crop_active(_load_cl(i))
        tail = df[df["sim_time"] >= df["sim_time"].max() - 5.0]
        est = tail["n_terrain_est"].mean()
        true_n = OOD_TRUTH[i]
        print(f"  OOD t{i}     true={true_n:.2f}  est={est:.3f}  err={est-true_n:+.3f}")

    print("\n=== open-loop tail-mean error (active window) ===")
    for terr in ("clay", "dirt", "sand"):
        df = _load_ol(terr)
        if df is None: continue
        df = _crop_active(df)
        tail = df[df["sim_time"] >= df["sim_time"].max() - 5.0]
        est = tail["n_terrain_est"].mean()
        true_n = ID_N[terr]
        print(f"  ID  {terr:5s}  true={true_n:.2f}  est={est:.3f}  err={est-true_n:+.3f}")
    for i in (1, 2, 3, 4):
        df = _load_ol(i)
        if df is None: continue
        df = _crop_active(df)
        tail = df[df["sim_time"] >= df["sim_time"].max() - 5.0]
        est = tail["n_terrain_est"].mean()
        true_n = OOD_TRUTH[i]
        print(f"  OOD t{i}     true={true_n:.2f}  est={est:.3f}  err={est-true_n:+.3f}")


if __name__ == "__main__":
    main()
