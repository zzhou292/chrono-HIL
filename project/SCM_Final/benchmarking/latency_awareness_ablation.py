#!/usr/bin/env python3
"""Latency-awareness ablation: does the DOB-CBF filter's *delay compensation*
actually buy anything, or does it just help to have a filter at all?

The command path is delayed by a fixed amount d in every run (the latency is
real and present). What we toggle is whether the filter is *told* about it:

  * ``none``        -- no filter (reference).
  * ``dob_blind``   -- DOB-CBF, but ``teleop_delay = 0``: it screens the stale,
                       delayed command as if there were no delay.
  * ``dob_aware``   -- DOB-CBF with ``teleop_delay = d``: it inflates the
                       obstacle buffer by a delay-dependent stopping distance and
                       forward-predicts over the round trip before acting.

Each cell is a deterministic replay of the same adversarial intent on the same
(scenario, delay) so blind-vs-aware is a clean, causal comparison. This is the
experiment that substantiates the paper's ``latency-aware'' claim.

Reproduce:
  python benchmarking/latency_awareness_ablation.py
"""
from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import run_process, timestamped_result_dir, save_summary_markdown  # noqa: E402
# Reuse the deterministic-replay machinery from the counterfactual eval.
from convoy_counterfactual_eval import (  # noqa: E402
    generate_reckless_trace, _metrics_from_run, PROJECT_ROOT, SIM_NODE,
)

VARIANTS = ("none", "dob_blind", "dob_aware")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--convoy", nargs="+",
                   default=["lead_brake", "convoy", "platoon", "rear_approach", "stalled"])
    p.add_argument("--delays", nargs="+", type=float, default=[0.15, 0.30],
                   help="Command-path delays at which delay-awareness can matter "
                        "(at 0 s blind==aware, so it is omitted by default).")
    p.add_argument("--reckless-throttle", nargs="+", type=float, default=[0.4, 0.6, 0.8])
    p.add_argument("--terrain", default="clay")
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--mesh-resolution", type=float, default=0.12)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--timeout", type=float, default=400.0)
    p.add_argument("--base-port", type=int, default=11800)
    return p.parse_args()


@dataclass(frozen=True)
class Task:
    idx: int
    variant: str
    convoy: str
    delay: float
    throttle: float
    sim_port: int
    run_dir: str
    trace: str
    terrain: str
    time_s: float
    mesh: float
    buffer: float
    timeout: float
    cell: str


def _build_cmd(t: Task) -> list[str]:
    cmd = [
        sys.executable, "-u", str(SIM_NODE),
        "--terrain", t.terrain, "--time", str(t.time_s), "--vis-mode", "none",
        "--sim-port", str(t.sim_port), "--mesh-resolution", str(t.mesh), "--no-noise",
        "--rocks", "0", "--convoy", t.convoy,
        "--replay-cmds", t.trace,
        "--sim-diag-csv", str(Path(t.run_dir) / "sim_diag.csv"),
        # The latency is ALWAYS present on the command path.
        "--manual-input-delay", str(t.delay),
    ]
    if t.variant != "none":
        cmd += ["--safety-filter", "--safety-flavor", "dob_cbf",
                "--safety-buffer", str(t.buffer)]
        # The ONLY difference between blind and aware: what the filter is told.
        cmd += ["--teleop-delay", str(t.delay) if t.variant == "dob_aware" else "0"]
    return cmd


def _run_one(task: Task) -> dict:
    run_dir = Path(task.run_dir)
    rc, wall, _ = run_process(_build_cmd(task), run_dir, task.timeout)
    row = {"idx": task.idx, "cell": task.cell, "variant": task.variant,
           "convoy": task.convoy, "delay_s": task.delay, "throttle": task.throttle,
           "rc": rc, "wall_s": round(wall, 1)}
    row.update(_metrics_from_run(run_dir))
    if rc != 0 and row["status"] == "ok":
        row["status"] = f"exit_{rc}"
    return row


def plot_figures(per_delay: pd.DataFrame, out_dir: Path) -> None:
    if per_delay.empty:
        return
    delays = sorted(per_delay["delay_s"].unique())
    variants = [v for v in VARIANTS if v in set(per_delay["variant"])]
    colors = {"none": "#b0392b", "dob_blind": "#e0a30c", "dob_aware": "#28c76f"}
    labels = {"none": "no filter", "dob_blind": "DOB-CBF, delay-blind",
              "dob_aware": "DOB-CBF, delay-aware"}
    fig, ax = plt.subplots(figsize=(6, 3.6))
    w = 0.26
    import numpy as np
    x = np.arange(len(delays))
    for i, v in enumerate(variants):
        sub = per_delay[per_delay["variant"] == v].set_index("delay_s").reindex(delays)
        ax.bar(x + (i - 1) * w, sub["collision_rate"].values, width=w,
               color=colors.get(v, "#888"), edgecolor="black", linewidth=0.4,
               label=labels.get(v, v))
    ax.set_xticks(x); ax.set_xticklabels([f"{d:g}" for d in delays])
    ax.set_xlabel("command-path delay $\\Delta_\\mathrm{cmd}$ (s)")
    ax.set_ylabel("collision rate")
    ax.set_title("Latency-awareness ablation (same adversarial intent)")
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "latency_awareness_ablation.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    out_dir = timestamped_result_dir("latency_awareness_ablation")
    print(f"Output: {out_dir}")

    traces = {}
    for thr in args.reckless_throttle:
        tp = str(out_dir / f"reckless_trace_t{thr:.2f}.csv")
        generate_reckless_trace(Path(tp), args.time, thr)
        traces[thr] = tp

    tasks: list[Task] = []
    idx = 0
    for thr in args.reckless_throttle:
        for preset in args.convoy:
            for delay in args.delays:
                for variant in VARIANTS:
                    run_dir = out_dir / "raw" / f"{idx:03d}_{preset}_{variant}_d{delay:.2f}_t{thr:.2f}"
                    tasks.append(Task(idx, variant, preset, delay, thr,
                                      args.base_port + 2 * idx, str(run_dir), traces[thr],
                                      args.terrain, args.time, args.mesh_resolution,
                                      args.safety_buffer, args.timeout,
                                      cell=f"{preset}@d{delay:.2f}@t{thr:.2f}"))
                    idx += 1

    rows = []
    print(f"[1/{len(tasks)}] prewarm ({tasks[0].variant})")
    rows.append(_run_one(tasks[0]))
    if len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
            futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
            for fut in as_completed(futs):
                rows.append(fut.result())
                pd.DataFrame(rows).sort_values("idx").to_csv(out_dir / "results.csv", index=False)
    df = pd.DataFrame(rows).sort_values("idx").reset_index(drop=True)
    df.to_csv(out_dir / "results.csv", index=False)

    ok = df[df["status"] == "ok"].copy()
    # Per (variant, delay): collision rate + mean clearance.
    per_delay = (ok.groupby(["variant", "delay_s"])
                 .agg(n=("collided", "size"), collisions=("collided", "sum"),
                      mean_clearance_m=("min_clearance_m", "mean"))
                 .reset_index())
    per_delay["collision_rate"] = (per_delay["collisions"] / per_delay["n"]).round(3)
    per_delay.to_csv(out_dir / "summary_by_delay.csv", index=False)

    # Per variant overall, with blind->aware harm-prevented matched by cell.
    summary_rows = []
    blind = ok[ok["variant"] == "dob_blind"].drop_duplicates("cell").set_index("cell")
    for v in VARIANTS:
        sub = ok[ok["variant"] == v]
        if sub.empty:
            continue
        rec = {"variant": v, "n": len(sub), "collisions": int(sub["collided"].sum()),
               "collision_rate": round(sub["collided"].mean(), 3),
               "mean_clearance_m": round(sub["min_clearance_m"].mean(), 3)}
        if v == "dob_aware":
            extra = 0  # collisions the blind filter had that aware prevented
            for _, r in sub.iterrows():
                if r["cell"] in blind.index and int(blind.loc[r["cell"], "collided"]) == 1 \
                        and r["collided"] == 0:
                    extra += 1
            rec["blind_collisions_prevented_by_awareness"] = extra
        summary_rows.append(rec)
    summary = pd.DataFrame(summary_rows)
    summary.to_csv(out_dir / "summary.csv", index=False)
    plot_figures(per_delay, out_dir)
    save_summary_markdown(out_dir, "Latency-Awareness Ablation", summary, [
        f"Convoy {args.convoy} x delays {args.delays} x intents {args.reckless_throttle}.",
        "Command path delayed by d in every run; only the filter's teleop_delay "
        "is toggled (blind=0, aware=d). Deterministic replay -> blind-vs-aware is "
        "causal. 'blind_collisions_prevented_by_awareness' counts cells the "
        "delay-blind filter collided on but the delay-aware filter did not.",
    ])
    print(f"\nDone: {out_dir}")
    print("Per (variant, delay):")
    print(per_delay.to_string(index=False))
    print("\nOverall:")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
