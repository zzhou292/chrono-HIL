#!/usr/bin/env python3
"""One command to collect both the latency-awareness and teleoperation
strengthening data the paper needs.

Three phases (run in order):

  1. LATENCY dose-response  [fully automated, no wheel]
       latency_awareness_ablation.py over a fine delay grid -> delay-aware vs
       delay-blind vs no-filter collision rate / clearance / intrusiveness as a
       function of command delay.  Turns the two-point ablation into a curve.

  2. TELEOP trace collection  [INTERACTIVE -- you drive the G29]
       human_delay_compensation_rounds.py logs a real operator command trace per
       (scenario, delay).  You will be prompted to drive each short round.

  3. TELEOP counterfactual replay  [automated]
       convoy_counterfactual_eval.py --trace-dir replays each recorded human
       trace filter-off vs DOB-CBF-on on the identical scenario -> causal
       collisions-prevented on genuine human intent.

Usage:
  python benchmarking/collect_strengthening_data.py            # all three phases
  python benchmarking/collect_strengthening_data.py --skip-teleop   # latency only (no wheel)
  python benchmarking/collect_strengthening_data.py --teleop-only   # phases 2-3 only

Notes:
  * Phase 2 needs the Logitech G29 and a human; it cannot be automated.
  * Set ACADOS_SOURCE_DIR and activate the `sim` conda env first.
"""
from __future__ import annotations
import argparse, glob, os, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "benchmarking"
RESULTS = BENCH / "results"
PY = sys.executable


def newest(prefix: str) -> str | None:
    ds = sorted(glob.glob(str(RESULTS / f"{prefix}_*")), key=os.path.getmtime)
    return ds[-1] if ds else None


def run(cmd: list[str], phase: str, interactive: bool = False) -> int:
    print("\n" + "=" * 72)
    print(f"  PHASE: {phase}")
    print("  " + " ".join(cmd))
    print("=" * 72, flush=True)
    # inherit stdio so the G29 prompts (and progress) are live
    rc = subprocess.run(cmd, cwd=str(ROOT)).returncode
    if rc != 0:
        print(f"  [warn] phase '{phase}' exited rc={rc}")
    return rc


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--skip-teleop", action="store_true", help="run only the automated latency phase")
    p.add_argument("--teleop-only", action="store_true", help="run only the teleop phases (2-3)")
    p.add_argument("--delays", nargs="+", type=float,
                   default=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                   help="fine command-delay grid for the latency dose-response")
    p.add_argument("--convoy", nargs="+",
                   default=["lead_brake", "convoy", "platoon", "rear_approach", "stalled"],
                   help="convoy scenarios for the (automated) latency sweep")
    p.add_argument("--teleop-convoy", nargs="+",
                   default=["lead_brake", "convoy", "platoon", "rear_approach", "stalled"],
                   help="convoy scenarios the human drives + replays (phase 2-3)")
    p.add_argument("--latency-profile",
                   default="config/latency_profiles/5g_nhits_geforce.json",
                   help="5G latency profile applied to BOTH the command (uplink) "
                        "and camera (downlink) channels during the human drive, and "
                        "to the command channel on replay. Default 5g_nhits_geforce: "
                        "full N-HiTS pipeline on the cloud-gaming geforce dataset -- "
                        "command from real UL traffic (p50/p95 ~29/69 ms), camera "
                        "from real DL video traffic (~96/207 ms), brief handover "
                        "spikes. Set to '' to fall back to fixed --hil-delays.")
    p.add_argument("--hil-delays", nargs="+", type=float, default=[0.30],
                   help="fixed command delay(s) the human drives under, used ONLY "
                        "if --latency-profile is empty")
    p.add_argument("--manual-mode", default="g29", choices=["g29", "wasd"])
    p.add_argument("--rounds", type=int, default=1, help="G29 rounds per (filter,delay) cell")
    p.add_argument("--workers", type=int, default=10)
    p.add_argument("--timeout", type=float, default=400.0)
    args = p.parse_args()

    if "ACADOS_SOURCE_DIR" not in os.environ:
        print("[warn] ACADOS_SOURCE_DIR not set; the controller may crash. "
              "export ACADOS_SOURCE_DIR=~/Documents/sbel/acados")

    outs: dict[str, str] = {}

    # ---- Phase 1: latency dose-response (automated) --------------------------
    if not args.teleop_only:
        run([PY, str(BENCH / "latency_awareness_ablation.py"),
             "--delays", *[str(d) for d in args.delays],
             "--convoy", *args.convoy,
             "--workers", str(args.workers), "--timeout", str(args.timeout)],
            "1/3  latency-awareness dose-response (automated)")
        outs["latency"] = newest("latency_awareness_ablation") or "(none found)"

    # ---- Phases 2-3: per scenario, drive (interactive) then replay (auto) ----
    # Looped per scenario so each recorded human trace is replayed on the
    # SAME convoy preset it was driven on (the replay uses one preset/dir).
    if not args.skip_teleop:
        use_profile = bool(args.latency_profile)
        lat_desc = (f"learned 5G profile ({Path(args.latency_profile).name}), both channels"
                    if use_profile else f"fixed delays {args.hil_delays}s, command only")
        n_drives = len(args.teleop_convoy) * (1 if use_profile else len(args.hil_delays))
        print("\n" + "*" * 72)
        print("  PHASE 2 IS INTERACTIVE: you drive the G29 for each short round.")
        print(f"  latency: {lat_desc}")
        print(f"  {len(args.teleop_convoy)} scenario(s) -> {n_drives} short drives.")
        print("  You drive under the delayed CAMERA (downlink) + delayed COMMAND (uplink);")
        print("  the filter is OFF while you drive, so this is your raw intent. Drive")
        print("  naturally toward the hazards so the filter has something to prevent.")
        print("*" * 72, flush=True)
        # latency args shared by the human drive (phase 2) and the replay (phase 3)
        if use_profile:
            drive_lat = ["--latency-profile-json", args.latency_profile, "--delays", "0.0"]
            replay_lat = ["--latency-profile-json", args.latency_profile]
        else:
            drive_lat = ["--delays", *[str(d) for d in args.hil_delays]]
            replay_lat = []   # convoy reads each round's recorded delay from its dir name
        replays = []
        for scen in args.teleop_convoy:
            # Phase 2: record raw human intent on this scenario (filter off), under
            # the realistic 5G latency on BOTH the camera and command channels.
            run([PY, str(BENCH / "human_delay_compensation_rounds.py"),
                 "--convoy", scen, "--filters", "none", *drive_lat,
                 "--rounds", str(args.rounds),
                 "--manual-mode", args.manual_mode, "--vis-mode", "sensor"],
                f"2/3  G29 drive: convoy='{scen}' under {lat_desc} (INTERACTIVE)",
                interactive=True)
            sess = newest("human_delay_compensation_rounds")
            if not sess:
                print(f"[warn] no recorded session for '{scen}'; skipping its replay")
                continue
            # Phase 3: replay this scenario's traces off vs DOB-CBF on the same preset,
            # under the same command-channel latency the filter saw live.
            run([PY, str(BENCH / "convoy_counterfactual_eval.py"),
                 "--trace-dir", sess, "--convoy", scen, "--filters", "none", "dob_cbf",
                 *replay_lat, "--workers", str(args.workers), "--timeout", str(args.timeout)],
                f"3/3  counterfactual replay: convoy='{scen}' (automated)")
            replays.append(f"{scen}: {newest('convoy_counterfactual_eval')}")
        outs["teleop_replays"] = "\n                    ".join(replays) if replays else "(none)"

    # ---- Summary -------------------------------------------------------------
    print("\n" + "=" * 72 + "\n  COLLECTION COMPLETE -- outputs:")
    for k, v in outs.items():
        print(f"    {k:14s}: {v}")
    print("\n  Feeds:")
    print("    latency      -> latency_awareness_ablation.png (dose-response) + summary_by_delay.csv")
    print("    teleop_replay-> per-trace collisions-prevented (off vs DOB-CBF) on real human intent")
    print("=" * 72)


if __name__ == "__main__":
    main()
