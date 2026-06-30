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
    p.add_argument("--latency-profiles", nargs="+",
                   default=["config/latency_profiles/5g_nhits_geforce.json",
                            "config/latency_profiles/5g_nhits_congested.json"],
                   help="One or more 5G latency profiles; each scenario is driven (and "
                        "its trace replayed) under EVERY listed profile, so the study "
                        "spans link quality. Defaults to the two-condition link study: "
                        "5g_nhits_geforce (GOOD cloud-gaming link -- command p50/p95 "
                        "29/69 ms, camera 96/207 ms) and 5g_nhits_congested (CONGESTED "
                        "cell -- command p50/p95 148/320 ms, camera 348/675 ms, real "
                        "congestion windows + brief outages). The profile drives both "
                        "the command (uplink) and camera (downlink) channels live and "
                        "the command channel on replay. Set to '' to fall back to fixed "
                        "--hil-delays.")
    p.add_argument("--hil-delays", nargs="+", type=float, default=[0.30],
                   help="fixed command delay(s) the human drives under, used ONLY "
                        "if --latency-profile is empty")
    p.add_argument("--manual-mode", default="g29", choices=["g29", "wasd"])
    p.add_argument("--teleop-terrain", default="clay",
                   help="single terrain the human drives on. Pinned to ONE value so "
                        "each convoy scenario is exactly one drive -- the variety you "
                        "experience is the SCENARIO, not the soil. Default clay (firm, "
                        "drivable). Avoid sand+bumps: the HMMWV bogs down and the round "
                        "records zero motion.")
    p.add_argument("--teleop-bumpiness", type=int, default=0,
                   help="single bumpiness level for the human drives (0 = smooth). "
                        "Pinned so one scenario = one drive.")
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

    # ---- Phases 2-3: per (link, scenario), drive (interactive) then replay (auto) ----
    # Looped per scenario so each recorded human trace is replayed on the SAME
    # convoy preset AND under the SAME link it was driven on. Each scenario is
    # ONE drive per link (terrain/path/speed/bumpiness pinned below), so the
    # variety the human experiences is the scenario and the link, not the soil.
    if not args.skip_teleop:
        profiles = [p for p in args.latency_profiles if p]  # drop '' entries
        use_profile = bool(profiles)
        if not use_profile:
            profiles = [""]  # single fixed-delay pass
        n_drives = len(args.teleop_convoy) * (
            len(profiles) if use_profile else len(args.hil_delays))
        print("\n" + "*" * 72)
        print("  PHASE 2 IS INTERACTIVE: you drive the G29 for each short round.")
        if use_profile:
            print(f"  {len(profiles)} link condition(s): "
                  + ", ".join(Path(p).stem for p in profiles))
        else:
            print(f"  fixed delays {args.hil_delays}s, command channel only")
        print(f"  {len(args.teleop_convoy)} scenario(s) x the above -> {n_drives} short drives.")
        print("  You drive under the delayed CAMERA (downlink) + delayed COMMAND (uplink);")
        print("  the filter is OFF while you drive, so this is your raw intent. Drive")
        print("  naturally toward the hazards so the filter has something to prevent.")
        print("*" * 72, flush=True)
        replays = []
        for prof in profiles:
            link = Path(prof).stem if prof else f"fixed{args.hil_delays}"
            if use_profile:
                drive_lat = ["--latency-profile-json", prof, "--delays", "0.0"]
                replay_lat = ["--latency-profile-json", prof]
            else:
                drive_lat = ["--delays", *[str(d) for d in args.hil_delays]]
                replay_lat = []   # convoy reads each round's recorded delay from its dir name
            for scen in args.teleop_convoy:
                # Phase 2: record raw human intent on this (link, scenario) with the
                # filter off, under the 5G latency on BOTH camera and command channels.
                run([PY, str(BENCH / "human_delay_compensation_rounds.py"),
                     "--convoy", scen, "--filters", "none", *drive_lat,
                     "--rounds", str(args.rounds),
                     # Pin terrain/path/speed/bumpiness to ONE value each so a single
                     # convoy scenario expands to exactly ONE drive (otherwise
                     # human_delay's default 2 terrains x 2 bumpiness balloons each
                     # scenario into 4 near-identical rounds, masking the scenario
                     # variety and burying the human under repeats of scenario #1).
                     "--terrains", args.teleop_terrain,
                     "--bumpiness", str(args.teleop_bumpiness),
                     "--paths", "straight", "--speeds", "4.0",
                     "--manual-mode", args.manual_mode, "--vis-mode", "sensor"],
                    f"2/3  G29 drive: link='{link}' convoy='{scen}' (INTERACTIVE)",
                    interactive=True)
                sess = newest("human_delay_compensation_rounds")
                if not sess:
                    print(f"[warn] no recorded session for '{link}/{scen}'; skipping its replay")
                    continue
                # Phase 3: replay this scenario's traces off vs DOB-CBF on the same
                # preset, under the same command-channel latency the filter saw live.
                run([PY, str(BENCH / "convoy_counterfactual_eval.py"),
                     "--trace-dir", sess, "--convoy", scen, "--filters", "none", "dob_cbf",
                     *replay_lat, "--workers", str(args.workers), "--timeout", str(args.timeout)],
                    f"3/3  counterfactual replay: link='{link}' convoy='{scen}' (automated)")
                replays.append(f"{link}/{scen}: {newest('convoy_counterfactual_eval')}")
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
