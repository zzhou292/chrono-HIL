#!/usr/bin/env python3
"""Tesla-style HMI overlay for HIL clips: a virtual steering wheel + a
linear accel/brake bar, rendered from logged telemetry and composited onto a
screen-recorded sim clip in post (so it never perturbs the real-time loop).

The overlay shows TWO traces so a DOB-CBF takeover is visible at a glance:
  * solid  = the command the vehicle actually applied (filter output);
  * ghost  = the operator's raw command (if logged).
When they diverge -- operator holding straight, applied wheel swinging around
an obstacle -- that divergence *is* the takeover.

Usage:
  # proof-of-concept still (no data needed):
  python make_hil_overlay.py --sample out.png

  # render the overlay frames from a run's telemetry at a video frame rate:
  python make_hil_overlay.py --sim-diag <run>/sim_diag.csv --fps 30 --out-dir frames/
  # then composite bottom-right onto the screen-recording with ffmpeg:
  #   ffmpeg -i clip.mp4 -framerate 30 -i frames/o_%05d.png \
  #     -filter_complex "[0][1]overlay=W-w-24:H-h-24:format=auto" -c:a copy annotated.mp4

Telemetry columns expected in sim_diag.csv: time, steering, throttle
(applied). Optional, for the takeover ghost: steering_cmd, throttle_cmd,
brake/brake_cmd. accel is taken as throttle-brake in [-1,1].
"""
from __future__ import annotations
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, Wedge

WHEEL_LOCK_DEG = 120.0   # display rotation at full steer (|steer|=1)
APPLIED = "#28c76f"      # solid (filter output / what the vehicle did)
GHOST   = "#9aa6b2"      # operator's raw command


def _wheel(ax, cx, cy, r, steer, color, lw, alpha=1.0, ghost=False):
    """Draw a steering wheel centred at (cx,cy), rotated by the steer input."""
    rot = np.deg2rad(steer * WHEEL_LOCK_DEG)   # Chrono steering: + = left turn
    ax.add_patch(Circle((cx, cy), r, fill=False, ec=color, lw=lw, alpha=alpha,
                        ls="--" if ghost else "-"))
    ax.add_patch(Circle((cx, cy), r * 0.16, fc=color, ec="none", alpha=alpha))
    for a0 in (90, 210, 330):                    # three spokes
        a = np.deg2rad(a0) + rot
        ax.plot([cx, cx + r * np.cos(a)], [cy, cy + r * np.sin(a)],
                color=color, lw=lw, alpha=alpha, ls="--" if ghost else "-")
    # top marker so the rotation is unambiguous
    top = np.pi / 2 + rot
    ax.add_patch(Wedge((cx + (r + 0.012) * np.cos(top), cy + (r + 0.012) * np.sin(top)),
                       0.02, 0, 360, fc=color, ec="none", alpha=alpha))


def render_panel(steer_app, accel_app, steer_cmd=None, accel_cmd=None):
    """Return an RGBA figure (transparent bg) of the HMI panel."""
    fig = plt.figure(figsize=(2.6, 1.5), dpi=200)
    fig.patch.set_alpha(0.0)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off"); ax.set_aspect("equal")
    # dark rounded backing
    ax.add_patch(FancyBboxPatch((0.02, 0.04), 0.96, 0.92,
                 boxstyle="round,pad=0.0,rounding_size=0.04",
                 fc="#11161c", ec="#2a3340", lw=1.0, alpha=0.78))
    # steering wheel (left)
    if steer_cmd is not None:
        _wheel(ax, 0.30, 0.50, 0.20, steer_cmd, GHOST, 1.6, alpha=0.7, ghost=True)
    _wheel(ax, 0.30, 0.50, 0.20, steer_app, APPLIED, 2.6)
    ax.text(0.30, 0.12, "steer", ha="center", va="center", color="#9aa6b2", fontsize=7)
    # accel/brake bar (right): center=0, throttle up (green), brake down (red)
    bx, by, bw, bh = 0.66, 0.50, 0.10, 0.34
    ax.add_patch(FancyBboxPatch((bx - bw/2, by - bh), bw, 2*bh, boxstyle="round,pad=0,rounding_size=0.02",
                 fc="#1b2530", ec="#2a3340", lw=0.8))
    ax.plot([bx - bw/2, bx + bw/2], [by, by], color="#3a4654", lw=1.0)  # zero line
    a = float(np.clip(accel_app, -1, 1))
    ax.add_patch(plt.Rectangle((bx - bw/2, by), bw, a*bh,
                 fc=(APPLIED if a >= 0 else "#ea5455"), ec="none", alpha=0.95))
    if accel_cmd is not None:
        ac = float(np.clip(accel_cmd, -1, 1))
        ax.plot([bx - bw/2 - 0.02, bx + bw/2 + 0.02], [by + ac*bh]*2, color=GHOST, lw=1.8, ls="--")
    ax.text(0.66, 0.12, "accel", ha="center", va="center", color="#9aa6b2", fontsize=7)
    return fig


def _render_frames(sim_diag, fps, outdir, t_offset=0.0, n_frames=None):
    import pandas as pd
    d = pd.read_csv(sim_diag)
    t = pd.to_numeric(d["time"], errors="coerce").to_numpy()
    def col(name, default=0.0):
        return pd.to_numeric(d[name], errors="coerce").to_numpy() if name in d.columns else np.full(len(d), float(default))
    steer = col("steering"); thr = col("throttle"); brk = col("brake")
    steer_c = col("steering_cmd"); thr_c = col("throttle_cmd"); brk_c = col("brake_cmd")
    has_cmd = "steering_cmd" in d.columns
    outd = Path(outdir); outd.mkdir(parents=True, exist_ok=True)
    n = int(n_frames if n_frames is not None else (t[-1] - t[0]) * fps)
    for i in range(n):
        tt = t[0] + t_offset + i / fps
        sa = float(np.interp(tt, t, steer)); aa = float(np.interp(tt, t, thr) - np.interp(tt, t, brk))
        sc = float(np.interp(tt, t, steer_c)) if has_cmd else None
        ac = (float(np.interp(tt, t, thr_c) - np.interp(tt, t, brk_c))) if has_cmd else None
        fig = render_panel(sa, aa, sc, ac)
        fig.savefig(outd / f"o_{i:05d}.png", dpi=200, transparent=True); plt.close(fig)
    return n, has_cmd


def _probe(video):
    import subprocess, json
    out = subprocess.run(["ffprobe", "-v", "0", "-of", "json", "-select_streams", "v:0",
        "-show_entries", "stream=r_frame_rate,width,height,duration",
        "-show_entries", "format=duration", video], capture_output=True, text=True).stdout
    j = json.loads(out); s = j["streams"][0]
    num, den = s["r_frame_rate"].split("/"); fps = float(num) / float(den)
    dur = float(s.get("duration") or j["format"]["duration"])
    return fps, int(s["width"]), int(s["height"]), dur


def main():
    import shutil, subprocess, tempfile
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sample", metavar="PNG", help="render a proof-of-concept takeover still and exit")
    p.add_argument("--sim-diag", help="run telemetry CSV (time, steering, throttle[, *_cmd])")
    p.add_argument("--video", help="base screen-recording; if given, composite in one step -> --out")
    p.add_argument("--out", default="annotated.mp4", help="output clip (with --video)")
    p.add_argument("--out-dir", help="dump overlay PNG frames here instead of compositing")
    p.add_argument("--fps", type=float, default=30.0, help="frame rate when no --video")
    p.add_argument("--t-offset", type=float, default=0.0, help="sim_diag time aligned to clip t=0 (s)")
    p.add_argument("--frac", type=float, default=0.24, help="overlay width / clip width")
    p.add_argument("--corner", choices=["br", "bl", "tr", "tl"], default="br")
    args = p.parse_args()

    if args.sample:
        fig = render_panel(steer_app=0.55, accel_app=-0.45, steer_cmd=0.0, accel_cmd=0.50)
        fig.savefig(args.sample, dpi=200, transparent=True, bbox_inches="tight")
        print(f"wrote {args.sample}"); return

    if not args.sim_diag:
        p.error("need --sim-diag (or --sample)")

    if args.video:
        if not (shutil.which("ffmpeg") and shutil.which("ffprobe")):
            raise SystemExit("ffmpeg/ffprobe not on PATH -- install them, or use --out-dir and composite manually.")
        fps, W, H, dur = _probe(args.video)
        tmp = Path(tempfile.mkdtemp(prefix="hilov_"))
        n, has_cmd = _render_frames(args.sim_diag, fps, tmp, args.t_offset, n_frames=int(dur * fps))
        ow = int(args.frac * W); pad = max(12, W // 90)
        pos = {"br": f"W-w-{pad}:H-h-{pad}", "bl": f"{pad}:H-h-{pad}",
               "tr": f"W-w-{pad}:{pad}", "tl": f"{pad}:{pad}"}[args.corner]
        subprocess.run(["ffmpeg", "-y", "-i", args.video, "-framerate", f"{fps}", "-i", str(tmp / "o_%05d.png"),
            "-filter_complex", f"[1:v]scale={ow}:-1[ov];[0:v][ov]overlay={pos}:shortest=1",
            "-c:a", "copy", "-c:v", "libx264", "-pix_fmt", "yuv420p", args.out], check=True)
        shutil.rmtree(tmp, ignore_errors=True)
        print(f"wrote {args.out}  ({n} frames @ {fps:.1f} fps, takeover-ghost={'on' if has_cmd else 'off (no *_cmd cols)'})")
        return

    out_dir = args.out_dir or "hil_overlay_frames"
    n, has_cmd = _render_frames(args.sim_diag, args.fps, out_dir, args.t_offset)
    print(f"wrote {n} overlay frames to {out_dir}/ at {args.fps} fps (composite with ffmpeg overlay=, see header).")


if __name__ == "__main__":
    main()
