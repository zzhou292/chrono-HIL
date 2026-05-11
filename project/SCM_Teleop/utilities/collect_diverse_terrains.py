#!/usr/bin/env python3
"""Drive ``collect_terrain_traces.collect_one`` over a diverse set of soils
that go beyond the three canonical clay/dirt/sand presets.

Three families of new terrain are generated:

1. **On-manifold n sweep** — linearly interpolate / mildly extrapolate the
   six-parameter Bekker vector along the (clay → dirt → sand) preset
   sequence at densely spaced n values.  Targets the interpolation gaps
   identified in the generalisation study (especially the clay→dirt
   midpoint at n≈0.6 which the original 3-preset MLP got 9% wrong).

2. **Off-manifold cohesion variants** — at each preset n, write soils
   with cohesion scaled by 0.5× and 1.5× while leaving the other five
   parameters fixed.  Forces the regressor to learn that ``n`` is not
   the only knob that moves the dynamics signature.

3. **Off-manifold friction-angle variants** — same pattern, ±5° around
   each preset's ``friction_angle``.

Each new YAML is run for two random seeds × two throttles × one steering
amplitude (≈ four runs of 25 s each, ≈ 12 min for the full sweep).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from itertools import product
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from collect_terrain_traces import collect_one
from param_consistency import TERRAIN_PRESETS


PRESETS_BY_N = sorted(
    [(name, dict(params)) for name, params in TERRAIN_PRESETS.items()],
    key=lambda kv: float(kv[1]["n"]),
)
N_LO = float(PRESETS_BY_N[0][1]["n"])
N_HI = float(PRESETS_BY_N[-1][1]["n"])
INTERP_KEYS = ("Kphi", "Kc", "n", "cohesion", "friction_angle", "janosi_shear")


def interp_terrain(target_n: float) -> Dict[str, float]:
    """Interpolate / extrapolate the six-parameter vector along the preset
    sequence (mirrors the manifold mapping used by the live estimator)."""
    target_n = float(target_n)
    if target_n <= float(PRESETS_BY_N[0][1]["n"]):
        a, b = PRESETS_BY_N[0][1], PRESETS_BY_N[1][1]
    elif target_n >= float(PRESETS_BY_N[-1][1]["n"]):
        a, b = PRESETS_BY_N[-2][1], PRESETS_BY_N[-1][1]
    else:
        for i in range(len(PRESETS_BY_N) - 1):
            n_a = float(PRESETS_BY_N[i][1]["n"])
            n_b = float(PRESETS_BY_N[i + 1][1]["n"])
            if n_a <= target_n <= n_b:
                a, b = PRESETS_BY_N[i][1], PRESETS_BY_N[i + 1][1]
                break
    n_a = float(a["n"])
    n_b = float(b["n"])
    ratio = (target_n - n_a) / (n_b - n_a) if n_b != n_a else 0.0
    out: Dict[str, float] = {}
    for k in INTERP_KEYS:
        out[k] = float(a[k] + ratio * (b[k] - a[k]))
    out["n"] = target_n
    out["elastic_stiffness"] = 2e8
    out["damping"] = 3e4
    return out


def closest_preset(n_val: float) -> str:
    return min(TERRAIN_PRESETS.items(),
               key=lambda kv: abs(float(kv[1]["n"]) - n_val))[0]


def build_terrain_specs() -> List[Tuple[str, str, float, Dict[str, float]]]:
    """Return list of (label, preset_proxy_name, n_true, params_dict)."""
    specs: List[Tuple[str, str, float, Dict[str, float]]] = []

    # --- on-manifold n sweep ---
    interp_ns = [0.45, 0.55, 0.60, 0.65, 0.80, 0.85, 0.95, 1.00, 1.20]
    for n in interp_ns:
        cfg = interp_terrain(n)
        cfg["description"] = f"interp_n_{n:.2f}"
        specs.append((f"interpN{int(round(n*100)):03d}",
                      closest_preset(n), float(n), cfg))

    # --- off-manifold cohesion variants ---
    for name, preset in TERRAIN_PRESETS.items():
        for scale in (0.5, 1.5):
            cfg = {k: preset[k] for k in INTERP_KEYS}
            cfg["cohesion"] = float(preset["cohesion"]) * scale
            cfg["elastic_stiffness"] = 2e8
            cfg["damping"] = 3e4
            cfg["description"] = f"{name}_coh_x{scale:.1f}"
            specs.append((
                f"{name}-cohX{int(round(scale*10)):02d}",
                name, float(preset["n"]), cfg,
            ))

    # --- off-manifold friction-angle variants ---
    for name, preset in TERRAIN_PRESETS.items():
        for delta_phi in (-5.0, +5.0):
            phi_new = float(preset["friction_angle"]) + delta_phi
            if phi_new <= 5.0:
                continue
            cfg = {k: preset[k] for k in INTERP_KEYS}
            cfg["friction_angle"] = phi_new
            cfg["elastic_stiffness"] = 2e8
            cfg["damping"] = 3e4
            cfg["description"] = f"{name}_phi{int(round(delta_phi)):+d}"
            specs.append((
                f"{name}-phi{int(round(delta_phi)):+d}".replace("+", "p")
                                                       .replace("-", "m"),
                name, float(preset["n"]), cfg,
            ))

    return specs


def write_yaml(cfg: Dict[str, float], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(
        Path(__file__).parent.parent / "data" / "terrain_traces"))
    p.add_argument("--yaml-dir", default=str(
        Path(__file__).parent.parent / "data" / "terrain_yamls"))
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--throttles", type=float, nargs="+", default=[0.45, 0.65])
    p.add_argument("--steer-amps", type=float, nargs="+", default=[0.5])
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    p.add_argument("--sim-port-base", type=int, default=24000)
    args = p.parse_args()

    out_dir = Path(args.out)
    yaml_dir = Path(args.yaml_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    yaml_dir.mkdir(parents=True, exist_ok=True)

    specs = build_terrain_specs()
    combos = list(product(specs, args.throttles, args.steer_amps, args.seeds))
    print(f"[diverse] {len(specs)} terrain specs × {len(args.throttles)} thr "
          f"× {len(args.steer_amps)} amp × {len(args.seeds)} seeds "
          f"= {len(combos)} runs total")

    port = args.sim_port_base
    for idx, ((label, preset_proxy, n_true, cfg), thr, amp, seed) in enumerate(combos, 1):
        yaml_path = yaml_dir / f"{label}.yaml"
        if not yaml_path.exists():
            write_yaml(cfg, yaml_path)
        out_csv = out_dir / (
            f"{label}_thr{int(round(thr*100)):02d}"
            f"_amp{int(round(amp*100)):02d}_seed{seed}.csv"
        )
        print(f"\n[{idx}/{len(combos)}] {label} (n={n_true:.2f}) "
              f"thr={thr} amp={amp} seed={seed}")
        ok = collect_one(
            terrain=preset_proxy, throttle=thr, steer_amp=amp,
            steer_period=args.steer_period, seed=seed,
            duration=args.duration,
            sim_port=port, ctrl_port=port + 1,
            out_csv=out_csv, python_exe="conda",
            terrain_yaml=yaml_path, n_true_override=n_true,
        )
        port += 4
        if port > args.sim_port_base + 200:
            port = args.sim_port_base
        if not ok:
            print(f"    run failed — continuing")
        time.sleep(0.5)

    print("\n[diverse] done")


if __name__ == "__main__":
    main()
