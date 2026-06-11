"""Closed-loop (live-in-the-loop) terrain-estimator benchmark over an N-sample
uniform-LHS Bekker-Mohr terrain sweep -- the live replacement for the offline
LHS-100 estimator figures (Figs 8/9). For each LHS soil it drives the full NMPC
with each estimator backend live (deployed MLP, NN-UKF, Bekker-UKF, fused) and
records the tail-window in-loop |dn| and |dn|/n.

The runtime estimators clamp n to the deployable [0.5,1.1] range, so the LHS n
is sampled in that range (the other five Bekker-Mohr params span the full box);
this is the deployable-envelope analogue of the offline n in [0.40,1.30].

Usage:
  python benchmarking/closed_loop_estimator_lhs.py --n 100 --workers 12
"""
import sys, argparse, collections, statistics, yaml
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
import numpy as np, pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation")); sys.path.insert(0, str(ROOT / "benchmarking"))
from common import launch_and_collect
from param_consistency import generate_lhs_terrain_yaml_dicts, TERRAIN_PRESETS

BACKENDS = {"MLP": "learned", "Bekker-UKF": "bekker_ukf", "NN-UKF": "nn_ukf", "Fused-UKF": "nn_ukf_aug"}
SOIL_DIR = Path("/tmp/ttrans/cl_lhs_soils")
N_LO, N_HI = 0.52, 1.08  # interior of the estimator clamp [0.5,1.1]
_MANIFOLD_KEYS = ("Kphi", "Kc", "cohesion", "friction_angle", "janosi_shear")


def _manifold_yaml_from_n(n_val: float, jitter_frac: float = 0.0, rng=None) -> dict:
    """Bekker-Mohr soil yaml dict on the canonical clay->dirt->sand preset
    manifold at ``n_val`` (the same soil model the deployed estimator
    reconstructs from its n output), with optional +-``jitter_frac`` on the five
    non-n parameters for realism. n itself is the swept ground truth (un-jittered),
    so n-recovery is well posed: the soil's n IS the effective n to recover."""
    pts = sorted(((float(p["n"]), p) for p in TERRAIN_PRESETS.values()),
                 key=lambda kv: kv[0])
    nv = min(max(float(n_val), pts[0][0]), pts[-1][0])
    out = {"elastic_stiffness": 2e8, "damping": 3e4,
           "description": f"manifold soil n={n_val:.3f} jitter={jitter_frac:g}"}
    for i in range(len(pts) - 1):
        n0, p0 = pts[i]
        n1, p1 = pts[i + 1]
        if nv <= n1 or i == len(pts) - 2:
            w = 0.0 if n1 == n0 else (nv - n0) / (n1 - n0)
            for k in _MANIFOLD_KEYS:
                v = (1.0 - w) * float(p0[k]) + w * float(p1[k])
                if jitter_frac and rng is not None:
                    v *= (1.0 + jitter_frac * float(rng.uniform(-1.0, 1.0)))
                out[k] = v
            break
    out["n"] = float(n_val)
    return out


@dataclass(frozen=True)
class Task:
    idx: int; bk: str; n_true: float; yaml_path: str; sim_port: int; ctrl_port: int


def _run_one(t: Task):
    extra = ["--terrain-config", t.yaml_path,
             "--terrain-estimator", "--terrain-estimator-backend", BACKENDS[t.bk],
             "--terrain-estimator-mode", "n", "--te-update-interval", "8",
             "--te-min-confidence", "0.0"]
    r = launch_and_collect(experiment="cl_lhs", variant=t.bk, controller_mode="standard",
        mpc_model="nn", nn_model="vehicle_rate_64_32_lhs", terrain="dirt", path="sinusoidal",
        speed=5.0, bumpiness=0, seed=720,
        run_dir=Path(f"/tmp/ttrans/cl_lhs/{t.bk}_{t.idx}"),
        sim_port=t.sim_port, ctrl_port=t.ctrl_port, sim_time=20.0, timeout=500.0,
        lead_in=5.0, metric_start=10.0, extra_args=extra)
    if r.status != "ok" or not r.diag_csv:
        return (t.bk, t.idx, t.n_true, "fail", float('nan'), float('nan'))
    d = pd.read_csv(r.diag_csv); tt = pd.to_numeric(d["sim_time"], errors="coerce")
    n = pd.to_numeric(d["n_terrain_est"], errors="coerce")
    tail = (tt >= 11) & np.isfinite(n)
    if not tail.any():
        return (t.bk, t.idx, t.n_true, "fail", float('nan'), float('nan'))
    est = float(n[tail].mean())
    return (t.bk, t.idx, t.n_true, "ok", abs(est - t.n_true), est)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--mode", choices=["manifold", "lhs"], default="manifold",
                    help="manifold: sweep n along the clay->dirt->sand preset "
                         "manifold (deployment-consistent, well-posed n-recovery); "
                         "lhs: all-6-param uniform-LHS box (off-manifold stress test).")
    ap.add_argument("--jitter", type=float, default=0.10,
                    help="manifold mode: +-fractional jitter on the 5 non-n params.")
    args = ap.parse_args()
    SOIL_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    soils = []  # (n_true, yaml_dict)
    if args.mode == "manifold":
        edges = np.linspace(N_LO, N_HI, args.n + 1)  # stratified-uniform n
        for i in range(args.n):
            nt = float(rng.uniform(edges[i], edges[i + 1]))
            soils.append((nt, _manifold_yaml_from_n(nt, args.jitter, rng)))
    else:
        dicts = generate_lhs_terrain_yaml_dicts(args.n, seed=args.seed)
        for i, d in enumerate(dicts):
            d = dict(d); nt = float(np.clip(d["n"], N_LO, N_HI))
            if dicts[i]["n"] < N_LO or dicts[i]["n"] > N_HI:
                nt = float(rng.uniform(N_LO, N_HI))
            d["n"] = nt
            soils.append((nt, d))
    tasks = []
    for i, (nt, d) in enumerate(soils):
        yp = SOIL_DIR / f"soil_{i}.yaml"; yp.write_text(yaml.safe_dump(d))
        for bk in BACKENDS:
            base = 18000 + 2 * (i * len(BACKENDS) + list(BACKENDS).index(bk))
            tasks.append(Task(i, bk, nt, str(yp), base, base + 1))
    # prewarm one per backend (shared acados build), then pool the rest
    pw = [next(t for t in tasks if t.bk == bk) for bk in BACKENDS]
    res = [_run_one(t) for t in pw]
    rem = [t for t in tasks if t not in pw]
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in rem}
        for f in as_completed(futs): res.append(f.result())

    rows = [{"backend": bk, "idx": i, "n_true": nt, "status": st,
             "abs_dn": e, "est_n": en}
            for (bk, i, nt, st, e, en) in res]
    df = pd.DataFrame(rows)
    df.to_csv(ROOT / "benchmarking" / "closed_loop_estimator_lhs_runs.csv", index=False)
    print(f"\nCLOSED-LOOP LHS-{args.n} live estimator benchmark:")
    print(f"{'backend':12}{'N_ok':>6}{'median|dn|':>12}{'mean|dn|':>10}{'med%err':>9}{'<=20%':>7}")
    summ = []
    for bk in BACKENDS:
        ok = df[(df.backend == bk) & (df.status == "ok")]
        if ok.empty:
            print(f"{bk:12}{0:6}"); continue
        adn = ok["abs_dn"].to_numpy(); pe = 100.0 * adn / ok["n_true"].to_numpy()
        print(f"{bk:12}{len(ok):6}{np.median(adn):12.3f}{np.mean(adn):10.3f}"
              f"{np.median(pe):9.1f}{100.0*np.mean(pe<=20):7.0f}")
        summ.append({"backend": bk, "N_ok": int(len(ok)),
                     "median_abs_dn": float(np.median(adn)),
                     "mean_abs_dn": float(np.mean(adn)),
                     "median_pct_err": float(np.median(pe)),
                     "pct_within_20": float(100.0 * np.mean(pe <= 20))})
    pd.DataFrame(summ).to_csv(ROOT / "benchmarking" / "closed_loop_estimator_lhs_summary.csv", index=False)
    print("wrote closed_loop_estimator_lhs_{runs,summary}.csv")


if __name__ == "__main__":
    main()
