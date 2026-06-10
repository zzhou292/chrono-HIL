"""Closed-loop (live-in-the-loop) terrain-estimator comparison across ALL four
runtime backends on the canonical clay/dirt/sand presets. Feeds the live
Fig. 11 (terrain_estimator_comparison) replacement: deployed MLP, online
NN-UKF, online Bekker-UKF, and the regime fusion, each run live inside the
NMPC. Tail-window in-loop |dn| + rms_cte. Writes a summary CSV."""
import sys, collections, statistics
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np, pandas as pd
sys.path.insert(0, "benchmarking")
from common import launch_and_collect
ROOT = Path(__file__).resolve().parents[1]
TN = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}
terrains = ["clay", "dirt", "sand"]; speeds = [5.0, 7.0]; seeds = [720, 721, 722]
backends = {"MLP": "learned", "Bekker-UKF": "bekker_ukf",
            "NN-UKF": "nn_ukf", "Fused-UKF": "nn_ukf_aug"}
cells = [(bk, t, sp, sd) for bk in backends for t in terrains for sp in speeds for sd in seeds]


def task(c):
    bk, terr, sp, sd = c; idx = abs(hash(c)) % 4000; port = 17000 + 2 * idx
    extra = ["--terrain-estimator", "--terrain-estimator-backend", backends[bk],
             "--terrain-estimator-mode", "n", "--te-update-interval", "8",
             "--te-min-confidence", "0.0"]
    r = launch_and_collect(experiment="cl_est_all", variant=bk, controller_mode="standard",
        mpc_model="nn", nn_model="vehicle_rate_64_32_lhs", terrain=terr, path="sinusoidal",
        speed=sp, bumpiness=0, seed=sd,
        run_dir=Path(f"/tmp/ttrans/clestall/{bk}_{terr}_v{sp:g}_s{sd}"),
        sim_port=port, ctrl_port=port + 1, sim_time=22.0, timeout=500.0,
        lead_in=5.0, metric_start=10.0, extra_args=extra)
    if r.status != "ok" or not r.diag_csv:
        return (bk, terr, sp, sd, "fail", float('nan'), float('nan'))
    d = pd.read_csv(r.diag_csv); t = pd.to_numeric(d["sim_time"], errors="coerce")
    n = pd.to_numeric(d["n_terrain_est"], errors="coerce")
    tail = (t >= 12) & np.isfinite(n)
    err = abs(float(n[tail].mean()) - TN[terr]) if tail.any() else float('nan')
    cte = float('nan')
    for col in ("rms_cte", "cte", "cross_track_error", "lateral_error"):
        if col in d.columns:
            cc = pd.to_numeric(d[col], errors="coerce")[t >= 10]
            if np.isfinite(cc).any(): cte = float(np.sqrt(np.nanmean(cc ** 2)))
            break
    return (bk, terr, sp, sd, "ok", err, cte)


pw = [next(c for c in cells if c[0] == bk) for bk in backends]
res = [task(c) for c in pw]
rem = [c for c in cells if c not in pw]
with ProcessPoolExecutor(max_workers=8) as ex:
    futs = {ex.submit(task, c): c for c in rem}
    for f in as_completed(futs): res.append(f.result())

agg = collections.defaultdict(lambda: collections.defaultdict(list)); cteagg = collections.defaultdict(list)
for bk, terr, sp, sd, st, err, cte in res:
    if st == "ok" and err == err: agg[bk][terr].append(err)
    if st == "ok" and cte == cte: cteagg[bk].append(cte)
rows = []
print("\nCLOSED-LOOP in-loop |dn| (tail-window), all live backends:")
print(f"{'backend':18}{'clay':>8}{'dirt':>8}{'sand':>8}{'ALL':>8}{'rms_cte':>9}{'N':>4}")
for bk in backends:
    row = []; allv = []
    for terr in terrains:
        v = agg[bk][terr]; row.append(statistics.mean(v) if v else float('nan')); allv += v
    am = statistics.mean(allv) if allv else float('nan')
    ct = statistics.mean(cteagg[bk]) if cteagg[bk] else float('nan')
    print(f"{bk:18}{row[0]:8.3f}{row[1]:8.3f}{row[2]:8.3f}{am:8.3f}{ct:9.4f}{len(allv):4}")
    rows.append({"backend": bk, "clay": row[0], "dirt": row[1], "sand": row[2],
                 "ALL": am, "rms_cte": ct, "N": len(allv)})
pd.DataFrame(rows).to_csv(ROOT / "benchmarking" / "closed_loop_estimator_all_summary.csv", index=False)
print("wrote closed_loop_estimator_all_summary.csv")
