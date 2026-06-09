import sys, collections, statistics
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np, pandas as pd
sys.path.insert(0,"benchmarking")
from common import launch_and_collect
TN={"clay":0.5,"sand":1.1}
terrains=["clay","sand"]; amps=[0.0,0.2,0.35,0.5]; seeds=[720,721]
cells=[(terr,amp,sd) for terr in terrains for amp in amps for sd in seeds]
def task(c):
    terr,amp,sd=c; idx=abs(hash(c))%4000; port=16000+2*idx
    extra=["--terrain-estimator","--terrain-estimator-backend","nn_ukf","--terrain-estimator-mode","n",
           "--te-update-interval","8","--te-min-confidence","0.0",
           "--excitation-steer-amp",str(amp)]
    r=launch_and_collect(experiment="nnukf_probe",variant=f"a{amp}",controller_mode="standard",mpc_model="nn",
        nn_model="vehicle_rate_64_32_lhs",terrain=terr,path="sinusoidal",speed=5.0,bumpiness=0,seed=sd,
        run_dir=Path(f"/tmp/ttrans/probe/{terr}_a{amp}_s{sd}"),sim_port=port,ctrl_port=port+1,
        sim_time=22.0,timeout=400.0,lead_in=5.0,metric_start=10.0,extra_args=extra)
    if r.status!="ok" or not r.diag_csv: return (terr,amp,sd,"fail",float('nan'),float('nan'))
    d=pd.read_csv(r.diag_csv); t=pd.to_numeric(d["sim_time"],errors="coerce"); n=pd.to_numeric(d["n_terrain_est"],errors="coerce")
    tail=(t>=12)&np.isfinite(n)
    err=abs(float(n[tail].mean())-TN[terr]) if tail.any() else float('nan')
    cte=float('nan')
    for col in ("rms_cte","cte","cross_track_error","lateral_error"):
        if col in d.columns:
            cc=pd.to_numeric(d[col],errors="coerce")[t>=10];
            if np.isfinite(cc).any(): cte=float(np.sqrt(np.nanmean(cc**2)))
            break
    return (terr,amp,sd,"ok",err,cte)
res=[task(cells[0])]  # prewarm
with ProcessPoolExecutor(max_workers=6) as ex:
    futs={ex.submit(task,c):c for c in cells[1:]}
    for f in as_completed(futs): res.append(f.result())
agg=collections.defaultdict(list); cteagg=collections.defaultdict(list)
for terr,amp,sd,st,err,cte in res:
    if st=="ok" and err==err: agg[(terr,amp)].append(err)
    if st=="ok" and cte==cte: cteagg[(terr,amp)].append(cte)
print("\nFIXED NN-UKF closed-loop |dn| vs steering-probe amplitude:")
print(f"{'terrain':8}{'amp':>6}{'|dn|':>8}{'rms_cte':>9}{'N':>4}")
for terr in terrains:
    for amp in amps:
        v=agg[(terr,amp)]; ct=cteagg[(terr,amp)]
        m=statistics.mean(v) if v else float('nan')
        c=statistics.mean(ct) if ct else float('nan')
        print(f"{terr:8}{amp:6.2f}{m:8.3f}{c:9.4f}{len(v):4}")
