import sys, itertools, collections, statistics, math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np, pandas as pd
sys.path.insert(0,"benchmarking")
from common import launch_and_collect
TN={"clay":0.5,"dirt":0.7,"sand":1.1}
terrains=["clay","dirt","sand"]; speeds=[5.0,7.0]; seeds=[720,721,722]
backends={"MLP(deployed)":"learned","NN-UKF(live)":"nn_ukf"}
cells=[(bk,terr,sp,sd) for bk in backends for terr in terrains for sp in speeds for sd in seeds]
def task(c):
    bk,terr,sp,sd=c; idx=abs(hash(c))%4000; port=14000+2*idx
    extra=["--terrain-estimator","--terrain-estimator-backend",backends[bk],"--terrain-estimator-mode","n",
           "--te-update-interval","8","--te-min-confidence","0.0"]
    r=launch_and_collect(experiment="cl_estimator",variant=bk,controller_mode="standard",mpc_model="nn",
        nn_model="vehicle_rate_64_32_lhs",terrain=terr,path="sinusoidal",speed=sp,bumpiness=0,seed=sd,
        run_dir=Path(f"/tmp/ttrans/clest/{bk}_{terr}_v{sp:g}_s{sd}"),sim_port=port,ctrl_port=port+1,
        sim_time=22.0,timeout=400.0,lead_in=5.0,metric_start=10.0,extra_args=extra)
    if r.status!="ok" or not r.diag_csv: return (bk,terr,sp,sd,"fail",float('nan'))
    d=pd.read_csv(r.diag_csv); t=pd.to_numeric(d["sim_time"],errors="coerce"); n=pd.to_numeric(d["n_terrain_est"],errors="coerce")
    tail=(t>=12)&np.isfinite(n)
    err=abs(float(n[tail].mean())-TN[terr]) if tail.any() else float('nan')
    return (bk,terr,sp,sd,"ok",err)
res=[task(cells[0]),task(cells[len(cells)//2])]  # prewarm both backends' acados (same tire model, so 1 build)
done=set(id(x) for x in [cells[0],cells[len(cells)//2]])
rem=[c for i,c in enumerate(cells) if i not in (0,len(cells)//2)]
with ProcessPoolExecutor(max_workers=6) as ex:
    futs={ex.submit(task,c):c for c in rem}
    for f in as_completed(futs): res.append(f.result())
agg=collections.defaultdict(lambda: collections.defaultdict(list))
for bk,terr,sp,sd,st,err in res:
    if st=="ok" and err==err: agg[bk][terr].append(err)
print("\nCLOSED-LOOP in-loop |dn| (tail-window), live estimators:")
print(f"{'backend':16}{'clay':>8}{'dirt':>8}{'sand':>8}{'ALL':>8}{'n':>5}")
for bk in backends:
    row=[]; allv=[]
    for terr in terrains:
        v=agg[bk][terr]; row.append(statistics.mean(v) if v else float('nan')); allv+=v
    am=statistics.mean(allv) if allv else float('nan')
    print(f"{bk:16}{row[0]:8.3f}{row[1]:8.3f}{row[2]:8.3f}{am:8.3f}{len(allv):5}")
