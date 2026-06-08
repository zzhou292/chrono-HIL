# paper_figure_data — figure-backing CSVs (reproduce figures without re-running Chrono)

Every figure/table in `my_paper/paper.tex` is plotted from a small result CSV,
not from the multi-GB raw Chrono logs. Those CSVs (≈17 MB total) are committed
here so the figures can be **regenerated or inspected without** re-running the
~12 h benchmark suite or shipping the gitignored `data/` (traces, per-run logs).

Each `<prefix>_results.csv` is the merged per-run results table for that sweep
(one row per Chrono run, deduped on the run key). To re-plot a figure, call the
owning benchmark module's `plot_figures(results_csv, out_dir)` — e.g.:

    python -c "import sys; sys.path.insert(0,'benchmarking'); \
      import terrain_transition_benchmark as m, pathlib; \
      m.plot_figures(pathlib.Path('paper_figure_data/terrain_transition_results.csv'), pathlib.Path('/tmp/repro'))"

Mapping (CSV -> paper figure -> plotter module):
- closed_loop_estimator_backends.csv  -> closed-loop estimator §IV  (benchmarking/closed_loop_estimator_compare.py)
- terrain_transition_results.csv / _ol_results.csv -> transition figs (terrain_transition_benchmark.py)
- closed_loop_estimator_results.csv   -> §IV-A estimator scatter     (terrain_estimator_benchmark.py)
- safety_filter*_results.csv, dob_cbf_nn_ablation_results.csv, mppi_seed_ablation_results.csv,
  autonomous_obstacle_results.csv, latency_compensation_results.csv, throttle_dob_results.csv -> §VI/§VII
- bench_tire_models* / lhs100* -> §III / §IV-B tire+estimator benches

What is NOT here (regenerate via the collectors if needed): raw SCM logs,
per-run diag CSVs, training datasets — all gitignored under data/.
