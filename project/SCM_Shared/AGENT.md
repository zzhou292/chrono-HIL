So I'm trying to write a paper on safe shared/teleopoerated control and autonomous contorl of humvees on deformable terrain (clay, dirt, sand). The target paper so far is writen in chrono-HIL/project/SCM_Teleop/my_paper/abstract.tex. Feel free to edit that any time you believe necessary. First some rules

1. Update Tracking.md with most up to date thoughts, edits, findings, bugs, results, open questions
2. Never claim anything works or is good until it has been tested with a simulation run.
3. We want to make this code/architecture work in real life eventually, so never assume you can get oracle like data from the sim. LIke if we're trying to train a terrain classifier, during trainiign it's ok to extract forces from chrono, but we can't use that during inference
4. Don't delete entire files. Just move them to the archive folder
5. Avoid using band-aids or workarounds or magic numbers to fix things. Please diagnose root problems and fix things correctly/cannonicaly. Always think "if a paper reviewer saw this code, would they be surprised or offended."
6. Don't stop until all the code is ready and is sufficient for the paper.

Things to note:

1. We're using project chrono as our simulator. It is what provides the humvee and SCM terrain. We are using the chrono that is here: /home/kyle/Documents/chrono_fork/chrono. You can look at this to understnad the API and what the simulator is actually doing. We are calling it in this codebase via SWIG wrapper with python.
2. The correct environment to execute code in is a conda env called "sim"
3. The paper_v1 series of models in SCM_Teleop/nn_models are deprecated
4. The paper_v2 "static" models are trained off of SCM_Teleop/data/normal_mlp_resnet/scm_static_100k_v4.csv while the "rate" ones trained off of SCM_Teleop/data/rate_mlp_resnet/rate_v1_100k.csv
5. If you generate datasets or images that are good enough for the final paper, please put them in my_paper/paper_figures. Need the dataset and image.
6. The active paper scope now keeps only the sliding-window MLP terrain-estimation path (`n` and optional joint `n`/`phi`), the force residual adapter, and the dynamics GP. Older UKF/hybrid estimators, observer experiments, and GP force-residual code belong in `archive/`.
7. Keep the filesystem layout clean: data collection and benchmark helpers belong in `utilities/`, validation entrypoints in `test_suite/`, diagnostics in `new_diagnostics/`, and paper-specific figure builders/assets in `my_paper/`.
