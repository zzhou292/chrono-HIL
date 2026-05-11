# Shared Verification

This directory holds bundle-specific verification scripts and generated notes.

Recommended workflow:

1. `bash verification/run_shared_verification.sh`
2. Inspect `my_paper/paper_figures/`, `logs/`, and `plots/`
3. Record the exact outcomes in `TRACKING.md`

The script is intentionally conservative: it runs compile checks first, then a
small but paper-relevant set of estimator and benchmark jobs from this shared
copy.
