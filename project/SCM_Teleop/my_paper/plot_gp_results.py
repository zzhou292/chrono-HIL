from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

PAPER_FIGS = Path(__file__).resolve().parent / "paper_figures"

df = pd.read_csv(PAPER_FIGS / "dynamics_gp_revalidation_summary.csv")

df_plot = df[df['run'].isin(['baseline', 'loaded'])]

# Create a figure with 2 rows (paths) and 2 columns (metrics)
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

for i, path in enumerate(['sinusoidal', 'right_left']):
    df_path = df_plot[df_plot['path'] == path]
    
    sns.barplot(data=df_path, x='terrain', y='path_err_m_mean', hue='run', ax=axes[i, 0])
    axes[i, 0].set_title(f'Path Tracking Error - {path.capitalize()}')
    axes[i, 0].set_ylabel('Error (m)')
    
    sns.barplot(data=df_path, x='terrain', y='frenet_lat_rms_m_mean', hue='run', ax=axes[i, 1])
    axes[i, 1].set_title(f'Frenet Lateral RMS Error - {path.capitalize()}')
    axes[i, 1].set_ylabel('Error (m)')

plt.tight_layout()
out_path = PAPER_FIGS / "dynamics_gp_revalidation.png"
plt.savefig(out_path, dpi=300)
print(f"Saved {out_path}")
