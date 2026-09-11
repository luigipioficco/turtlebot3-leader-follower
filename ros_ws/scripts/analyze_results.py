"""Reads the CSV summaries of every run and produces the comparison table
and the figures.

When a folder holds several repetitions of the same configuration, they are
averaged and the standard deviation is reported: with a noisy simulation a
single run says nothing.
"""
import argparse
import glob
import os
import sys
from collections import defaultdict

import numpy as np

try:
    import pandas as pd
except ImportError:
    print("pandas is required:  pip3 install pandas --break-system-packages")
    sys.exit(1)


# The metrics reported, in the order they should be read:
#   the first three are the ones committed to in the proposal
#   definitive_losses separates the losses the recovery resolved from the rest
# The four metrics committed to in Phase 6 of the proposal, plus the
# breakdown of the definitive losses.
NUMERIC = ['duration_s', 'tracking_loss_events', 'definitive_losses',
           'mean_recovery_time_s', 'rmse_distance_m', 'collisions']

LABELS = {
    'tracking_loss_events': 'target losses',
    'definitive_losses': 'definitive losses (SEARCH)',
    'mean_recovery_time_s': 'mean recovery time [s]',
    'rmse_distance_m': 'RMSE of measured distance [m]',
    'collisions': 'collisioni',
}


def base_label(path):
    """run_label without the timestamp: predictive_loop_20260725_101530 ->
    predictive_stairs"""
    name = os.path.basename(path).replace('_summary.csv', '')
    parts = name.split('_')
    if len(parts) >= 3 and parts[-1].isdigit() and parts[-2].isdigit():
        return '_'.join(parts[:-2])
    return name


def load_summaries(folder):
    groups = defaultdict(list)
    for f in sorted(glob.glob(os.path.join(folder, '*_summary.csv'))):
        df = pd.read_csv(f)
        if df.empty:
            continue
        groups[base_label(f)].append(df.iloc[0])
    return groups


def build_table(groups):
    rows = []
    for label, entries in sorted(groups.items()):
        row = {'configurazione': label, 'n_prove': len(entries)}
        for col in NUMERIC:
            vals = []
            for e in entries:
                if col in e and pd.notna(e[col]) and e[col] != '':
                    try:
                        vals.append(float(e[col]))
                    except (TypeError, ValueError):
                        pass
            if not vals:
                row[col] = ''
                continue
            m = float(np.mean(vals))
            row[col] = f"{m:.3f}" if len(vals) == 1 else f"{m:.3f} ± {np.std(vals):.3f}"
        rows.append(row)
    return pd.DataFrame(rows)


def plot_timeseries(folder, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    files = sorted(glob.glob(os.path.join(folder, '*_samples.csv')))
    if not files:
        return

    # one series per configuration (the first repetition found)
    seen = {}
    for f in files:
        lab = base_label(f.replace('_samples.csv', '_summary.csv'))
        seen.setdefault(lab, f)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for lab, f in sorted(seen.items()):
        df = pd.read_csv(f)
        axes[0].plot(df['t'], df['meas_distance'], label=lab, linewidth=1.2)

    axes[0].axhline(1.0, color='k', linestyle='--', linewidth=0.8,
                    label='distanza desiderata')
    axes[0].set_ylabel('distanza misurata [m]')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].set_xlabel('tempo [s]')
    axes[1].set_yticks([0, 1])
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    p = os.path.join(out_dir, 'timeseries.png')
    fig.savefig(p, dpi=150)
    plt.close(fig)
    print(f"figure: {p}")


def plot_bars(table, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    metrics = ['tracking_loss_events', 'definitive_losses',
               'mean_recovery_time_s', 'rmse_distance_m']
    metrics = [m for m in metrics if m in table.columns]
    if not metrics:
        return

    fig, axes = plt.subplots(1, len(metrics), figsize=(4 * len(metrics), 4))
    if len(metrics) == 1:
        axes = [axes]
    labels = table['configurazione'].tolist()

    for ax, m in zip(axes, metrics):
        vals, errs = [], []
        for v in table[m]:
            if isinstance(v, str) and '±' in v:
                a, b = v.split('±')
                vals.append(float(a))
                errs.append(float(b))
            elif v == '' or pd.isna(v):
                vals.append(0.0)
                errs.append(0.0)
            else:
                vals.append(float(v))
                errs.append(0.0)
        ax.bar(range(len(vals)), vals, yerr=errs, capsize=4,
               color=['#c44' if 'react' in lab else '#48a' for lab in labels])
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=30, ha='right', fontsize=8)
        ax.set_title(LABELS.get(m, m), fontsize=10)
        ax.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    p = os.path.join(out_dir, 'comparison.png')
    fig.savefig(p, dpi=150)
    plt.close(fig)
    print(f"figure: {p}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default='/root/ros_workspace/results')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    if not os.path.isdir(args.folder):
        print(f"folder not found: {args.folder}")
        return 1

    groups = load_summaries(args.folder)
    if not groups:
        print(f"nessun file *_summary.csv in {args.folder}")
        return 1

    table = build_table(groups)
    out_csv = os.path.join(args.folder, 'comparison_table.csv')
    table.to_csv(out_csv, index=False)

    print()
    print(table.to_string(index=False))
    print()
    print(f"table: {out_csv}")

    if not args.no_plots:
        try:
            plot_timeseries(args.folder, args.folder)
            plot_bars(table, args.folder)
        except ImportError:
            print("matplotlib not installed, figures skipped "
                  "(pip3 install matplotlib --break-system-packages)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
