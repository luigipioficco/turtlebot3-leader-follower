"""Reads results/summary.csv and results/samples.csv (both appended to by
every run, see metrics_logger_node.py) and produces the comparison table and
the figures.

Runs are grouped by their base label (run_label, e.g. 'predictive_default'):
several repetitions of the same configuration are averaged, and the standard
deviation is reported, because with a noisy simulation a single run says
nothing.
"""
import argparse
import os
import sys

try:
    import pandas as pd
except ImportError:
    print("pandas is required:  pip3 install pandas --break-system-packages")
    sys.exit(1)


# The metrics reported, in the order they should be read: the first three are
# the ones committed to in the proposal; definitive_losses separates the
# losses the recovery resolved from the rest.
NUMERIC = ['duration_s', 'tracking_loss_events', 'definitive_losses',
           'mean_recovery_time_s', 'rmse_distance_m', 'collisions']

LABELS = {
    'tracking_loss_events': 'target losses',
    'definitive_losses': 'definitive losses (SEARCH)',
    'mean_recovery_time_s': 'mean recovery time [s]',
    'rmse_distance_m': 'RMSE of measured distance [m]',
    'collisions': 'collisions',
}


def mean_std(s):
    v = pd.to_numeric(s, errors='coerce').dropna()
    if v.empty:
        return ''
    return f"{v.mean():.3f}" if len(v) == 1 else f"{v.mean():.3f} \u00b1 {v.std():.3f}"


def build_table(summary):
    grouped = summary.groupby('run_label', sort=True)
    table = grouped.agg(n_runs=('run_id', 'count'),
                        **{c: (c, mean_std) for c in NUMERIC if c in summary.columns})
    return table.reset_index().rename(columns={'run_label': 'configuration'})


def plot_timeseries(samples, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # one series per configuration (the first run_id found for it)
    base = samples['run_id'].str.rsplit('_', n=2).str[0]
    first_run = samples.assign(base=base).groupby('base')['run_id'].first()

    fig, ax = plt.subplots(figsize=(11, 4))
    for lab, run_id in sorted(first_run.items()):
        df = samples[samples['run_id'] == run_id]
        ax.plot(df['t'], df['meas_distance'], label=lab, linewidth=1.2)

    ax.axhline(1.5, color='k', linestyle='--', linewidth=0.8,
               label='desired distance')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('measured distance [m]')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    p = os.path.join(out_dir, 'timeseries.png')
    fig.savefig(p, dpi=150)
    plt.close(fig)
    print(f"figure: {p}")


def plot_bars(table, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    metrics = [m for m in
              ['tracking_loss_events', 'definitive_losses',
               'mean_recovery_time_s', 'rmse_distance_m']
              if m in table.columns]
    if not metrics:
        return

    fig, axes = plt.subplots(1, len(metrics), figsize=(4 * len(metrics), 4))
    if len(metrics) == 1:
        axes = [axes]
    labels = table['configuration'].tolist()

    for ax, m in zip(axes, metrics):
        vals, errs = [], []
        for v in table[m]:
            if isinstance(v, str) and '\u00b1' in v:
                a, b = v.split('\u00b1')
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

    summary_path = os.path.join(args.folder, 'summary.csv')
    if not os.path.isfile(summary_path):
        print(f"file not found: {summary_path}")
        return 1

    summary = pd.read_csv(summary_path)
    if summary.empty:
        print(f"{summary_path} is empty")
        return 1

    table = build_table(summary)
    out_csv = os.path.join(args.folder, 'comparison_table.csv')
    table.to_csv(out_csv, index=False)

    print()
    print(table.to_string(index=False))
    print()
    print(f"table: {out_csv}")

    if not args.no_plots:
        try:
            samples_path = os.path.join(args.folder, 'samples.csv')
            if os.path.isfile(samples_path):
                plot_timeseries(pd.read_csv(samples_path), args.folder)
            plot_bars(table, args.folder)
        except ImportError:
            print("matplotlib not installed, figures skipped "
                  "(pip3 install matplotlib --break-system-packages)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
