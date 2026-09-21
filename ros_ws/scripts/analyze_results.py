"""Reads results/summary.csv (appended to by every run, see
metrics_logger_node.py) and produces the comparison table.

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default='/root/ros_workspace/results')
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
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
