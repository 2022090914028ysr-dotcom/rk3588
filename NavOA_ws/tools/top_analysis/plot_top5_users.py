#!/usr/bin/env python3
import os
import argparse
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_top5_cpu_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        return pd.DataFrame()
    df = pd.read_csv(path)
    # Expected columns: time, rank, pid, user, cpu_pct, rss_MB, cmd
    # Normalize types
    if 'time' in df.columns:
        # Parse time-of-day consistently to avoid warnings
        df['dt'] = pd.to_datetime(df['time'], format='%H:%M:%S', errors='coerce')
    else:
        df['dt'] = pd.NaT
    for col in ['cpu_pct', 'rss_MB']:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def _plot_wide(wide: pd.DataFrame, labels: list[str], out_dir: str, out_name: str = 'top5_users_cpu.png') -> str:
    plt.figure(figsize=(10, 5))
    for label in labels:
        if label in wide.columns:
            plt.plot(wide.index.values, wide[label].values, label=str(label))

    plt.title('CPU Usage Over Time — Labels (top-5 rows per tick)')
    plt.xlabel('Time')
    plt.ylabel('CPU % (sum per label)')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right', ncol=1, fontsize=9)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, out_name)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    return out_path


def _normalize_cmd(cmd: str) -> str:
    c = (cmd or '').lower()
    if c.startswith('python3') or c == 'python3':
        return 'python'
    if c.startswith('python'):
        return 'python'
    return cmd


def plot_users_cpu(df: pd.DataFrame, out_dir: str, top_n: int = 5, names: list[str] | None = None, by: str = 'user') -> str | None:
    if df.empty:
        return None
    # If specific names provided, build label-based aggregation (match user exact or cmd contains)
    if names:
        # Prepare per-label per-timestamp sums
        labels = [n.strip() for n in names if n.strip()]
        # Aggregate per timestamp for faster matching
        df2 = df.copy()
        df2['user'] = df2['user'].astype(str)
        df2['cmd'] = df2['cmd'].astype(str)
        # Build series per label
        frames = []
        for label in labels:
            # Match either user exact (case-insensitive) or command contains label
            mask = df2['user'].str.lower().eq(label.lower()) | df2['cmd'].str.lower().str.contains(label.lower())
            s = df2.loc[mask, ['dt', 'cpu_pct']].groupby('dt', as_index=True)['cpu_pct'].sum()
            frames.append(s.rename(label))
        # Combine into wide
        wide = pd.concat(frames, axis=1).sort_index().fillna(0.0)
        return _plot_wide(wide, labels, out_dir, out_name='top5_users_cpu.png')

    # Otherwise: aggregate by requested dimension
    if by == 'cmd':
        df2 = df.copy()
        df2['cmd_norm'] = df2['cmd'].astype(str).map(_normalize_cmd)
        agg = df2.groupby(['dt', 'cmd_norm'], as_index=False).agg(cpu_label=('cpu_pct', 'sum'))
        totals = agg.groupby('cmd_norm', as_index=False)['cpu_label'].sum().sort_values('cpu_label', ascending=False)
        top_labels = totals['cmd_norm'].head(top_n).tolist()
        wide = agg[agg['cmd_norm'].isin(top_labels)].pivot(index='dt', columns='cmd_norm', values='cpu_label').sort_index().fillna(0.0)
        return _plot_wide(wide, top_labels, out_dir, out_name=f'top{top_n}_users_cpu.png')
    else:
        agg = df.groupby(['dt', 'user'], as_index=False).agg(cpu_label=('cpu_pct', 'sum'))
        totals = agg.groupby('user', as_index=False)['cpu_label'].sum().sort_values('cpu_label', ascending=False)
        top_labels = totals['user'].head(top_n).tolist()
        wide = agg[agg['user'].isin(top_labels)].pivot(index='dt', columns='user', values='cpu_label').sort_index().fillna(0.0)
        return _plot_wide(wide, top_labels, out_dir, out_name=f'top{top_n}_users_cpu.png')


def plot_ranking(df: pd.DataFrame, labels: list[str], by: str, out_dir: str, out_name: str) -> str:
    # Compute total CPU per label across the run
    if by == 'cmd':
        df2 = df.copy()
        df2['cmd_norm'] = df2['cmd'].astype(str).map(_normalize_cmd)
        totals = df2[df2['cmd_norm'].isin(labels)].groupby('cmd_norm', as_index=False)['cpu_pct'].sum()
        totals = totals.sort_values('cpu_pct', ascending=True)  # ascending for level-up bars
        names = totals['cmd_norm'].tolist()
        values = totals['cpu_pct'].tolist()
    else:
        totals = df[df['user'].isin(labels)].groupby('user', as_index=False)['cpu_pct'].sum()
        totals = totals.sort_values('cpu_pct', ascending=True)
        names = totals['user'].tolist()
        values = totals['cpu_pct'].tolist()

    os.makedirs(out_dir, exist_ok=True)
    plt.figure(figsize=(8, 6))
    plt.barh(names, values)
    plt.xlabel('Total CPU % (sum over run)')
    plt.title(f'Top {len(labels)} Labels — Total CPU')
    plt.grid(True, axis='x', alpha=0.3)
    out_path = os.path.join(out_dir, out_name)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    return out_path


def main():
    ap = argparse.ArgumentParser(description='Plot CPU comparison for labels appearing in top-5 per tick (users or commands)')
    ap.add_argument('--csv', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/tools/system_monitor_top5_cpu.csv'), help='Path to system_monitor_top5_cpu.csv')
    ap.add_argument('--out', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/output/top_plots'), help='Output directory')
    ap.add_argument('--topN', type=int, default=5, help='Number of labels to compare if names not provided')
    ap.add_argument('--names', default='', help='Comma-separated labels to plot (matches user exact or cmd contains), e.g. "cartographer,python3,robit,ubuntu,root"')
    ap.add_argument('--by', choices=['user', 'cmd'], default='cmd', help='Aggregate by user or command when names not provided')
    args = ap.parse_args()

    df = read_top5_cpu_csv(args.csv)
    if df.empty:
        print('(No top-5 CPU CSV data found)')
        return
    names = [x.strip() for x in args.names.split(',')] if args.names else None
    p = plot_users_cpu(df, args.out, args.topN, names, args.by)
    if p:
        print(f'Users CPU comparison plot saved: {p}')
        # Also emit a ranking bar chart when auto-selecting labels (no names provided)
        if not names:
            # Recompute selected labels to match the time-series output name
            if args.by == 'cmd':
                df2 = df.copy()
                df2['cmd_norm'] = df2['cmd'].astype(str).map(_normalize_cmd)
                agg = df2.groupby(['dt', 'cmd_norm'], as_index=False).agg(cpu_label=('cpu_pct', 'sum'))
                totals = agg.groupby('cmd_norm', as_index=False)['cpu_label'].sum().sort_values('cpu_label', ascending=False)
                top_labels = totals['cmd_norm'].head(args.topN).tolist()
            else:
                agg = df.groupby(['dt', 'user'], as_index=False).agg(cpu_label=('cpu_pct', 'sum'))
                totals = agg.groupby('user', as_index=False)['cpu_label'].sum().sort_values('cpu_label', ascending=False)
                top_labels = totals['user'].head(args.topN).tolist()

            rp = plot_ranking(df, top_labels, args.by, args.out, out_name=f'top{args.topN}_users_rank.png')
            print(f'Users CPU ranking plot saved: {rp}')


if __name__ == '__main__':
    main()
