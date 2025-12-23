#!/usr/bin/env python3
import os
import argparse
from datetime import datetime, date

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        return pd.DataFrame()
    try:
        return pd.read_csv(path)
    except Exception:
        return pd.DataFrame()


def with_datetime(df: pd.DataFrame) -> pd.DataFrame:
    if df.empty or 'time' not in df.columns:
        return pd.DataFrame()
    today = date.today().isoformat()
    def to_dt(s: str):
        try:
            return datetime.strptime(f"{today} {s}", "%Y-%m-%d %H:%M:%S")
        except Exception:
            return pd.NaT
    out = df.copy()
    out['dt'] = out['time'].astype(str).map(to_dt)
    return out.dropna(subset=['dt'])


def plot_top5_cpu(df: pd.DataFrame, out_dir: str) -> str | None:
    if df.empty:
        return None
    df['cpu_pct'] = pd.to_numeric(df.get('cpu_pct'), errors='coerce').fillna(0.0)
    # Sum of top-5 CPU per timestamp; also show top-1
    agg = df.groupby('dt').agg(
        top5_sum=('cpu_pct', lambda s: s.nlargest(5).sum()),
        top1=('cpu_pct', 'max'),
    ).reset_index()

    plt.figure(figsize=(12, 4))
    plt.plot(agg['dt'].values, agg['top5_sum'].values, label='Top-5 CPU sum (%)')
    plt.plot(agg['dt'].values, agg['top1'].values, label='Top-1 CPU (%)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('Top-5 CPU usage per interval')
    plt.xlabel('Time')
    plt.ylabel('CPU %')
    os.makedirs(out_dir, exist_ok=True)
    p = os.path.join(out_dir, 'top5_cpu.png')
    plt.tight_layout()
    plt.savefig(p)
    plt.close()
    return p


def plot_top5_mem(df: pd.DataFrame, out_dir: str) -> str | None:
    if df.empty:
        return None
    df['rss_MB'] = pd.to_numeric(df.get('rss_MB'), errors='coerce').fillna(0.0)
    agg = df.groupby('dt').agg(
        top5_sum_mb=('rss_MB', lambda s: s.nlargest(5).sum()),
        top1_mb=('rss_MB', 'max'),
    ).reset_index()

    plt.figure(figsize=(12, 4))
    plt.plot(agg['dt'].values, agg['top5_sum_mb'].values, label='Top-5 RSS sum (MB)')
    plt.plot(agg['dt'].values, agg['top1_mb'].values, label='Top-1 RSS (MB)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('Top-5 Memory (RSS) per interval')
    plt.xlabel('Time')
    plt.ylabel('MB')
    os.makedirs(out_dir, exist_ok=True)
    p = os.path.join(out_dir, 'top5_mem.png')
    plt.tight_layout()
    plt.savefig(p)
    plt.close()
    return p


def main():
    ap = argparse.ArgumentParser(description='Plot top-5 CPU and memory from system_monitor_top5_* CSVs')
    ap.add_argument('--cpu', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/tools/system_monitor_top5_cpu.csv'), help='Top-5 CPU CSV path')
    ap.add_argument('--mem', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/tools/system_monitor_top5_mem.csv'), help='Top-5 memory CSV path')
    ap.add_argument('--out', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/output/top_plots'), help='Output directory')
    args = ap.parse_args()

    cpu_df = with_datetime(read_csv(args.cpu))
    mem_df = with_datetime(read_csv(args.mem))

    cpu_p = plot_top5_cpu(cpu_df, args.out)
    mem_p = plot_top5_mem(mem_df, args.out)

    print('Top-5 plots saved:')
    if cpu_p:
        print(f'- {cpu_p}')
    if mem_p:
        print(f'- {mem_p}')
    if not any([cpu_p, mem_p]):
        print('(No top-5 CSV data found)')


if __name__ == '__main__':
    main()
