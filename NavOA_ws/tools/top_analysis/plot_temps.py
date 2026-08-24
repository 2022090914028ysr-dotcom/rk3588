#!/usr/bin/env python3
import os
import argparse
from datetime import datetime, date

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_monitor_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        return pd.DataFrame()
    df = pd.read_csv(path)
    if df.empty or 'time' not in df.columns:
        return pd.DataFrame()
    # Build datetime from today's date + HH:MM:SS
    today = date.today().isoformat()
    def to_dt(s: str):
        try:
            return datetime.strptime(f"{today} {s}", "%Y-%m-%d %H:%M:%S")
        except Exception:
            return pd.NaT
    df['dt'] = df['time'].astype(str).map(to_dt)
    # Numeric conversions and NA handling
    for c in ['cpu_temp_C', 'npu_temp_C', 'total_cpu_pct', 'npu_usage_pct']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce')
    return df.dropna(subset=['dt'])


def plot_temps(df: pd.DataFrame, out_dir: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    fig, ax1 = plt.subplots(figsize=(12, 5))

    if 'cpu_temp_C' in df.columns:
        ax1.plot(df['dt'].values, df['cpu_temp_C'].values, label='CPU temp (C)', color='tab:red')
    if 'npu_temp_C' in df.columns:
        ax1.plot(df['dt'].values, df['npu_temp_C'].values, label='NPU temp (C)', color='tab:orange')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Temperature (°C)')
    ax1.grid(True, alpha=0.3)

    # Overlay usage percentages (total CPU% and NPU usage%) on a twin axis
    has_pct = ('total_cpu_pct' in df.columns and df['total_cpu_pct'].notna().any()) or \
              ('npu_usage_pct' in df.columns and df['npu_usage_pct'].notna().any())
    if has_pct:
        ax2 = ax1.twinx()
        if 'total_cpu_pct' in df.columns and df['total_cpu_pct'].notna().any():
            ax2.plot(df['dt'].values, df['total_cpu_pct'].values, label='Total CPU %', color='tab:blue', alpha=0.5)
        if 'npu_usage_pct' in df.columns and df['npu_usage_pct'].notna().any():
            ax2.plot(df['dt'].values, df['npu_usage_pct'].values, label='NPU usage %', color='tab:green', alpha=0.8)
        ax2.set_ylabel('Usage %')
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    else:
        ax1.legend(loc='upper left')

    path = os.path.join(out_dir, 'temps.png')
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(description='Plot temperatures (CPU/NPU) and optional total CPU % from system_monitor.csv')
    ap.add_argument('--csv', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/tools/system_monitor.csv'), help='Path to system_monitor.csv')
    ap.add_argument('--out', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/output/top_plots'), help='Output directory')
    args = ap.parse_args()

    df = read_monitor_csv(args.csv)
    if df.empty:
        print('No data found in system_monitor.csv; run system_monitor.sh first.')
        return
    p = plot_temps(df, args.out)
    print(f'Temperature plot saved: {p}')


if __name__ == '__main__':
    main()
