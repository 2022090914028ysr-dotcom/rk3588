#!/usr/bin/env python3
import re
import os
from datetime import datetime
import argparse
from collections import defaultdict

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Regexes to detect sections and time
CPU_HEADER_RE = re.compile(r"UID\s+PID\s+%usr\s+%system\s+%guest\s+%wait\s+%CPU\s+CPU\s+Command")
MEM_HEADER_RE = re.compile(r"UID\s+PID\s+minflt/s\s+majflt/s\s+VSZ\s+RSS\s+%MEM\s+Command")
TIME_RE = re.compile(r"^(?P<time>\d{1,2}:\d{2}:\d{2})\s+(?P<ampm>AM|PM)\b")
DATE_HDR_RE = re.compile(r"Linux\s+\S+\s+\([^)]*\)\s+(?P<date>\d{2}/\d{2}/\d{4})")


def parse_pidstat_log(path: str) -> pd.DataFrame:
    """Parse pidstat -u -r log and aggregate by timestamp.
    Returns DataFrame with:
      - time
            - cpu_total, cpu_top1, cpu_top5_sum, cpu_top10_sum, procs_cpu
      - mem_rss_total_mb, mem_top5_rss_mb, procs_mem
    """
    section = None
    date_str = None

    cpu_values = defaultdict(list)  # time -> list of %CPU floats
    mem_values = defaultdict(list)  # time -> list of RSS (KB) ints

    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = raw.rstrip('\n')
            if not line:
                continue
            # Date header (grab date for datetime composition)
            if date_str is None:
                m_date = DATE_HDR_RE.search(line)
                if m_date:
                    date_str = m_date.group('date')  # e.g., 12/23/2025
                    continue

            # Section headers
            if CPU_HEADER_RE.search(line):
                section = 'cpu'
                continue
            if MEM_HEADER_RE.search(line):
                section = 'mem'
                continue

            # Data rows start with time
            m_time = TIME_RE.match(line)
            if not m_time:
                continue
            ts_text = f"{m_time.group('time')} {m_time.group('ampm')}"
            # Compose datetime
            # If date unknown, use today's date
            date_part = date_str or datetime.now().strftime('%m/%d/%Y')
            dt = datetime.strptime(f"{date_part} {ts_text}", "%m/%d/%Y %I:%M:%S %p")

            # Tokenize row
            tokens = line.split()
            # tokens[0]=HH:MM:SS, tokens[1]=AM/PM
            if section == 'cpu':
                if len(tokens) < 11:
                    continue
                try:
                    # indices: 2 UID, 3 PID, 4 %usr, 5 %system, 6 %guest, 7 %wait, 8 %CPU, 9 CPU, 10+ Command
                    cpu_pct = float(tokens[8])
                except Exception:
                    cpu_pct = 0.0
                cpu_values[dt].append(cpu_pct)
            elif section == 'mem':
                if len(tokens) < 10:
                    continue
                try:
                    # indices: 2 UID, 3 PID, 4 minflt/s, 5 majflt/s, 6 VSZ, 7 RSS, 8 %MEM, 9+ Command
                    rss_kb = int(float(tokens[7]))  # some systems report as floats
                except Exception:
                    rss_kb = 0
                mem_values[dt].append(rss_kb)

    # Build aggregated dataframe
    times = sorted(set(cpu_values.keys()) | set(mem_values.keys()))
    rows = []
    for t in times:
        cpu_list = sorted(cpu_values.get(t, []), reverse=True)
        mem_list = sorted(mem_values.get(t, []), reverse=True)
        cpu_total = float(sum(cpu_list)) if cpu_list else 0.0
        cpu_top1 = float(cpu_list[0]) if cpu_list else 0.0
        cpu_top5_sum = float(sum(cpu_list[:5])) if cpu_list else 0.0
        cpu_top10_sum = float(sum(cpu_list[:10])) if cpu_list else 0.0
        procs_cpu = int(len(cpu_list))
        mem_total_mb = sum(mem_list) / 1024.0 if mem_list else 0.0
        mem_top5_mb = sum(mem_list[:5]) / 1024.0 if mem_list else 0.0
        procs_mem = int(len(mem_list))
        rows.append({
            'time': t,
            'cpu_total': cpu_total,
            'cpu_top1': cpu_top1,
            'cpu_top5_sum': cpu_top5_sum,
            'cpu_top10_sum': cpu_top10_sum,
            'procs_cpu': procs_cpu,
            'mem_rss_total_mb': mem_total_mb,
            'mem_top5_rss_mb': mem_top5_mb,
            'procs_mem': procs_mem,
        })
    return pd.DataFrame(rows).sort_values('time')


def plot_cpu(df: pd.DataFrame, out_dir: str) -> str:
    plt.figure(figsize=(12, 6))
    plt.plot(df['time'].values, df['cpu_total'].values, label='CPU total (%)')
    plt.plot(df['time'].values, df['cpu_top10_sum'].values, label='Top-10 sum (%)')
    plt.plot(df['time'].values, df['cpu_top5_sum'].values, label='Top-5 sum (%)')
    plt.plot(df['time'].values, df['cpu_top1'].values, label='Top-1 (%)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('CPU usage over time (pidstat)')
    plt.xlabel('Time')
    plt.ylabel('CPU %')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'pidstat_cpu.png')
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return path


def plot_mem(df: pd.DataFrame, out_dir: str) -> str:
    plt.figure(figsize=(12, 4))
    plt.plot(df['time'].values, df['mem_rss_total_mb'].values, label='RSS total (MB)')
    plt.plot(df['time'].values, df['mem_top5_rss_mb'].values, label='Top-5 RSS (MB)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title('Memory RSS over time (pidstat)')
    plt.xlabel('Time')
    plt.ylabel('MB')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'pidstat_mem.png')
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return path


def main():
    ap = argparse.ArgumentParser(description='Plot CPU and memory from pidstat log (-u -r).')
    ap.add_argument('--log', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/tools/top_analysis/pidstat.log'), help='Path to pidstat.log')
    ap.add_argument('--out', default=os.path.expanduser('~/Robit_SlamNav/NavOA_ws/output/top_plots'), help='Output directory')
    args = ap.parse_args()

    df = parse_pidstat_log(args.log)
    if df.empty:
        print('No data parsed from pidstat log. Check format or options.')
        return
    cpu_path = plot_cpu(df, args.out)
    mem_path = plot_mem(df, args.out)
    print('Plots saved:')
    print(f'- {cpu_path}')
    print(f'- {mem_path}')


if __name__ == '__main__':
    main()
