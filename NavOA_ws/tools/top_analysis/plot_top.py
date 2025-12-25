#!/usr/bin/env python3
import re
import os
from datetime import datetime, date
import argparse

import pandas as pd
import matplotlib.pyplot as plt


ts_re = re.compile(r"top - (?P<hms>\d{2}:\d{2}:\d{2}).*load average: (?P<l1>[0-9.]+),\s*(?P<l5>[0-9.]+),\s*(?P<l15>[0-9.]+)")


def parse_top_csv(path: str) -> pd.DataFrame:
    """Parse the per-process top CSV to time-aligned aggregates.
    Returns a DataFrame with columns:
    - time (datetime)
    - load1, load5, load15 (float)
    - cpu_total (float): sum of per-process CPU% at that timestamp
    - cpu_top1 (float): max per-process CPU% at that timestamp
    - cpu_top5_sum (float): sum of top 5 per-process CPU% at that timestamp
    - procs (int): number of processes in snapshot
    """
    df = pd.read_csv(path)
    # Normalize column names commonly seen in your CSV
    # Expect columns: TIMESTAMP, PID, USER, PR, NI, VIRT, RES, SHR, S, CPU, MEM, TIME, COMMAND
    # Filter out non-numeric PID rows (like the repeated header row)
    df = df[pd.to_numeric(df["PID"], errors="coerce").notna()].copy()

    # Extract time and load averages from TIMESTAMP string
    parsed = df["TIMESTAMP"].str.extract(ts_re)
    # Drop rows that failed to parse timestamp/load
    df = df[parsed.notna().all(axis=1)].copy()
    parsed = parsed.loc[df.index]

    # Build a datetime using today's date + time-of-day from top
    today = date.today()
    df["time"] = parsed["hms"].apply(lambda s: datetime.strptime(f"{today.isoformat()} {s}", "%Y-%m-%d %H:%M:%S"))
    df["load1"] = parsed["l1"].astype(float)
    df["load5"] = parsed["l5"].astype(float)
    df["load15"] = parsed["l15"].astype(float)

    # CPU column to float
    # In your CSV it's named CPU and uses values like 105.6 etc.
    df["CPU"] = pd.to_numeric(df["CPU"], errors="coerce").fillna(0.0)

    # Aggregate per timestamp
    agg = (
        df
        .sort_values(["time", "CPU"], ascending=[True, False])
        .groupby("time")
        .agg(
            load1=("load1", "first"),
            load5=("load5", "first"),
            load15=("load15", "first"),
            cpu_total=("CPU", "sum"),
            cpu_top1=("CPU", "max"),
            cpu_top5_sum=("CPU", lambda s: s.nlargest(5).sum()),
            procs=("CPU", "size"),
        )
        .reset_index()
    )
    return agg


def maybe_read_psi_csv(path: str) -> pd.DataFrame | None:
    if not os.path.exists(path):
        return None
    psi = pd.read_csv(path, parse_dates=["time"])  # columns: time, cpu_some_avg10, cpu_some_avg60, cpu_some_avg300, io_some_*, mem_some_*
    return psi


def plot_cpu_series(agg: pd.DataFrame, out_dir: str):
    plt.figure(figsize=(12, 6))
    plt.plot(agg["time"], agg["cpu_total"], label="CPU total (sum of per-proc %)")
    plt.plot(agg["time"], agg["cpu_top5_sum"], label="Top-5 sum")
    plt.plot(agg["time"], agg["cpu_top1"], label="Top-1")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title("CPU usage by time (top per-process %)")
    plt.xlabel("Time")
    plt.ylabel("CPU % (sum across processes)")
    path = os.path.join(out_dir, "top_cpu_usage.png")
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return path


def plot_load_avg(agg: pd.DataFrame, out_dir: str):
    plt.figure(figsize=(12, 4))
    plt.plot(agg["time"], agg["load1"], label="load1")
    plt.plot(agg["time"], agg["load5"], label="load5")
    plt.plot(agg["time"], agg["load15"], label="load15")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title("Load averages (from top TIMESTAMP)")
    plt.xlabel("Time")
    plt.ylabel("Load")
    path = os.path.join(out_dir, "top_load_avg.png")
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return path


def plot_psi(psi: pd.DataFrame, out_dir: str):
    # Show avg10 time series for cpu/io/memory "some" stall
    cols = [
        ("cpu_some_avg10", "CPU PSI avg10"),
        ("io_some_avg10", "IO PSI avg10"),
        ("mem_some_avg10", "Mem PSI avg10"),
    ]
    available = [c for c, _ in cols if c in psi.columns]
    if not available:
        return None
    plt.figure(figsize=(12, 4))
    for c, label in cols:
        if c in psi.columns:
            plt.plot(psi["time"], psi[c], label=label)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title("Pressure Stall Information (PSI) avg10")
    plt.xlabel("Time")
    plt.ylabel("Stall %")
    path = os.path.join(out_dir, "psi_avg10.png")
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return path


def main():
    ap = argparse.ArgumentParser(description="Plot CPU usage, load averages, and optional PSI from top CSV.")
    ap.add_argument("--csv", default=os.path.expanduser("~/Robit_SlamNav/NavOA_ws/top.csv"), help="Path to top CSV file")
    ap.add_argument("--psi", default=os.path.expanduser("~/Robit_SlamNav/NavOA_ws/tools/top_analysis/psi.csv"), help="Optional PSI CSV file path")
    ap.add_argument("--out", default=os.path.expanduser("~/Robit_SlamNav/NavOA_ws/output/top_plots"), help="Output directory for plots")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    cpu_path = None
    load_path = None
    if os.path.exists(args.csv):
        try:
            agg = parse_top_csv(args.csv)
            if not agg.empty:
                cpu_path = plot_cpu_series(agg, args.out)
                load_path = plot_load_avg(agg, args.out)
        except Exception:
            pass

    psi = maybe_read_psi_csv(args.psi)
    psi_path = None
    if psi is not None:
        psi_path = plot_psi(psi, args.out)

    print("Plots saved:")
    if cpu_path:
        print(f"- {cpu_path}")
    if load_path:
        print(f"- {load_path}")
    if psi_path:
        print(f"- {psi_path}")
    if not any([cpu_path, load_path, psi_path]):
        print("(No plots generated: missing inputs)")


if __name__ == "__main__":
    main()
