#!/usr/bin/env python3
import time
import argparse
from datetime import datetime
import os
import pandas as pd

# Reads /proc/pressure/{cpu, io, memory} every interval and stores avg10/avg60/avg300 for "some" line.


def parse_psi_block(text: str):
    # Example line: "some avg10=0.00 avg60=0.00 avg300=0.00 total=0"
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        kind = parts[0]
        kv = {k.split("=")[0]: float(k.split("=")[1]) for k in parts[1:] if "=" in k}
        if kind == "some":
            out.update({
                "some_avg10": kv.get("avg10", 0.0),
                "some_avg60": kv.get("avg60", 0.0),
                "some_avg300": kv.get("avg300", 0.0),
            })
    return out


def read_psi_once():
    def slurp(p):
        try:
            with open(p) as f:
                return f.read()
        except Exception:
            return ""
    cpu = parse_psi_block(slurp("/proc/pressure/cpu"))
    io = parse_psi_block(slurp("/proc/pressure/io"))
    mem = parse_psi_block(slurp("/proc/pressure/memory"))
    return {
        "cpu_some_avg10": cpu.get("some_avg10", 0.0),
        "cpu_some_avg60": cpu.get("some_avg60", 0.0),
        "cpu_some_avg300": cpu.get("some_avg300", 0.0),
        "io_some_avg10": io.get("some_avg10", 0.0),
        "io_some_avg60": io.get("some_avg60", 0.0),
        "io_some_avg300": io.get("some_avg300", 0.0),
        "mem_some_avg10": mem.get("some_avg10", 0.0),
        "mem_some_avg60": mem.get("some_avg60", 0.0),
        "mem_some_avg300": mem.get("some_avg300", 0.0),
    }


def main():
    ap = argparse.ArgumentParser(description="Collect PSI (pressure stall) metrics into a CSV")
    ap.add_argument("--out", default=os.path.expanduser("~/Robit_SlamNav/NavOA_ws/tools/top_analysis/psi.csv"), help="Output CSV path")
    ap.add_argument("--interval", type=float, default=1.0, help="Sampling interval in seconds")
    ap.add_argument("--duration", type=float, default=120.0, help="Total duration in seconds")
    args = ap.parse_args()

    rows = []
    t_end = time.time() + args.duration
    while time.time() < t_end:
        row = read_psi_once()
        row["time"] = datetime.now()
        rows.append(row)
        time.sleep(args.interval)

    df = pd.DataFrame(rows)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    df.to_csv(args.out, index=False)
    print(f"Wrote PSI series: {args.out} ({len(df)} samples)")


if __name__ == "__main__":
    main()
