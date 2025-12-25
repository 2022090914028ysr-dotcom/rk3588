# System Monitoring & Plots

This folder contains scripts to collect and visualize system metrics (CPU/memory, load averages, temperatures, PSI) during robot runs.

**One-Command Run (recommended)**

```bash
cd ~/Robit_SlamNav/NavOA_ws/tools
bash system_monitor.sh
```

- Collects `pidstat` CPU/memory into [NavOA_ws/tools/top_analysis/pidstat.log](../tools/top_analysis/pidstat.log)
- Samples temps + total CPU into [NavOA_ws/tools/system_monitor.csv](../tools/system_monitor.csv)
- Generates plots into [NavOA_ws/output/top_plots](../output/top_plots):
  - `pidstat_cpu.png`, `pidstat_mem.png` (CPU/memory vs time)
  - `temps.png` (CPU/NPU temps; overlays total CPU%)
  - Optionally PSI if enabled
  - `psi_avg10.png` if PSI collection is enabled
  - If `ENABLE_TOP5=true`, also `top5_cpu.png`, `top5_mem.png` and a per-user comparison `top5_users_cpu.png`

**Flags in system_monitor.sh**
- `INTERVAL` (seconds) and `DURATION` control sampling cadence and length.
- `ENABLE_PIDSTAT=true` collects `pidstat -u -r` concurrently.
- `ENABLE_PLOTS=true` writes PNGs at the end.
- `ENABLE_PSI=false` to collect `/proc/pressure/*` via `collect_psi.py`.
- `ENABLE_TOP5=false` to skip CSVs for top-5 CPU/memory per tick.

**Standalone Plotters**
- CPU/memory from pidstat:

```bash
python3 ~/Robit_SlamNav/NavOA_ws/tools/top_analysis/plot_pidstat.py \
  --log ~/Robit_SlamNav/NavOA_ws/tools/top_analysis/pidstat.log \
  --out ~/Robit_SlamNav/NavOA_ws/output/top_plots
```

- Temperatures and total CPU from system_monitor.csv:

```bash
python3 ~/Robit_SlamNav/NavOA_ws/tools/top_analysis/plot_temps.py \
  --csv ~/Robit_SlamNav/NavOA_ws/tools/system_monitor.csv \
  --out ~/Robit_SlamNav/NavOA_ws/output/top_plots
```

- Load averages and optional PSI from `top.csv`:

```bash
python3 ~/Robit_SlamNav/NavOA_ws/tools/top_analysis/plot_top.py \
  --csv ~/Robit_SlamNav/NavOA_ws/top.csv \  # optional; skip if missing
  --psi ~/Robit_SlamNav/NavOA_ws/tools/top_analysis/psi.csv \  # optional
  --out ~/Robit_SlamNav/NavOA_ws/output/top_plots
```

**Troubleshooting**
- If PSI was enabled but `psi.csv` didn’t appear, ensure `collect_psi.py` exists under `tools/top_analysis`. The script now runs it via `python3`, no executable bit required.
- If you enable `ENABLE_TOP5=true`, the script writes `system_monitor_top5_cpu.csv` and `system_monitor_top5_mem.csv` and generates `top5_cpu.png`, `top5_mem.png`, and `top5_users_cpu.png` at the end.

**Notes**
- CPU totals can exceed 100% on multi-core systems (sum of per-process %CPU).
- CPU temperature is derived from the hottest of rk3588 zones (bigcore/littlecore/soc/center).
- PSI requires a kernel exposing `/proc/pressure/*`; if unavailable, skip or expect zeros.
