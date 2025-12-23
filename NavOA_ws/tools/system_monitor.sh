#!/bin/bash

# Ensure running under Bash (arrays/functions used below)
if [ -z "$BASH_VERSION" ]; then
  echo "[system_monitor] Please run with bash: bash system_monitor.sh" >&2
  exit 2
fi

OUT=system_monitor.csv
OUT_TOP_CPU=system_monitor_top5_cpu.csv
OUT_TOP_MEM=system_monitor_top5_mem.csv
INTERVAL=1
DURATION=60   # seconds (change if needed)

# All-in-one extras
ENABLE_PIDSTAT=true        # also collect pidstat -u -r into a log
ENABLE_PSI=true           # also collect PSI (/proc/pressure/*) into a CSV
ENABLE_PLOTS=true          # generate plots at the end (from pidstat and optional PSI)
ENABLE_TOP5=true          # record top-5 CPU/memory CSVs each interval (set true to enable)

# Derived paths
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TOP_ANALYSIS_DIR="$SCRIPT_DIR/top_analysis"
PIDSTAT_LOG="$TOP_ANALYSIS_DIR/pidstat.log"
PSI_CSV="$TOP_ANALYSIS_DIR/psi.csv"
PLOTS_OUT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/tools/output/top_plots"

echo "time,cpu_temp_C,npu_temp_C,total_cpu_pct,proc,proc_cpu_pct,proc_rss_MB" > "$OUT"
if [ "$ENABLE_TOP5" = true ]; then
  echo "time,rank,pid,user,cpu_pct,rss_MB,cmd" > "$OUT_TOP_CPU"
  echo "time,rank,pid,user,cpu_pct,rss_MB,cmd" > "$OUT_TOP_MEM"
fi

get_temp() {
  local name=$1
  # Special handling for CPU on rk3588: take the hottest among big/little clusters or SoC/center if available
  if [ "$name" = "cpu" ]; then
    local max=""
    for z in /sys/class/thermal/thermal_zone*; do
      if grep -qiE 'bigcore|littlecore|cpu-thermal|soc-thermal|center-thermal' "$z/type" 2>/dev/null; then
        if [ -r "$z/temp" ]; then
          local t
          t=$(awk '{printf "%.1f", $1/1000}' "$z/temp")
          if [ -z "$max" ]; then
            max="$t"
          else
            # set max = max(max, t)
            if awk -v a="$max" -v b="$t" 'BEGIN { exit !(b>a) }'; then
              max="$t"
            fi
          fi
        fi
      fi
    done
    if [ -n "$max" ]; then echo "$max"; else echo "NA"; fi
    return
  fi
  # Generic: match by substring in thermal zone type
  for z in /sys/class/thermal/thermal_zone*; do
    if grep -qi "$name" "$z/type" 2>/dev/null; then
      awk '{printf "%.1f", $1/1000}' "$z/temp"
      return
    fi
  done
  echo "NA"
}

END=$((SECONDS + DURATION))

# Start optional background collectors
mkdir -p "$TOP_ANALYSIS_DIR" "$PLOTS_OUT_DIR" >/dev/null 2>&1

PIDS_TO_KILL=()

if [ "$ENABLE_PIDSTAT" = true ]; then
  # Run pidstat for the duration; 1Hz sampling
  ( timeout "$DURATION" pidstat -u -r -p ALL "$INTERVAL" > "$PIDSTAT_LOG" ) &
  PIDS_TO_KILL+=("$!")
fi

if [ "$ENABLE_PSI" = true ] && [ -f "$TOP_ANALYSIS_DIR/collect_psi.py" ]; then
  ( python3 "$TOP_ANALYSIS_DIR/collect_psi.py" --out "$PSI_CSV" --interval "$INTERVAL" --duration "$DURATION" ) &
  PIDS_TO_KILL+=("$!")
fi

cleanup() {
  for p in "${PIDS_TO_KILL[@]}"; do
    kill "$p" >/dev/null 2>&1 || true
  done
}
trap cleanup INT TERM

while [ $SECONDS -lt $END ]; do
  TIME=$(date +%H:%M:%S)

  CPU_TEMP=$(get_temp cpu)
  NPU_TEMP=$(get_temp npu)

  # Robustly compute total CPU usage = 100 - idle% from top's Cpu(s) line
  TOTAL_CPU=$(LANG=C top -bn1 | awk -F',' '/Cpu\(s\)/{for(i=1;i<=NF;i++){if($i ~ / id/){gsub(/^.* /,"",$i); gsub(/ id.*/,"",$i); idle=$i}}} END{if(idle=="") idle=0; printf "%.1f", 100-idle}')

  found=0
  for P in python3 rviz cartographer cartographer_node roslaunch rosout; do
    ps -C "$P" -o %cpu,rss --no-headers 2>/dev/null | while read CPU RSS; do
      RSS_MB=$(awk "BEGIN {printf \"%.1f\", $RSS/1024}")
      echo "$TIME,$CPU_TEMP,$NPU_TEMP,$TOTAL_CPU,$P,$CPU,$RSS_MB" >> "$OUT"
      found=1
    done
  done

  # If none of the watched processes were present, still record a baseline row
  if [ "$found" -eq 0 ]; then
    echo "$TIME,$CPU_TEMP,$NPU_TEMP,$TOTAL_CPU,-,0,0" >> "$OUT"
  fi

  if [ "$ENABLE_TOP5" = true ]; then
    # Top-5 by CPU this interval
    rank=0
    ps -eo pid,user,pcpu,rss,comm --sort=-pcpu | awk 'NR>1 {print; count++; if(count==5) exit}' | while read PID USER PCPU RSS COMM; do
      rank=$((rank+1))
      RSS_MB=$(awk "BEGIN {printf \"%.1f\", $RSS/1024}")
      echo "$TIME,$rank,$PID,$USER,$PCPU,$RSS_MB,$COMM" >> "$OUT_TOP_CPU"
    done

    # Top-5 by Memory (RSS) this interval
    rank=0
    ps -eo pid,user,pcpu,rss,comm --sort=-rss | awk 'NR>1 {print; count++; if(count==5) exit}' | while read PID USER PCPU RSS COMM; do
      rank=$((rank+1))
      RSS_MB=$(awk "BEGIN {printf \"%.1f\", $RSS/1024}")
      echo "$TIME,$rank,$PID,$USER,$PCPU,$RSS_MB,$COMM" >> "$OUT_TOP_MEM"
    done
  fi

  sleep $INTERVAL
done

# Finish: wait briefly for background tools
for p in "${PIDS_TO_KILL[@]}"; do
  wait "$p" 2>/dev/null || true
done

# Generate plots if enabled and pidstat log exists
if [ "$ENABLE_PLOTS" = true ] && [ -s "$PIDSTAT_LOG" ] && [ -f "$TOP_ANALYSIS_DIR/plot_pidstat.py" ]; then
  python3 "$TOP_ANALYSIS_DIR/plot_pidstat.py" --log "$PIDSTAT_LOG" --out "$PLOTS_OUT_DIR" || true
fi

if [ "$ENABLE_PLOTS" = true ] && [ -s "$PSI_CSV" ] && [ -f "$TOP_ANALYSIS_DIR/plot_top.py" ]; then
  # plot_top.py can also overlay PSI if provided; a CSV for top is optional so we pass only --psi
  python3 "$TOP_ANALYSIS_DIR/plot_top.py" --psi "$PSI_CSV" --out "$PLOTS_OUT_DIR" || true
fi

# Always try to generate temperature plot from system_monitor.csv
if [ "$ENABLE_PLOTS" = true ] && [ -s "$OUT" ] && [ -f "$TOP_ANALYSIS_DIR/plot_temps.py" ]; then
  python3 "$TOP_ANALYSIS_DIR/plot_temps.py" --csv "$OUT" --out "$PLOTS_OUT_DIR" || true
fi

echo "[system_monitor] Done. Data: $OUT, $OUT_TOP_CPU, $OUT_TOP_MEM" >&2
if [ -s "$PIDSTAT_LOG" ]; then
  echo "[system_monitor] pidstat: $PIDSTAT_LOG" >&2
  echo "[system_monitor] plots (if enabled): $PLOTS_OUT_DIR" >&2
fi

# Generate plots from top-5 CSVs if enabled
if [ "$ENABLE_TOP5" = true ] && [ -f "$TOP_ANALYSIS_DIR/plot_top5.py" ]; then
  if [ -s "$OUT_TOP_CPU" ] || [ -s "$OUT_TOP_MEM" ]; then
    python3 "$TOP_ANALYSIS_DIR/plot_top5.py" --cpu "$OUT_TOP_CPU" --mem "$OUT_TOP_MEM" --out "$PLOTS_OUT_DIR" || true
  fi
fi

# Per-user CPU comparison plot (top-N users across run) if data exists
if [ "$ENABLE_TOP5" = true ] && [ -f "$TOP_ANALYSIS_DIR/plot_top5_users.py" ] && [ -s "$OUT_TOP_CPU" ]; then
  # Plot per-user/label CPU: include requested labels for clear comparison
  python3 "$TOP_ANALYSIS_DIR/plot_top5_users.py" --csv "$OUT_TOP_CPU" --out "$PLOTS_OUT_DIR" \
    --names "cartographer,python3,robit,ubuntu,root" --by cmd || true
  # Auto-select and plot top-10 labels by command, plus ranking chart
  python3 "$TOP_ANALYSIS_DIR/plot_top5_users.py" --csv "$OUT_TOP_CPU" --out "$PLOTS_OUT_DIR" \
    --topN 10 --by cmd || true
fi
