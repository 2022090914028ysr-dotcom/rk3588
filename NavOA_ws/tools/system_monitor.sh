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

# Processes to track in the main CSV. An entry ending in '*' is a comm prefix;
# Linux truncates comm to 15 chars, so cartographer_node -> cartographer_no.
WATCH_PROCESSES="python3 roslaunch rosout cartographer*"

# Clock ticks per second (used to convert /proc/<pid>/stat CPU deltas to %)
HZ=$(getconf CLK_TCK 2>/dev/null || echo 100)
# Page size in kB (used to convert /proc/<pid>/statm resident pages to RSS)
PAGE_KB=$(( $(getconf PAGESIZE 2>/dev/null || echo 4096) / 1024 ))

# Derived paths
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TOP_ANALYSIS_DIR="$SCRIPT_DIR/top_analysis"
PIDSTAT_LOG="$TOP_ANALYSIS_DIR/pidstat.log"
PSI_CSV="$TOP_ANALYSIS_DIR/psi.csv"
PLOTS_OUT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/tools/output/top_plots"
TOP_TMP=$(mktemp "${TMPDIR:-/tmp}/system_monitor_top.XXXXXX")

echo "time,cpu_temp_C,npu_temp_C,total_cpu_pct,npu_usage_pct,proc,proc_cpu_pct,proc_rss_MB" > "$OUT"
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

# NPU utilization % (rk3588: rknpu_ondemand exposes "load@freqHz" in devfreq).
# Returns "NA" if no NPU devfreq node is present.
get_npu_usage() {
  local f l v
  for f in /sys/class/devfreq/*npu*/load; do
    [ -r "$f" ] || continue
    l=$(cat "$f" 2>/dev/null) || continue
    [ -n "$l" ] || continue
    v=${l%%@*}; v=${v//[^0-9]/}
    [ -n "$v" ] && { echo "$v"; return; }
  done
  echo "NA"
}

# ---- /proc-based helpers (instantaneous CPU, robust to comm truncation) ----

declare -A PREV_TICKS       # pid -> utime+stime (clock ticks) at last sample
declare -A UID_CACHE        # uid -> username
PREV_NS=0
PREV_CPU_TOTAL=0
PREV_CPU_IDLE=0

# First line of /proc/stat -> "total idle" in clock ticks (idle includes iowait).
read_cpu_stat() {
  awk 'NR==1{print $2+$3+$4+$5+$6+$7+$8+$9, $5+$6}' /proc/stat
}

# Cumulative CPU ticks (utime+stime) for a pid -> global PROC_TICKS (-1 on failure).
read_proc_ticks() {
  local pid=$1 stat rest
  IFS= read -r stat < "/proc/$pid/stat" 2>/dev/null || { PROC_TICKS=-1; return 1; }
  rest=${stat##*) }           # strip everything through the final "comm)" field
  set -- $rest                # state ppid ... utime stime ...
  PROC_TICKS=$(( ${12:-0} + ${13:-0} ))
}

# Resident set size in kB -> global PROC_RSS_KB (0 if unreadable). Uses statm.
read_proc_rss_kb() {
  local pid=$1 size rss_pages shared
  read -r size rss_pages shared < "/proc/$pid/statm" 2>/dev/null || { PROC_RSS_KB=0; return 0; }
  PROC_RSS_KB=$(( rss_pages * PAGE_KB ))
}

# Match a comm name against WATCH_PROCESSES (exact, or '*' prefix).
is_watched_comm() {
  local c=$1 w
  for w in $WATCH_PROCESSES; do
    case "$w" in
      *'*') case "$c" in "${w%\*}"*) return 0 ;; esac ;;
      *)    [ "$c" = "$w" ] && return 0 ;;
    esac
  done
  return 1
}

# Resolve pid -> username (cached), falling back to numeric uid.
uid_to_name() {
  local pid=$1 uid name
  [ -r "/proc/$pid/status" ] || { echo "-"; return; }
  uid=$(awk '/^Uid:/{print $2}' "/proc/$pid/status" 2>/dev/null)
  [ -n "$uid" ] || { echo "-"; return; }
  name=${UID_CACHE[$uid]:-}
  if [ -z "$name" ]; then
    name=$(getent passwd "$uid" 2>/dev/null | cut -d: -f1)
    [ -z "$name" ] && name="$uid"
    UID_CACHE[$uid]=$name
  fi
  echo "$name"
}

# integer tenths -> "x.y" (e.g. 1234 -> "123.4")
fmt_tenths() {
  local t=$1
  printf '%d.%d' $(( t / 10 )) $(( t % 10 ))
}

# Sample all processes once and write this interval's rows.
report_interval() {
  local now_ns delta_ns d_total d_idle cpu_total cpu_idle
  local pid comm ticks rss_kb cpu_tenths rss_tenths delta_ticks found=0 d

  now_ns=$(date +%s%N)
  delta_ns=$(( now_ns - PREV_NS )); PREV_NS=$now_ns

  # Total CPU% from /proc/stat delta over this interval
  read -r cpu_total cpu_idle <<< "$(read_cpu_stat)"
  d_total=$(( cpu_total - PREV_CPU_TOTAL )); PREV_CPU_TOTAL=$cpu_total
  d_idle=$(( cpu_idle - PREV_CPU_IDLE ));     PREV_CPU_IDLE=$cpu_idle
  TOTAL_CPU=$(awk -v t="$d_total" -v i="$d_idle" 'BEGIN{if(t>0) printf "%.1f", 100*(t-i)/t; else print "0.0"}')

  : > "$TOP_TMP"

  for d in /proc/[0-9]*; do
    pid=${d##*/}
    [ -r "$d/comm" ] || continue
    IFS= read -r comm < "$d/comm" 2>/dev/null || continue

    read_proc_ticks "$pid" || continue
    ticks=$PROC_TICKS
    read_proc_rss_kb "$pid"
    rss_kb=$PROC_RSS_KB

    cpu_tenths=0
    if [ "$delta_ns" -gt 0 ] && [ -n "${PREV_TICKS[$pid]+x}" ]; then
      delta_ticks=$(( ticks - PREV_TICKS[$pid] ))
      if [ "$delta_ticks" -ge 0 ]; then
        # tenths of a % = delta_ticks * 1e12 / (HZ * delta_ns)
        cpu_tenths=$(( delta_ticks * 1000000000000 / (HZ * delta_ns) ))
      fi
    fi
    PREV_TICKS[$pid]=$ticks

    if is_watched_comm "$comm"; then
      rss_tenths=$(( (rss_kb * 10 + 512) / 1024 ))
      echo "$TIME,$CPU_TEMP,$NPU_TEMP,$TOTAL_CPU,$NPU_USAGE,$comm,$(fmt_tenths "$cpu_tenths"),$(fmt_tenths "$rss_tenths")" >> "$OUT"
      found=1
    fi

    printf '%s %s %s %s\n' "$cpu_tenths" "$rss_kb" "$comm" "$pid" >> "$TOP_TMP"
  done

  [ "$found" -eq 0 ] && echo "$TIME,$CPU_TEMP,$NPU_TEMP,$TOTAL_CPU,$NPU_USAGE,-,0,0" >> "$OUT"

  if [ "$ENABLE_TOP5" = true ]; then
    write_top5
  fi
}

write_top5() {
  local rank=0 cpu rss comm pid rss_tenths user
  while read -r cpu rss comm pid; do
    rank=$((rank+1))
    rss_tenths=$(( (rss * 10 + 512) / 1024 ))
    user=$(uid_to_name "$pid")
    echo "$TIME,$rank,$pid,$user,$(fmt_tenths "$cpu"),$(fmt_tenths "$rss_tenths"),$comm" >> "$OUT_TOP_CPU"
  done < <(sort -k1,1nr "$TOP_TMP" | head -5)

  rank=0
  while read -r cpu rss comm pid; do
    rank=$((rank+1))
    rss_tenths=$(( (rss * 10 + 512) / 1024 ))
    user=$(uid_to_name "$pid")
    echo "$TIME,$rank,$pid,$user,$(fmt_tenths "$cpu"),$(fmt_tenths "$rss_tenths"),$comm" >> "$OUT_TOP_MEM"
  done < <(sort -k2,2nr "$TOP_TMP" | head -5)
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
  rm -f "$TOP_TMP"
}
trap cleanup INT TERM EXIT

# Seed baseline samples (no output): /proc/stat and every pid's CPU ticks.
PREV_NS=$(date +%s%N)
read -r PREV_CPU_TOTAL PREV_CPU_IDLE <<< "$(read_cpu_stat)"
for d in /proc/[0-9]*; do
  pid=${d##*/}
  [ -r "$d/comm" ] || continue
  if read_proc_ticks "$pid"; then
    PREV_TICKS[$pid]=$PROC_TICKS
  fi
done

while [ $SECONDS -lt $END ]; do
  sleep "$INTERVAL"
  TIME=$(date +%H:%M:%S)
  CPU_TEMP=$(get_temp cpu)
  NPU_TEMP=$(get_temp npu)
  NPU_USAGE=$(get_npu_usage)
  report_interval
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
