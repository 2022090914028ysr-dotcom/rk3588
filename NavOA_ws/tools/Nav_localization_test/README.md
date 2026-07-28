# Nav Localization Test Tools

This directory contains tools to generate test images and record/plot
localization stability data for the SlamNav navigation report.

## Files

| File | Purpose |
|------|---------|
| `generate_map_quality.py` | Generate annotated occupancy grid map from test001.pgm |
| `generate_demo_trace.py` | Generate demo matching_score trace (simulated, for immediate use) |
| `matching_score_recorder.py` | ROS node to record live /matching_score to CSV |
| `plot_matching_score_trace.py` | Standalone plotter for recorded matching_score CSV data |
| `map_quality_test001.png` | Output: annotated occupancy grid of meeting_room/test001 |
| `matching_score_trace_demo.png` | Output: demo localization stability trace |
| `matching_score_trace_demo.csv` | Output: demo trace data (reproducible by plotter) |

## Usage

### Generate Map Quality Figure (offline, no ROS needed)

```bash
cd /path/to/NavOA_ws/tools/Nav_localization_test/
python3 generate_map_quality.py
# → creates map_quality_test001.png
```

### Generate Demo Trace (offline, no ROS needed)

```bash
python3 generate_demo_trace.py
# → creates matching_score_trace_demo.png
# → creates matching_score_trace_demo.csv
```

### Record Live Matching Score (requires live robot with ROS)

On the robot:

```bash
rosrun Nav_localization_test matching_score_recorder.py
```

Or directly:

```bash
python3 /path/to/matching_score_recorder.py
```

Records to `matching_score_<datetime>.csv` with columns: `timestamp,score,mode`

Press Ctrl+C to stop recording.

### Plot Recorded Data (offline)

```bash
python3 plot_matching_score_trace.py matching_score_20260724_143022.csv
# → creates matching_score_trace_matching_score_20260724_143022.png
```

Custom output path:

```bash
python3 plot_matching_score_trace.py data.csv --output my_plot.png
```

### Verify Plotter With Demo Data

```bash
python3 plot_matching_score_trace.py matching_score_trace_demo.csv
# → should reproduce the demo trace figure
```

## Threshold Reference

From `status_state.py` (defaults):

| Threshold | Value | Meaning |
|-----------|-------|---------|
| `score_ok` | 0.7 | LOCALIZED HIGH |
| `score_warn` | 0.3 | LOCALIZED LOW / LOST boundary |
| `score_timeout` | 5.0 s | No score → LOST |

Typical matching_score values observed from Cartographer in the
meeting room environment: **0.60 – 0.68** (constraint builder scores
from `hard_reset_seeded.log`).

## Notes

- The demo trace is **simulated data** based on real thresholds and
  typical observed values. Replace it with real recordings for reports
  requiring production data.
- The map quality figure annotates the meeting_room/test001 occupancy
  grid with dimensions and scale. No reference floor plan photo exists
  on disk — if one becomes available, it can be composited side-by-side.
