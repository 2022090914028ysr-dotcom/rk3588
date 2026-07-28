#!/usr/bin/env python3
"""
Standalone plotter for /matching_score CSV data.

Usage:
    python3 plot_matching_score_trace.py matching_score_20260724_143022.csv
    python3 plot_matching_score_trace.py matching_score_20260724_143022.csv --output my_plot.png

Expected CSV format: timestamp,score[,mode]
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Thresholds (from status_state.py defaults)
THRESH_HIGH = 0.7   # LOCALIZED HIGH
THRESH_LOST = 0.3   # LOST boundary


def parse_args():
    parser = argparse.ArgumentParser(
        description='Plot matching_score trace from CSV recording')
    parser.add_argument('csv', help='Path to CSV file (timestamp,score[,mode])')
    parser.add_argument('--output', '-o', default=None,
                        help='Output PNG path (default: matching_score_trace_<input>.png)')
    return parser.parse_args()


def load_csv(path):
    """Load CSV and return (timestamps, scores, modes)."""
    t, scores, modes = [], [], []
    with open(path) as f:
        # Skip header if it exists
        first = f.readline().strip()
        if first.startswith('timestamp') or first.startswith('#'):
            pass  # skip header
        else:
            # First line is data, process it
            parts = first.split(',')
            try:
                t.append(float(parts[0]))
                scores.append(float(parts[1]))
                modes.append(parts[2] if len(parts) > 2 else '')
            except (ValueError, IndexError):
                pass

        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            try:
                t.append(float(parts[0]))
                scores.append(float(parts[1]))
                modes.append(parts[2] if len(parts) > 2 else '')
            except (ValueError, IndexError):
                continue

    t = np.array(t)
    scores = np.array(scores)

    # Normalize timestamps to seconds from start
    if len(t) > 0:
        t_zero = t[0]
        t = t - t_zero

    return t, scores, modes


def build_figure(t, scores, modes, output_path):
    """Plot matching_score trace with threshold zones."""
    if len(t) == 0:
        print('[ERROR] No data points found in CSV')
        sys.exit(1)

    duration = t[-1] if len(t) > 1 else 1.0

    fig, ax = plt.subplots(1, 1, figsize=(16, 7), dpi=150)
    fig.patch.set_facecolor('white')

    # Zone fills
    ax.axhspan(THRESH_HIGH, 1.0,    xmin=0, xmax=1, facecolor='#c8e6c9', alpha=0.25, zorder=0)
    ax.axhspan(THRESH_LOST, THRESH_HIGH, xmin=0, xmax=1, facecolor='#fff9c4', alpha=0.25, zorder=0)
    ax.axhspan(0.0, THRESH_LOST,     xmin=0, xmax=1, facecolor='#ffcdd2', alpha=0.25, zorder=0)

    # Threshold lines
    ax.axhline(y=THRESH_HIGH, color='#2e7d32', linestyle='--', linewidth=1.2,
               label=f'LOCALIZED HIGH (≥{THRESH_HIGH})')
    ax.axhline(y=THRESH_LOST, color='#e65100', linestyle='--', linewidth=1.2,
               label=f'LOST (<{THRESH_LOST})')

    # Score trace
    ax.plot(t, scores, color='#1565c0', linewidth=0.8, alpha=0.8,
            label=f'/matching_score ({len(t)} samples)')

    # Overlay mode changes if available
    if any(modes):
        # Find mode transition points
        prev_mode = modes[0]
        for i, mode in enumerate(modes):
            if mode != prev_mode:
                ax.axvline(x=t[i], color='#333333', linestyle=':', linewidth=0.6, alpha=0.5)
                ax.text(t[i], 0.02, mode, fontsize=6, rotation=90,
                        va='bottom', ha='right', alpha=0.6,
                        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', alpha=0.7))
                prev_mode = mode

    # Zone labels
    ax.text(0.95 * duration, 0.85, 'LOCALIZED HIGH', fontsize=8,
            color='#2e7d32', ha='right', va='center', alpha=0.6, fontweight='bold')
    ax.text(0.95 * duration, 0.50, 'LOCALIZED LOW', fontsize=8,
            color='#e65100', ha='right', va='center', alpha=0.6, fontweight='bold')
    ax.text(0.95 * duration, 0.15, 'LOST', fontsize=8,
            color='#c62828', ha='right', va='center', alpha=0.6, fontweight='bold')

    # Stats text
    mean_score = np.mean(scores)
    min_score = np.min(scores)
    lost_pct = 100 * np.sum(scores < THRESH_LOST) / len(scores)
    high_pct = 100 * np.sum(scores >= THRESH_HIGH) / len(scores)

    stats_text = (f'Mean: {mean_score:.3f}  |  Min: {min_score:.3f}\n'
                  f'HIGH: {high_pct:.1f}%  |  LOST: {lost_pct:.1f}%')
    ax.annotate(stats_text, xy=(0.02, 0.97), xycoords='axes fraction',
                fontsize=8, va='top', ha='left',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='#cccccc', alpha=0.9))

    # Labels
    duration_sec = t[-1]
    if duration_sec > 120:
        # Show minutes
        ax.set_xlabel(f'Time (seconds) — total: {duration_sec:.0f}s ({duration_sec/60:.1f} min)', fontsize=11)
    else:
        ax.set_xlabel(f'Time (seconds)', fontsize=11)
    ax.set_ylabel('Matching Score', fontsize=11)
    ax.set_title('/matching_score — Localization Stability Trace', fontsize=14,
                 fontweight='bold', pad=12)

    ax.set_xlim(0, duration)
    ax.set_ylim(0.0, 1.0)
    ax.legend(loc='upper right', fontsize=9, framealpha=0.9, edgecolor='#cccccc')
    ax.grid(True, alpha=0.3, linestyle=':')

    plt.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'[OK] Trace plot saved: {output_path}')
    print(f'     {len(t)} samples over {duration:.1f}s')
    print(f'     Score range: {min_score:.3f} – {scores.max():.3f}')


def main():
    args = parse_args()

    if not os.path.isfile(args.csv):
        print(f'[ERROR] CSV file not found: {args.csv}')
        sys.exit(1)

    # Default output name
    if args.output is None:
        basename = os.path.splitext(os.path.basename(args.csv))[0]
        args.output = os.path.join(os.path.dirname(args.csv),
                                   f'matching_score_trace_{basename}.png')

    t, scores, modes = load_csv(args.csv)
    build_figure(t, scores, modes, args.output)


if __name__ == '__main__':
    main()
