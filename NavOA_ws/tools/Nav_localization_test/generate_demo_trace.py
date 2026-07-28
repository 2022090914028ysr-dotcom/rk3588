#!/usr/bin/env python3
"""
Generate a demo /matching_score localization stability trace for the report.

Creates a realistic synthetic time series based on actual system thresholds
and observed score values from the Cartographer guard logs. Saves both the
plot PNG and the demo CSV so the standalone plotter can reproduce it.

Thresholds (from status_state.py defaults):
    >= 0.7  → LOCALIZED HIGH
    >= 0.3  → LOCALIZED LOW
    <  0.3  → LOST

Output: matching_score_trace_demo.png
        matching_score_trace_demo.csv
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ─── Configuration ───────────────────────────────────────────────────────────

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
PNG_PATH = os.path.join(OUTPUT_DIR, 'matching_score_trace_demo.png')
CSV_PATH = os.path.join(OUTPUT_DIR, 'matching_score_trace_demo.csv')

# Synthetic trace parameters
N_SAMPLES = 400
DURATION_SEC = 150.0           # 2.5 minute demo
TR = DURATION_SEC / N_SAMPLES  # ~0.375s per sample

# Thresholds
THRESH_HIGH = 0.7   # LOCALIZED HIGH
THRESH_LOST = 0.3   # LOST boundary

rng = np.random.RandomState(42)  # deterministic seed

# ─── Generate Realistic Trace ────────────────────────────────────────────────

def generate_trace():
    t = np.linspace(0, DURATION_SEC, N_SAMPLES)

    # Base: typical score during steady localization
    # From actual logs: 0.60–0.68 most of the time
    base = 0.64 * np.ones(N_SAMPLES)

    # Slow drift
    drift = 0.04 * np.sin(2 * np.pi * t / 80.0)

    # Noise
    noise = rng.normal(0, 0.015, N_SAMPLES)

    # ── Events ─────────────────────────────────────────────────────────
    # 1. Small dip at t≈30s (robot turns a corner)
    dip1_idx = (t >= 28) & (t <= 38)
    base[dip1_idx] -= 0.12 * np.sin(np.pi * (t[dip1_idx] - 28) / 10)

    # 2. Brief drop at t≈55s (doorway transition)
    dip2_idx = (t >= 52) & (t <= 62)
    base[dip2_idx] -= 0.08 * np.sin(np.pi * (t[dip2_idx] - 52) / 10)

    # 3. Recovery event at t≈80–110s: score drops below 0.3, then recovers
    event_start, event_recover, event_end = 80, 95, 110
    event_idx = (t >= event_start) & (t <= event_end)

    # Sharp drop
    drop_phase = (t[event_idx] - event_start) / (event_recover - event_start)
    base[event_idx] -= 0.40 * np.minimum(drop_phase, 1.0)

    # Recovery (slower)
    rec_phase = (t[event_idx] - event_recover) / (event_end - event_recover)
    rec_mask = t[event_idx] >= event_recover
    base[event_idx][rec_mask] += 0.40 * np.minimum(rec_phase[rec_mask], 1.0)

    # 4. Oscillation at t≈125s (featureless corridor)
    osc_idx = (t >= 120) & (t <= 135)
    base[osc_idx] += 0.06 * np.sin(2 * np.pi * (t[osc_idx] - 120) / 3)

    # Combine
    score = np.clip(base + drift + noise, 0.0, 1.0)

    # Clamp the event to actually go below threshold
    event_bottom = (t >= 87) & (t <= 92)
    score[event_bottom] = np.clip(
        score[event_bottom] - 0.15,
        0.0, 1.0
    )

    return t, score

# ─── Build Figure ────────────────────────────────────────────────────────────

def build_figure():
    t, score = generate_trace()

    # Save CSV for standalone plotter
    header = 'timestamp,score'
    np.savetxt(CSV_PATH, np.column_stack((t, score)),
               delimiter=',', header=header, comments='',
               fmt='%.6f')
    print(f'[OK] Demo CSV saved: {CSV_PATH}')

    # ── Plot ────────────────────────────────────────────────────────────
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
    ax.plot(t, score, color='#1565c0', linewidth=1.0, alpha=0.85,
            label='/matching_score')

    # Annotation: recovery event
    ax.annotate('Localization lost →\nauto-recovery triggered',
                xy=(88, 0.22), xytext=(102, 0.15),
                fontsize=9, color='#c62828', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='#c62828', lw=1.2),
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffebee',
                          edgecolor='#c62828', alpha=0.9))

    ax.annotate('Guard recovery:\ntrajectory re-initialized\nwith captured pose',
                xy=(97, 0.58), xytext=(112, 0.72),
                fontsize=9, color='#2e7d32', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='#2e7d32', lw=1.2),
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#e8f5e9',
                          edgecolor='#2e7d32', alpha=0.9))

    # Zone labels
    ax.text(0.95 * DURATION_SEC, 0.85, 'LOCALIZED HIGH', fontsize=8,
            color='#2e7d32', ha='right', va='center', alpha=0.6, fontweight='bold')
    ax.text(0.95 * DURATION_SEC, 0.50, 'LOCALIZED LOW', fontsize=8,
            color='#e65100', ha='right', va='center', alpha=0.6, fontweight='bold')
    ax.text(0.95 * DURATION_SEC, 0.15, 'LOST', fontsize=8,
            color='#c62828', ha='right', va='center', alpha=0.6, fontweight='bold')

    # Axis labels
    ax.set_xlabel('Time (seconds)', fontsize=11)
    ax.set_ylabel('Matching Score', fontsize=11)
    ax.set_title('/matching_score — Localization Stability Trace (Demo)',
                 fontsize=14, fontweight='bold', pad=12)

    ax.set_xlim(0, DURATION_SEC)
    ax.set_ylim(0.0, 1.0)
    ax.legend(loc='upper right', fontsize=9, framealpha=0.9, edgecolor='#cccccc')
    ax.grid(True, alpha=0.3, linestyle=':')

    plt.tight_layout()
    fig.savefig(PNG_PATH, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'[OK] Demo trace image saved: {PNG_PATH}')
    print(f'     {N_SAMPLES} samples over {DURATION_SEC}s')
    print(f'     Score range: {score.min():.3f} – {score.max():.3f}')

if __name__ == '__main__':
    build_figure()
