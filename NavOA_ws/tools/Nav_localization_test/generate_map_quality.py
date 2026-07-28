#!/usr/bin/env python3
"""
Generate an annotated occupancy grid map for the navigation report.

Reads test001.pgm + test001.yaml and renders a publication-quality figure
with dimensional annotations, scale bar, and coordinate grid.

Output: map_quality_test001.png
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap, BoundaryNorm
from PIL import Image

# ─── Configuration ───────────────────────────────────────────────────────────

WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
MAP_DIR = os.path.join(WORKSPACE, 'src', 'carto_ws', 'map', 'meeting_room')
PGM_PATH = os.path.join(MAP_DIR, 'test001.pgm')
YAML_PATH = os.path.join(MAP_DIR, 'test001.yaml')
OUTPUT_PATH = os.path.join(os.path.dirname(__file__), 'map_quality_test001.png')

# ─── Read Map Metadata ───────────────────────────────────────────────────────

def read_yaml(path):
    """Read a simple ROS map YAML (no pyyaml dependency needed)."""
    meta = {}
    with open(path) as f:
        for line in f:
            if ':' in line:
                key, val = line.split(':', 1)
                key = key.strip()
                val = val.strip().rstrip(',')
                meta[key] = val
    # Parse numeric values
    meta['resolution'] = float(meta.get('resolution', 0.05))
    origin_str = meta.get('origin', '[-5.544689, -12.431804, 0.0]').strip('[] ')
    meta['origin'] = [float(x) for x in origin_str.split(',')]
    meta['negate'] = int(meta.get('negate', 0))
    meta['occupied_thresh'] = float(meta.get('occupied_thresh', 0.65))
    meta['free_thresh'] = float(meta.get('free_thresh', 0.196))
    return meta

def load_pgm(path):
    """Load PGM and return (data_array, shape)."""
    img = Image.open(path)
    # PGM values: 0=black(occupied), 205=free, 255=unknown in Cartographer's output
    arr = np.array(img, dtype=np.uint8)
    return arr

# ─── Build Figure ────────────────────────────────────────────────────────────

def build_figure():
    meta = read_yaml(YAML_PATH)
    grid = load_pgm(PGM_PATH)

    res = meta['resolution']            # m/pixel
    ox, oy, _ = meta['origin']          # map origin in world coords
    height_m = grid.shape[0] * res       # physical height (meters)
    width_m  = grid.shape[1] * res       # physical width (meters)

    # World coordinate extents (map frame)
    x_min = ox
    x_max = ox + grid.shape[1] * res
    y_min = oy
    y_max = oy + grid.shape[0] * res

    # Convert PGM to our colormap categories:
    #   PGM 0     = occupied (black)
    #   PGM 205   = free (white)
    #   PGM 255   = unknown (grey)
    #   Everything else → interpolate
    categories = np.full(grid.shape, 2, dtype=np.uint8)  # default: unknown
    free_mask = grid >= 200   # free space
    occ_mask  = grid <= 30    # occupied
    categories[free_mask] = 1
    categories[occ_mask]  = 0

    # Custom colormap: occupied=#d62728(red), free=white, unknown=#cccccc(light grey)
    cmap = ListedColormap(['#d62728', '#f0f0f0', '#c0c0c0'])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)

    # ── Plot ─────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(1, 1, figsize=(14, 10), dpi=150)
    fig.patch.set_facecolor('white')

    im = ax.imshow(categories, cmap=cmap, norm=norm,
                   extent=[x_min, x_max, y_min, y_max],
                   origin='lower', interpolation='none', aspect='equal')

    # ── Grid lines every 5 meters ────────────────────────────────────────
    grid_step = 5.0  # meters
    x_ticks = np.arange(np.floor(x_min / grid_step) * grid_step,
                        np.ceil(x_max / grid_step) * grid_step + 0.01, grid_step)
    y_ticks = np.arange(np.floor(y_min / grid_step) * grid_step,
                        np.ceil(y_max / grid_step) * grid_step + 0.01, grid_step)

    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)
    ax.grid(True, which='major', color='#333333', linestyle='--', linewidth=0.4, alpha=0.5)
    ax.set_xticklabels([f'{t:.0f}' for t in x_ticks], fontsize=7)
    ax.set_yticklabels([f'{t:.0f}' for t in y_ticks], fontsize=7)
    ax.set_xlabel('X (meters)', fontsize=10)
    ax.set_ylabel('Y (meters)', fontsize=10)

    # ── Scale bar (5m) ──────────────────────────────────────────────────
    scale_bar_len = 5.0  # meters
    bar_x = x_max - scale_bar_len - 1.0
    bar_y = y_min + 0.5
    ax.plot([bar_x, bar_x + scale_bar_len], [bar_y, bar_y],
            color='black', linewidth=3, solid_capstyle='butt')
    ax.text(bar_x + scale_bar_len / 2, bar_y + 0.3,
            f'{scale_bar_len} m', ha='center', va='bottom',
            fontsize=9, fontweight='bold')

    # ── Dimension annotation ─────────────────────────────────────────────
    dim_text = (f'Map Size: {width_m:.1f} m × {height_m:.1f} m\n'
                f'Resolution: {res*100:.0f} cm/px  |  '
                f'Grid: {grid.shape[1]}×{grid.shape[0]} px')
    ax.annotate(dim_text, xy=(0.02, 0.97), xycoords='axes fraction',
                fontsize=8, va='top', ha='left',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='#cccccc', alpha=0.9))

    # ── Origin marker ────────────────────────────────────────────────────
    ax.plot(ox, oy, marker='*', color='#1f77b4', markersize=12, zorder=5)
    ax.annotate('Map Origin\n[−5.54, −12.43]', (ox, oy),
                xytext=(ox + 1.5, oy + 1.5),
                fontsize=7, color='#1f77b4', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='#1f77b4', lw=0.8),
                bbox=dict(boxstyle='round,pad=0.2', facecolor='#eef6fc',
                          edgecolor='#1f77b4', alpha=0.9))

    # ── Legend ───────────────────────────────────────────────────────────
    legend_patches = [
        mpatches.Patch(color='#d62728', label='Occupied'),
        mpatches.Patch(color='#f0f0f0', ec='#cccccc', label='Free'),
        mpatches.Patch(color='#c0c0c0', label='Unknown'),
    ]
    ax.legend(handles=legend_patches, loc='lower right',
              fontsize=8, framealpha=0.9, edgecolor='#cccccc')

    ax.set_title('Occupancy Grid Map — meeting_room/test001', fontsize=14,
                 fontweight='bold', pad=12)

    # ── Trim whitespace ──────────────────────────────────────────────────
    plt.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'[OK] Map quality image saved to: {OUTPUT_PATH}')
    print(f'     Dimensions: {width_m:.1f} m × {height_m:.1f} m  '
          f'({grid.shape[1]}×{grid.shape[0]} px @ {res*100:.0f} cm/px)')

if __name__ == '__main__':
    build_figure()
