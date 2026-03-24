#!/usr/bin/env python3
"""
compare_model_mismatch.py
=========================
Compares model-plant mismatch between two runs (e.g. baseline vs improved model).

Usage
-----
# Compare two runs:
python3 compare_model_mismatch.py ~/results_circle_baseline.csv ~/results_circle_improved.csv

# Single run only:
python3 compare_model_mismatch.py ~/results_circle_baseline.csv

CSV format expected (written by bluerov2heavy_mpc_node.cpp):
    time,
    ref_x, ref_y, ref_z, ref_phi, ref_theta, ref_psi,
    real_x, real_y, real_z, real_phi, real_theta, real_psi,
    pred_x, pred_y, pred_z, pred_phi, pred_theta, pred_psi,
    u1, u2, u3, u4, u5, u6, u7, u8

Model-plant mismatch definition
--------------------------------
At each timestep i the MPC predicts what the state will be at i+1.
That prediction is stored in pred_* columns of row i.
The actual state at i+1 is stored in real_* columns of row i+1.
Mismatch at step i = real[i+1] - pred[i]

This metric is independent of the controller — it measures purely
how accurately the dynamics model predicts vehicle behaviour.
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── colour scheme ──────────────────────────────────────────────────────────────
COLOURS = {
    'baseline': '#E24B4A',   # red
    'improved':  '#1D9E75',  # teal
}

# ── helper ─────────────────────────────────────────────────────────────────────
def load(path):
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    return df


def compute_mismatch(df):
    """
    Returns a DataFrame of mismatch values.
    mismatch[i] = real[i+1] - pred[i]   (shift pred forward by one row)
    The last row is dropped (no real[i+1] for the final prediction).
    """
    real_cols = ['real_x', 'real_y', 'real_z', 'real_phi', 'real_theta', 'real_psi']
    pred_cols = ['pred_x', 'pred_y', 'pred_z', 'pred_phi', 'pred_theta', 'pred_psi']
    labels    = ['x', 'y', 'z', 'phi', 'theta', 'psi']

    real_next = df[real_cols].iloc[1:].values          # real state at t+1
    pred_curr = df[pred_cols].iloc[:-1].values          # model prediction for t+1
    time      = df['time'].iloc[1:].values              # time axis aligned to t+1

    mismatch = real_next - pred_curr

    result = pd.DataFrame(mismatch, columns=labels)
    result['time'] = time - time[0]                     # normalise to start at 0
    return result


def rmse(arr):
    return np.sqrt(np.mean(arr ** 2))


def mae(arr):
    return np.mean(np.abs(arr))


# ── main ───────────────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 compare_model_mismatch.py <baseline.csv> [improved.csv]")
        sys.exit(1)

    files = sys.argv[1:]
    runs  = {}

    for i, fpath in enumerate(files[:2]):
        if not os.path.exists(fpath):
            print(f"File not found: {fpath}")
            sys.exit(1)
        label = 'baseline' if i == 0 else 'improved'
        df = load(fpath)
        runs[label] = compute_mismatch(df)
        print(f"\n── {label.upper()}  ({os.path.basename(fpath)}) ──")
        for col in ['x', 'y', 'z', 'phi', 'theta', 'psi']:
            r = rmse(runs[label][col].values)
            m = mae(runs[label][col].values)
            print(f"   {col:6s}  RMSE={r:.5f}   MAE={m:.5f}")

    # ── Figure 1: Position mismatch (x, y, z) ──────────────────────────────────
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle('Model-plant mismatch — position (real[t+1] − predicted[t+1])',
                  fontsize=13)

    pos_labels = ['x (m)', 'y (m)', 'z (m)']
    pos_cols   = ['x', 'y', 'z']

    for ax, col, ylabel in zip(axes1, pos_cols, pos_labels):
        for label, mm in runs.items():
            ax.plot(mm['time'], mm[col],
                    color=COLOURS[label], label=label, alpha=0.8, linewidth=0.9)
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes1[-1].set_xlabel('Time (s)')
    fig1.tight_layout()

    # ── Figure 2: Attitude mismatch (phi, theta, psi) ──────────────────────────
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle('Model-plant mismatch — attitude (real[t+1] − predicted[t+1])',
                  fontsize=13)

    att_labels = ['roll φ (rad)', 'pitch θ (rad)', 'yaw ψ (rad)']
    att_cols   = ['phi', 'theta', 'psi']

    for ax, col, ylabel in zip(axes2, att_cols, att_labels):
        for label, mm in runs.items():
            ax.plot(mm['time'], mm[col],
                    color=COLOURS[label], label=label, alpha=0.8, linewidth=0.9)
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes2[-1].set_xlabel('Time (s)')
    fig2.tight_layout()

    # ── Figure 3: RMS mismatch bar chart summary ────────────────────────────────
    all_cols = ['x', 'y', 'z', 'phi', 'theta', 'psi']
    col_labels = ['x', 'y', 'z', 'φ', 'θ', 'ψ']
    n_cols  = len(all_cols)
    x_pos   = np.arange(n_cols)
    bar_w   = 0.35

    fig3, ax3 = plt.subplots(figsize=(10, 5))
    fig3.suptitle('RMSE summary — model-plant mismatch per state', fontsize=13)

    for offset, (label, mm) in zip([-bar_w/2, bar_w/2], runs.items()):
        rmses = [rmse(mm[c].values) for c in all_cols]
        ax3.bar(x_pos + offset, rmses, bar_w,
                label=label, color=COLOURS[label], alpha=0.85)

    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(col_labels)
    ax3.set_ylabel('RMSE')
    ax3.legend()
    ax3.grid(True, axis='y', alpha=0.3)
    fig3.tight_layout()

    # ── Figure 4: Rolling RMSE (mismatch over time windows) ────────────────────
    window = 100   # ~5 seconds at 20 Hz

    fig4, axes4 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig4.suptitle(f'Rolling RMSE (window = {window} steps ≈ 5 s)', fontsize=13)

    pos_cols3 = ['x', 'y', 'z']
    att_cols3  = ['phi', 'theta', 'psi']

    for label, mm in runs.items():
        # position rolling RMSE
        pos_rms = mm[pos_cols3].pow(2).mean(axis=1).pow(0.5).rolling(window).mean()
        att_rms = mm[att_cols3].pow(2).mean(axis=1).pow(0.5).rolling(window).mean()
        axes4[0].plot(mm['time'], pos_rms,
                      color=COLOURS[label], label=label, linewidth=1.2)
        axes4[1].plot(mm['time'], att_rms,
                      color=COLOURS[label], label=label, linewidth=1.2)

    axes4[0].set_ylabel('Position mismatch\nRMSE (m)')
    axes4[0].legend()
    axes4[0].grid(True, alpha=0.3)
    axes4[1].set_ylabel('Attitude mismatch\nRMSE (rad)')
    axes4[1].legend()
    axes4[1].grid(True, alpha=0.3)
    axes4[-1].set_xlabel('Time (s)')
    fig4.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()
