#!/usr/bin/env python3
import csv
import os
import sys
import matplotlib.pyplot as plt

CSV_FILE = os.path.join(os.path.dirname(__file__), 'trajectory_log.csv')

if len(sys.argv) > 1:
    CSV_FILE = sys.argv[1]

if not os.path.exists(CSV_FILE):
    print("CSV file not found:", CSV_FILE)
    sys.exit(1)

gt_x, gt_y, gt_z, gt_t = [], [], [], []
ref_x, ref_y, ref_z, ref_t = [], [], [], []

with open(CSV_FILE, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            t = float(row['time'])
            tag = row['type']
            x = float(row['x'])
            y = float(row['y'])
            z = float(row.get('z', '0'))
        except Exception:
            # skip malformed rows
            continue
        if tag == 'gt':
            gt_t.append(t); gt_x.append(x); gt_y.append(y); gt_z.append(z)
        elif tag == 'ref':
            ref_t.append(t); ref_x.append(x); ref_y.append(y); ref_z.append(z)

# --- XY plot (map) ---
plt.figure(figsize=(8, 6))
plt.xlabel('X')
plt.ylabel('Y')
plt.plot(gt_x, gt_y, label='Adaptive MPC (gt)', color='blue')
plt.plot(ref_x, ref_y, label='Reference', color='orange')
if gt_x:
    plt.scatter(gt_x[-1], gt_y[-1], color='blue')
if ref_x:
    plt.scatter(ref_x[-1], ref_y[-1], color='orange')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.title(os.path.basename(CSV_FILE))

# --- Tracking plots (x,y,z vs time) ---
# normalize time to start at zero
def normalize_time(t_list):
    if not t_list:
        return []
    t0 = t_list[0]
    return [tt - t0 for tt in t_list]

# choose common zero (prefer gt start, else ref start)
if gt_t:
    t0 = gt_t[0]
elif ref_t:
    t0 = ref_t[0]
else:
    t0 = 0.0

gt_time = [tt - t0 for tt in gt_t]
ref_time = [tt - t0 for tt in ref_t]

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
axs[0].plot(gt_time, gt_x, label='gt', color='blue')
axs[0].plot(ref_time, ref_x, label='ref', color='orange', linestyle='--')
axs[0].set_ylabel('X (m)')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(gt_time, gt_y, label='gt', color='blue')
axs[1].plot(ref_time, ref_y, label='ref', color='orange', linestyle='--')
axs[1].set_ylabel('Y (m)')
axs[1].legend()
axs[1].grid(True)

axs[2].plot(gt_time, gt_z, label='gt', color='blue')
axs[2].plot(ref_time, ref_z, label='ref', color='orange', linestyle='--')
axs[2].set_ylabel('Z (m)')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()