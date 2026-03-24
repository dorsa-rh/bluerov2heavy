#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry  # replace with the actual message type of pose_gt and mpc_reference topics
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv
import os
import time

real_x, real_y = [], []
ref_x, ref_y = [], []
gt_start = False
ref_start = False

CSV_FILE = os.path.join(os.path.dirname(__file__), 'trajectory_log.csv')

# create file and header if it doesn't exist
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'type', 'x', 'y', 'z'])

def write_row_to_csv(t, tag, x, y, z):
    # append a single row and flush immediately
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([t, tag, x, y, z])
        f.flush()

def gt_callback(data):
    global real_x, real_y
    global gt_start
    gt_start = True
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    # timestamp: prefer header stamp if available
    try:
        t = data.header.stamp.to_sec()
    except:
        t = rospy.Time.now().to_sec()
    real_x.append(x)
    real_y.append(y)
    write_row_to_csv(t, 'gt', x, y, z)
    
    
def ref_callback(data):
    global ref_start
    ref_start = True
    global ref_x, ref_y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    try:
        t = data.header.stamp.to_sec()
    except:
        t = rospy.Time.now().to_sec()
    ref_x.append(x)
    ref_y.append(y)
    write_row_to_csv(t, 'ref', x, y, z)
    
    
rospy.init_node('plotter', anonymous=True)
rospy.Subscriber('/bluerov2heavy/pose_gt', Odometry, gt_callback)
rospy.Subscriber('/bluerov2heavy/mpc/reference', Odometry, ref_callback)

fig, ax = plt.subplots()
plt.ion()

while not rospy.is_shutdown():
    if gt_start==True and ref_start==True:
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim([-22, 22])
        ax.set_ylim([-22, 22])
        ax.plot(real_x[-200:], real_y[-200:], label='Adaptive MPC', color='blue')
        ax.plot(ref_x[-200:], ref_y[-200:], label='Reference', color='orange')
        if real_x:
            ax.scatter(real_x[-1], real_y[-1], color='blue', marker='o')
        if ref_x:
            ax.scatter(ref_x[-1], ref_y[-1], color='orange', marker='o')
        ax.legend()
        plt.draw()
        plt.pause(0.05)
    else:
        # keep loop responsive
        time.sleep(0.05)

plt.ioff()
plt.show()