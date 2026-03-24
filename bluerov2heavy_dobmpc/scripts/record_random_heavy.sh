#!/bin/bash
# record_random_heavy.sh
# =======================
# Records all topics needed for RLS-VFF system identification
# on the BlueROV2 Heavy (8 thrusters, 6 DOF).
#
# CHANGES vs record_random.sh:
#   - Namespace: /bluerov2heavy/ (was /bluerov2/)
#   - Thruster topics: 0-7 (was 0-5)
#   - Control inputs:  0-7 (was 0-3)
#   - Bag name: single_dof_heavy_1

rosbag record -O single_dof_heavy_1 \
    /bluerov2heavy/pose_gt \
    /bluerov2heavy/thrusters/0/thrust \
    /bluerov2heavy/thrusters/1/thrust \
    /bluerov2heavy/thrusters/2/thrust \
    /bluerov2heavy/thrusters/3/thrust \
    /bluerov2heavy/thrusters/4/thrust \
    /bluerov2heavy/thrusters/5/thrust \
    /bluerov2heavy/thrusters/6/thrust \
    /bluerov2heavy/thrusters/7/thrust \
    /bluerov2heavy/thrusters/0/input \
    /bluerov2heavy/thrusters/1/input \
    /bluerov2heavy/thrusters/2/input \
    /bluerov2heavy/thrusters/3/input \
    /bluerov2heavy/thrusters/4/input \
    /bluerov2heavy/thrusters/5/input \
    /bluerov2heavy/thrusters/6/input \
    /bluerov2heavy/thrusters/7/input \
    /bluerov2heavy/control_input/0 \
    /bluerov2heavy/control_input/1 \
    /bluerov2heavy/control_input/2 \
    /bluerov2heavy/control_input/3 \
    /bluerov2heavy/control_input/4 \
    /bluerov2heavy/control_input/5 \
    /bluerov2heavy/control_input/6 \
    /bluerov2heavy/control_input/7