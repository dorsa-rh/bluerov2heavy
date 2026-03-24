#computing mpc and returns outputs and this also can be achieved by subscribing outputs generated in /bluerov2heavy_mpc/..._node
import random
import time
#import numpy as np
from pymavlink import mavutil
import output_pub

def mpc_loop(sub, reference, dt, horizon):
    
    outputs = [0, 0, 0, 0, 0, 0]  # placeholder for control outputs
    '''msg = sub.recv_match(type='ATTITUDE', blocking=True)
    roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
    msg = sub.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    x, y, z = msg.x, msg.y, msg.z'''
    #... MPC computations to get control outputs u
    #...
    outputs[0] = random.uniform(-1, 1)  # pitch
    outputs[1] = random.uniform(-1, 1)  # roll
    outputs[2] = random.uniform(-1, 1)  # throttle
    outputs[3] = random.uniform(-1, 1)  # yaw
    outputs[4] = random.uniform(-1, 1)  # forward
    outputs[5] = random.uniform(-1, 1)  # lateral
    return outputs