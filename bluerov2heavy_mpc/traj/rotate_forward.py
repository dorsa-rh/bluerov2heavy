#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import math
#import tf.transformations as tf


# Parameters
sample_time = 0.05             # seconds
duration = 240                   # seconds

#r = 2
v = 1
p_v = 0.15
q_v = 0.15
x0 = 0                       
y0 = 0
z0 = -5

# trajectory
traj = np.zeros((int(duration/sample_time+1),18)) # x y z phi theta psi u v w p q r u1 u2 u3 u4 u5 u6
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = v*t    # x
traj[:,1] = 0       #3*np.sin(t*v/1)+y0    # y
traj[:,2] = -5     #-3*np.sin(t*v/1)+z0                      # z
traj[:,3] = 0.5*np.pi*np.sin(t*p_v/1)                       # phi
traj[:,4] = 0                    # theta
traj[:,5] =0         # psi
traj[:,6] = 0      # u
traj[:,7] = 0     # v
traj[:,8] = 0                       # w
traj[:,9] = 0                       # p
traj[:,10] = 0                      # q
traj[:,11] = 0                      # r
traj[:,12] = 0                      # u1
traj[:,13] = 0                      # u2
traj[:,14] = 57.5                   # u3
traj[:,15] = 0                      # u4
'''
for i in range(0,int(duration/sample_time+1)):
    if np.sin(traj[i,5])>=0:
        traj[i,5] = traj[i,5]%np.pi
    else:
        traj[i,5] = -np.pi + traj[i,5]%np.pi
    #traj[i,5] = np.sin(traj[i,5])
'''
# write to txt
np.savetxt('rotate_forward.txt',traj,fmt='%f')