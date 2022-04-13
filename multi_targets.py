import numpy as np
import pybullet as p
from parameters import *

def set_traj_control_interp(q0, target_traj, t):
    sz = len(target_traj)
    prev_pos = q0
    prev_vel = np.array([0,0,0,0,0,0])
    t0 = 0
    for idx in range(sz):
        target = target_traj[idx]
        curr_pos = np.array(target["pos"])
        next_pos = prev_pos
        ts = target["ts"]
        t1 = ts
        if (idx < sz-1):
            next_pos = np.array(target_traj[idx+1]["pos"])
        curr_vel = next_pos-prev_pos
        
        norm = np.linalg.norm(curr_vel)
        if (norm > 1e-4):
            curr_vel /= norm
        else:
            curr_vel = np.array([0,0,0,0,0,0])

        if (t >= t0 and t < t1 ):
            return traj_segment_cubic(prev_pos, prev_vel, curr_pos, curr_vel, ts-t0, t-t0)
        t0 = t1
        prev_pos = curr_pos
        prev_vel = curr_vel
        
def traj_segment_cubic(q_start, vel_start, q_end, vel_end, ts, t):
    b0 = q_start
    db0 = vel_start
    b1 = q_end
    db1 = vel_end

    a0 = b0
    a1 = db0
    a2 = (3*b1 - 3*b0 - 2*db0*ts - db1*ts)/(ts**2)
    a3 = (2*b0 + (db0 + db1)*ts - 2*b1)/(ts**3)

    return a0 + a1*t + a2*t**2 + a3*t**3
