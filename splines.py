import math
import pybullet as p
from parameters import *

def lin_interp(q0, qd, t, T):
    s = t/T
    return q0 + s*(qd-q0)

def cubic_interp(q0, qd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    return q0 + s*(qd-q0)

def trap_interp(q0, qd, t, T):
    a = 2*4/T**2
    v = (a*T - math.sqrt(a)*math.sqrt(a*T**2-4))/2
    ta = v/a
    if t >= 0 and t <= ta:
        s = (a*t**2)/2
    if t > ta and t <= (T-ta):
        s = v*t - v**2/(2*a)
    if t > (T-ta):
        s = (2*a*v*T - 2*v**2 - a**2*(t-T)**2)/(2*a)
    return q0 + s*(qd-q0)

def get_inverse_kinematics(position, orientation):
	return p.calculateInverseKinematics(
		bodyIndex=body_id,
		endEffectorLinkIndex=eef_link_idx,
		targetPosition=position,
		targetOrientation=orientation
	)

def get_cart_pose():
	link_state = p.getLinkState(bodyUniqueId=body_id, 
								linkIndex=eef_link_idx)
	return (link_state[0], link_state[1])
