import math
import pybullet as p
from parameters import *
import modern_robotics as mr

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

def s_curve_interp(rob, q0, qd, t, T):
    t_2 = T/6 # choose
    t_3 = T/3 # choose
    t_1 = (T - t_3 - 2*t_2)/4
    v = 2/(T + t_3)
    a = v/(t_1+t_2)
    J = a/t_1
    if 0 <= t <= t_1:
        s = (J*t**3)/6
    elif t_1 < t <= (t_2 + t_1):
        t_d = t - t_1
        s = (a*t_1**2)/6 + a*t_1*t_d/2 + (a*t_d**2)/2
    elif (t_2 + t_1) < t <= (t_2 + 2*t_1):
        t_d = t - (t_2 + t_1)
        s = (a*t_1**2)/6 + a*t_1*t_2/2 + (a*t_2**2)/2 + a*t_1*t_d/2 + a*t_2*t_d + (a*t_d**2)/2 - (J*t_d**3)/6
    elif (t_2 + 2*t_1) < t <= (t_3 + t_2 + 2*t_1):
        t_d = t - (t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_d
    elif (t_3 + t_2 + 2*t_1) < t <= (t_3 + t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_d - (J*t_d**3)/6
    elif (t_3 + t_2 + 3*t_1) < t <= (t_3 + 2*t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 3*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_1 - (a*t_1**2)/6 + v*t_d - a*t_1*t_d/2 - (a*t_d**2)/2
    else:
        t_d = t - (t_3 + 2*t_2 + 3*t_1)
        s = v*t_3 + v*t_2 + 2*v*t_1 - (a*t_1**2)/6 + a*t_1*t_d/2 - (a*t_d**2)/2 + (J*t_d**3)/6
    return q0 + s*(qd-q0)


def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def p2p_lin_screw(X0, Xd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    X = X0 @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(X0)@Xd)*s)

    pos = X[0:3,3]
    mat_rot = X[0:3,0:3]
    eul = rot2eul(mat_rot)

    quat = p.getQuaternionFromEuler(eul)
    joints = p.calculateInverseKinematics(
            body_id,
            endEffectorLinkIndex=eef_link_idx,
            targetPosition=pos,
            targetOrientation=quat
        )
    return joints

def p2p_lin_decoupled(X0, Xd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    # X = X0 @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(X0)@Xd)*s)

    pos0 = X0[0:3,3]
    pos1 = Xd[0:3,3]
    pos = pos0 + (pos1-pos0)*s
    mat_rot0 = X0[0:3,0:3]
    mat_rot1 = Xd[0:3,0:3]
    mat_rot = mat_rot0 @ mr.MatrixExp3(mr.MatrixLog3(mat_rot0.T@mat_rot1)*s)
    eul = rot2eul(mat_rot)

    quat = p.getQuaternionFromEuler(eul)
    joints = p.calculateInverseKinematics(
            body_id,
            endEffectorLinkIndex=eef_link_idx,
            targetPosition=pos,
            targetOrientation=quat
        )
    return joints

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

def pybullet_to_homogeneous(pos, quat):
    np_pos = np.array([pos]).T
    flat_rot = np.array(p.getMatrixFromQuaternion(quat))
    mat_rot = np.reshape(flat_rot, (3,3))
    X = np.concatenate((mat_rot, np_pos), axis = 1)
    X = np.concatenate((X, np.array([[0,0,0,1]])), axis = 0)
    return X

def get_homogeneous():
    return pybullet_to_homogeneous(*get_cart_pose())
