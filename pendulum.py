import time
import splines
import pybullet as p
from parameters import *

GUI = False

physicsClient = p.connect(p.GUI if GUI else p.DIRECT)
p.setGravity(0,0,-g)
body_id = p.loadURDF("./pendulum.urdf")

# go to the starting position
p.setJointMotorControlArray(bodyIndex=body_id, 
                        jointIndices=joints_idx, 
                        targetPositions=q0,
                        controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# j1 = -0.2:
# ((0.784355017578571, 0.15909353788366457, 0.9006243344056912), (-0.09983341664637793, 0.9950038521688175, -2.9979989403517104e-07, 0.0007893603961037777))

#j1 = 0.2:
# ((0.784355017578571, -0.15909353788366457, 0.9006243344056912), (0.09983341664637793, 0.9950038521688175, 2.9979989403517104e-07, 0.0007893603961037777))

q0 = [state[0] for state in p.getJointStates(body_id, joints_idx)]
print(f'q0: {q0}')
(pos0, orient0) = splines.get_cart_pose()
X0 = splines.get_homogeneous()
pos1 = [0.784355017578571, -0.15909353788366457, 0.9006243344056912]
orient1 = [0.09983341664637793, 0.9950038521688175, 2.9979989403517104e-07, 0.0007893603961037777]
Xd = splines.pybullet_to_homogeneous(pos1, orient0)
print(f'pos0: {(pos0, orient0)}')
print(f'pos1: {(pos1, orient0)}')
qd = np.array(splines.get_inverse_kinematics(pos1, orient0))
print(f'qd: {qd}')


for i in range(0,sz):
    full_state = p.getJointStates(body_id, joints_idx)
    pos[i] = [state[0] for state in full_state]
    vel[i] = [state[1] for state in p.getJointStates(body_id, joints_idx)]
    if i>0:
        acc[i] = [(vel[i][col] - vel[i-1][col])/dt for col in range(6)]
    xyz[i] = list(splines.get_cart_pose()[0])
    
    qq = qd
    if t <= trajTime:
        # qq = lin_interp(q0, qd, t, trajTime)
        # qq = cubic_interp(q0, qd, t, trajTime)
        # qq = splines.trap_interp(q0, qd, t, trajTime)
        # qq = splines.p2p_lin_screw(X0, Xd, t, trajTime)
        qq = splines.p2p_lin_decoupled(X0, Xd, t, trajTime)
        # print(f"qq: {qq}")

    p.setJointMotorControlArray(bodyIndex=body_id, 
                        jointIndices=joints_idx, 
                        targetPositions=qq,
                        controlMode=p.POSITION_CONTROL)

    p.stepSimulation()
    if GUI:
        time.sleep(dt)
    t += dt
    tt[i] = t
p.disconnect()

import matplotlib.pyplot as plt

def plot(t, vals, title):
    fig, ax = plt.subplots(3, 2,
                        sharex='col', 
                        sharey='row',
                        num=title)
    for row in range(3):
        for col in range(2):
            num = row + 3*col
            ax[row, col].plot(tt, [r[num] for r in vals])
            ax[row, col].grid(True)
            ax[row, col].set(title="Joint"+str(num+1))

# joints plots
plot(tt, pos, 'Joints')
plot(tt, vel, 'Velocity')
plot(tt, acc, 'Acceleration')

# cartesian plot (XY-plane)
plt.figure('xy')
plt.plot([x[0] for x in xyz], [x[1] for x in xyz])
# plt.xlim(0, 1)
# plt.ylim(-0.5,0.5)
plt.grid(True)

plt.show()