import pybullet as p
import time
import numpy as np
from scipy.integrate import odeint
import math
import copy

dt = 1/240 # pybullet simulation step
q0 = 0.1
maxTime = 10
g = 10
L = 0.8
m = 1
sz = int(maxTime/dt)
tt = [None]*sz
pos = [None]*sz
t = 0
e_int = 0

# physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
physicsClient = p.connect(p.DIRECT)
p.setGravity(0,0,-g)
bodyId = p.loadURDF("./pendulum.urdf")

# get rid of all the default damping forces
p.changeDynamics(bodyId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(bodyId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=bodyId, 
                        jointIndex=1, 
                        targetPosition=q0,
                        controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
q0 = p.getJointState(bodyId, 1)[0]
print(f'q0: {q0}')
qd = 0.78

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex = bodyId, 
                        jointIndex = 1, 
                        targetVelocity = 0, 
                        controlMode = p.VELOCITY_CONTROL, 
                        force = 0)
for i in range(0,sz):
    q = p.getJointState(bodyId, 1)[0]
    w = p.getJointState(bodyId, 1)[1]
    pos[i] = q
    e = q - qd
    e_int += e*dt
    kp = 20
    ki = 30
    kd = 4
    p.setJointMotorControl2(bodyIndex = bodyId, 
                        jointIndex = 1, 
                        controlMode = p.TORQUE_CONTROL, 
                        force = -kp*e -ki*e_int -kd*w)
    p.stepSimulation()
    # time.sleep(dt)
    t += dt
    tt[i] = t
p.disconnect()

def rp(X, t):
    # dx = [X[1], -g/L*math.sin(X[0])]
    dx = [X[1], -g/L*X[0]]
    return dx

# symplectic euler method
def symp_euler(fun, x0, TT):
    x1 = copy.copy(x0)
    xx = np.array(x1)
    for i in range(len(TT)-1):
        dt = (TT[i+1]-TT[i])
        x1[1] += rp(x1, 0)[1]*dt
        x1[0] += x1[1]*dt
        xx = np.vstack((xx,x1))
    return xx

# Y = odeint(rp, [q0,0], tt)
# Y = symp_euler(rp, [q0, 0], tt)

import matplotlib.pyplot as plt
plt.plot(tt, pos)
plt.grid(True)
# plt.plot(tt, Y[:,0])
plt.show()