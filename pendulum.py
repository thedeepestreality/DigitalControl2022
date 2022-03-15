import pybullet as p
import time
import numpy as np
from scipy.integrate import odeint
import math
import copy

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

dt = 1/240 # pybullet simulation step
q0 = 0.1
maxTime = 10
trajTime = 5
g = 10
L = 0.8
m = 1
sz = int(maxTime/dt)
tt = [None]*sz
pos = [None]*sz
vel = [None]*sz
acc = [0]*sz
t = 0

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
qd = 0.2
# turn off the motor for the free motion
# p.setJointMotorControl2(bodyIndex = bodyId, 
#                         jointIndex = 1, 
#                         targetVelocity = 0, 
#                         controlMode = p.VELOCITY_CONTROL, 
#                         force = 0)
for i in range(0,sz):
    q = p.getJointState(bodyId, 1)[0]
    w = p.getJointState(bodyId, 1)[1]
    pos[i] = q
    vel[i] = w
    if i>0:
        acc[i] = (vel[i] - vel[i-1])/dt
    
    qq = qd
    if t <= trajTime:
        # qq = lin_interp(q0, qd, t, trajTime)
        # qq = cubic_interp(q0, qd, t, trajTime)
        qq = trap_interp(q0, qd, t, trajTime)
        # qq = q0 + t*(qd-q0)/trajTime
    p.setJointMotorControl2(bodyIndex=bodyId, 
                        jointIndex=1, 
                        targetPosition=qq,
                        controlMode=p.POSITION_CONTROL)
    # Q = np.vstack((q,w))

    # kq = 12.48  
    # kw = 7.68
    # p.setJointMotorControl2(bodyIndex = bodyId, 
    #                     jointIndex = 1, 
    #                     controlMode = p.TORQUE_CONTROL, 
    #                     force = -kq*q -kw*w)
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
plt.figure("pos")
plt.plot(tt, pos)
plt.grid(True)
# plt.plot(tt, Y[:,0])

plt.figure("vel")
plt.plot(tt, vel)
plt.grid(True)

plt.figure("acc")
plt.plot(tt, acc)
plt.grid(True)

plt.show()