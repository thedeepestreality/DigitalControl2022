import pybullet as p
import time
import numpy as np
from scipy.integrate import odeint
import math

dt = 1/240 # pybullet simulation step
q0 = 0.5
maxTime = 10
g = 10
L = 0.8
m = 1
tt = np.arange(0, maxTime+2*dt, dt)
pos = [q0]
vel = [0]
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

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex = bodyId, 
                        jointIndex = 1, 
                        targetVelocity = 0, 
                        controlMode = p.VELOCITY_CONTROL, 
                        force = 0)
qd = 0.4
while t < maxTime:
    # u = 1/dt
    # if t > 0:
    #     u = 0
    q = p.getJointState(bodyId, 1)[0]
    w = p.getJointState(bodyId, 1)[1]
    u = -2*(q-qd) - 2*w
    tau_nonlin = m*g*L*math.sin(q)
    tau_lin = m*L*L*u
    tau = tau_nonlin + tau_lin
    p.setJointMotorControl2(bodyIndex = bodyId, 
                            jointIndex = 1,
                            controlMode = p.TORQUE_CONTROL, 
                            force = tau)
    pos.append(q)
    vel.append(w)
    p.stepSimulation()
    # time.sleep(dt)
    t += dt
p.disconnect()

def rp(X, t):
    dx = [X[1], -g/L*math.sin(X[0])]
    return dx

Y = odeint(rp, [q0,0], tt)

import matplotlib.pyplot as plt
plt.figure("pos")
plt.plot(tt, pos)
plt.grid(True)
# plt.plot(tt, Y[:,0])

plt.figure("vel")
plt.plot(tt, vel)
plt.grid(True)

plt.show()