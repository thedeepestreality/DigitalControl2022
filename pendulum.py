import pybullet as p
import time
import math

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
acc = [None]*sz
acc[0] = [0]*6

t = 0
qd = 0.2
joints_idx = [1,2,3,4,5,6]

# physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
physicsClient = p.connect(p.DIRECT)
p.setGravity(0,0,-g)
bodyId = p.loadURDF("./pendulum.urdf")

# go to the starting position
p.setJointMotorControl2(bodyIndex=bodyId, 
                        jointIndex=1, 
                        targetPosition=q0,
                        controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
q0 = p.getJointState(bodyId, 1)[0]
print(f'q0: {q0}')

for i in range(0,sz):
    q = p.getJointState(bodyId, 1)[0]
    w = p.getJointState(bodyId, 1)[1]
    pos[i] = [state[0] for state in p.getJointStates(bodyId, joints_idx)]
    vel[i] = [state[1] for state in p.getJointStates(bodyId, joints_idx)]
    if i>0:
        acc[i] = [(vel[i][col] - vel[i-1][col])/dt for col in range(6)]
    
    qq = qd
    if t <= trajTime:
        # qq = lin_interp(q0, qd, t, trajTime)
        # qq = cubic_interp(q0, qd, t, trajTime)
        qq = trap_interp(q0, qd, t, trajTime)
        # print(f"qq: {qq}")

    p.setJointMotorControl2(bodyIndex=bodyId, 
                        jointIndex=1, 
                        targetPosition=qq,
                        controlMode=p.POSITION_CONTROL)

    p.stepSimulation()
    # time.sleep(dt)
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

plot(tt, pos, 'Joints')
plot(tt, vel, 'Velocity')
plot(tt, acc, 'Acceleration')

plt.show()