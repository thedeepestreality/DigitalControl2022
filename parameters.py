import numpy as np

dt = 1/240 # pybullet simulation step
q0 = np.array([0,0,1.57,0,1.57,0])
joints_idx = [1,2,3,4,5,6]
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
xyz = [None]*sz
acc[0] = [0]*6

t = 0

eef_link_idx = 7
global body_id
body_id = 0