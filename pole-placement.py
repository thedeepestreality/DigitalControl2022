from control.matlab import place
import numpy as np
g = 10
L = 0.8
m = 1
A = np.array(([0, 1], 
              [-g/L, 0]))
B = np.array(([0], [m/L**2]))
poles = np.array([-4,-8])
K = place(A,B,poles)
print(K)

poles_fact = np.linalg.eig(A-B@K)
print(poles_fact)