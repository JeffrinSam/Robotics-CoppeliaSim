import numpy as np
import modern_robotics as mr
Tf=5
t=3
Q=mr.QuinticTimeScaling(Tf, t)
print(Q)
Xstart = np.array([ [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
Xend = np.array([[0, 0, 1, 1],
                 [1, 0, 0, 2],
                 [0, 1, 0, 3],
                 [0, 0, 0, 1]])

Tf = 10
N = 10
method = 3
count=0
S=mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
rounded_S = np.round(S, decimals=3)

for i in rounded_S:
    count=count + 1
    print("\n",count,"\n",i)
    
method = 5
count2=0
C=mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
rounded_C = np.round(C, decimals=3)
for j in rounded_C:
    count2=count2 + 1
    print("\n",count2,"\n",j)



