import numpy as np
import modern_robotics as mr

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist =np.array([[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]])
g = np.array([0, 0, -9.81])
thetalist = np.array([0,0.52359878,0.78539816,1.04719755,1.57079633,2.0943951 ])
dthetalist = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
ddthetalist=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
Ftip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

Mass=mr.MassMatrix(thetalist, Mlist, Glist, Slist)
rounded_Mass = np.round(Mass, decimals=4)
print("\nrounded_Mass:\n", rounded_Mass)


V=mr.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
rounded_V = np.round(V, decimals=3)
print("\nrounded_V:\n", rounded_V)

G=mr.GravityForces(thetalist, g, Mlist, Glist, Slist)
rounded_G = np.round(G, decimals=3)
print("\nrounded_G:\n", rounded_G)

E=mr.EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist)
rounded_E = np.round(E, decimals=3)
print("\nrounded_E:\n", rounded_E)
Slist =np.array([[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]])
tau_matrix = np.array([0.0128,-41.1477,-3.7809,0.0323,0.0370,0.1034])
FD=mr.ForwardDynamics(thetalist, dthetalist, tau_matrix, g, Ftip, Mlist,
                    Glist, Slist)
rounded_FD = np.round(FD, decimals=2)
print("\nrounded_FD:\n", rounded_FD)
