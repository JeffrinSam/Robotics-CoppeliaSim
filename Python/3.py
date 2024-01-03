import numpy as np
import modern_robotics as mr
# Transformation matrices
import numpy as np

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
Mlist =[M01, M12, M23, M34, M45, M56, M67]
Slist =np.array([[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]])
# Concatenate along the third axis to form a 3D array
#Glist = np.concatenate([G1[np.newaxis, :, :], G2[np.newaxis, :, :], G3[np.newaxis, :, :],
#                        G4[np.newaxis, :, :], G5[np.newaxis, :, :], G6[np.newaxis, :, :]], axis=0)

# Assuming M01, M12, M23, M34, M45, M56, M67 are defined as in the previous example
#Mlist = np.concatenate([M01[np.newaxis, :, :], M12[np.newaxis, :, :], M23[np.newaxis, :, :],
#                       M34[np.newaxis, :, :], M45[np.newaxis, :, :], M56[np.newaxis, :, :], M67[np.newaxis, :, :]], axis=0)


# Print the matrices (optional)
print("Mlist:")
print(Mlist)

print("\nGlist:")
print(Glist)

print("\nSlist:")
print(Slist)

g = np.array([0, 0, -9.81])
thetalist = np.array([0,0.52359878,0.78539816,1.04719755,1.57079633,2.0943951 ])
dthetalist = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
ddthetalist=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
Ftip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
taulist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
                    
print("\ntaulist:\n",taulist)
rounded_taulist = np.round(taulist, decimals=2)
'''q=mr.MassMatrix(thetalist, Mlist, Glist, Slist)
print(q)
print("\nrounded_taulist:\n", rounded_taulist)
# Simulation
thetamat, dthetamat = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist,Glist, Slist)
# Output trajectory as a CSV file
columns = ['Theta1', 'Theta2', 'Theta3', 'Theta4', 'Theta5', 'Theta6']
df = pd.DataFrame(thetamat, columns=columns)
df.to_csv('forSimulation.csv', index=False)'''