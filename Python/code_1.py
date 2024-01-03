import numpy as np
from numpy.linalg import norm, pinv
from modern_robotics import FKinBody, MatrixLog6, TransInv, JacobianBody, se3ToVec

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0)
    i = 0
    maxiterations = 20
    Tsb = FKinBody(M, Blist, thetalist)
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))
    err = norm(Vb[:3]) > eomg or norm(Vb[3:]) > ev

    # Initial guess
    print(f'Iteration {i} :\n')
    print('Joint vector :')
    print(thetalist)
    print('\nSE(3) end-effector config: ')
    print(Tsb)
    print('\nError twist V_b:\n')
    print(f'{Vb[0]},{Vb[1]},{Vb[2]},{Vb[3]},{Vb[4]},{Vb[5]}\n')
    print(f'Angular error magnitude ||omega_b||: {eomg}\n')
    print(f'Linear error magnitude ||v_b||: {ev}\n')

    # Initializing vector matrix
    finalmatx = np.array([thetalist])

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(pinv(JacobianBody(Blist, thetalist)), Vb)
        i += 1
        Tsb = FKinBody(M, Blist, thetalist)
        Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))
        err = norm(Vb[:3]) > eomg or norm(Vb[3:]) > ev

        # Iteration output
        print(f'Iteration {i} :\n')
        print('Joint vector :')
        print(thetalist)
        print('\nSE(3) end-effector config: ')
        print(Tsb)
        print('\nError twist V_b:\n')
        print(f'{Vb[0]},{Vb[1]},{Vb[2]},{Vb[3]},{Vb[4]},{Vb[5]}\n')
        print(f'Angular error magnitude ||omega_b||: {eomg}\n')
        print(f'Linear error magnitude ||v_b||: {ev}\n')

        # Append thetalist to the matrix
        finalmatx = np.vstack([finalmatx, thetalist])

    success = not err
    # Create the CSV file
    np.savetxt('iterates.csv', finalmatx, delimiter=',')

# Example usage
Blist = np.array([[0, 0, -1, 2, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0.1]]).T
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
T = np.array([[0, 1, 0, -5], [1, 0, 0, 4], [0, 0, -1, 1.6858], [0, 0, 0, 1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001

IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
