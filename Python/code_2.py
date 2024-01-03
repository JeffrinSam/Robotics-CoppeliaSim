import numpy as np
import modern_robotics as mr

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0)
    i = 0
    maxiterations = 20
    Tsb = mr.FKinBody(M, Blist, thetalist)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
    err = np.linalg.norm(Vb[:3]) > eomg or np.linalg.norm(Vb[3:]) > ev

    # Print initial iteration
    print(f"Iteration {i}:\n")
    print("Joint vector:")
    print(thetalist)
    print("\nSE(3) end-effector config:")
    print(Tsb)
    print("\nError twist V_b:")
    print(f"{Vb[0]}, {Vb[1]}, {Vb[2]}, {Vb[3]}, {Vb[4]}, {Vb[5]}\n")
    print(f"Angular error magnitude ||omega_b||: {eomg}")
    print(f"Linear error magnitude ||v_b||: {ev}\n")

    # Initialize matrix to store joint vectors
    finalmatx = np.array([thetalist])

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        i += 1
        Tsb = mr.FKinBody(M, Blist, thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
        err = np.linalg.norm(Vb[:3]) > eomg or np.linalg.norm(Vb[3:]) > ev

        # Print iteration
        print(f"Iteration {i}:\n")
        print("Joint vector:")
        print(thetalist)
        print("\nSE(3) end-effector config:")
        print(Tsb)
        print("\nError twist V_b:")
        print(f"{Vb[0]}, {Vb[1]}, {Vb[2]}, {Vb[3]}, {Vb[4]}, {Vb[5]}\n")
        print(f"Angular error magnitude ||omega_b||: {eomg}")
        print(f"Linear error magnitude ||v_b||: {ev}\n")

        # Append thetalist to the matrix
        finalmatx = np.vstack([finalmatx, thetalist])

        # Check convergence criteria
        if np.linalg.norm(Vb[:3]) <= eomg and np.linalg.norm(Vb[3:]) <= ev:
            print("Converged!")
            break

    success = not err
    # Save joint vectors as a .csv file
    np.savetxt('iterates.csv', finalmatx, delimiter=',')

# Example usage
Blist = np.array([[0, 0, -1, 2, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0.1]]).T
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
T = np.array([[0, 1, 0, -5], [1, 0, 0, 4], [0, 0, -1, 1.6858], [0, 0, 0, 1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001

IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
