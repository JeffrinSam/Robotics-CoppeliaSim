import numpy as np
import modern_robotics as mr

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev, T_sd):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    
    # Initialize the matrix to store joint vectors for each iteration
    joint_matrix = np.zeros((maxiterations + 1, len(thetalist)))

    while i <= maxiterations:
        # Save joint vector for the current iteration
        joint_matrix[i] = thetalist.copy()

        # Calculate the end-effector configuration
        Tsb = mr.FKinBody(M, Blist, thetalist)
        
        # Calculate the error twist
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
        
        # Calculate angular and linear error magnitudes
        angular_error = np.linalg.norm(Vb[:3])
        linear_error = np.linalg.norm(Vb[3:])
        
        # Print iteration report
        print(f"Iteration {i}:")
        print(f"Joint vector: {', '.join(map(str, thetalist))}")
        print("SE(3) end-effector config:")
        print(Tsb)
        print(f"Error twist V_b: {', '.join(map(str, Vb))}")
        print(f"Angular error magnitude ||omega_b||: {angular_error}")
        print(f"Linear error magnitude ||v_b||: {linear_error}\n")
        
        # Check convergence criteria
        if angular_error <= eomg and linear_error <= ev:
            print("Converged!")
            break

        # Update joint vector using Newton-Raphson iteration
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        
        i += 1

    # Save joint vectors as a .csv file
    np.savetxt('iterates2.csv', joint_matrix, delimiter=',')

# Define the input matrices and parameters
Blist = np.array([[0, 0, -1, 2, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0.1]]).T
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
T = np.array([[0, 1, 0, -0.5], [0, 0, -1, 0.1], [-1, 0, 0, 0.1], [0, 0, 0, 1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001

# Desired end-effector configuration
T_sd = np.array([[0, 1, 0, -0.5],
                 [0, 0, -1, 0.1],
                 [-1, 0, 0, 0.1],
                 [0, 0, 0, 1]])

IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev, T_sd)
