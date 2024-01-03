import numpy as np

# Define the system of equations
def f(x, y):
    return np.array([x**2 - 9, y**2 - 4])

# Define the Jacobian matrix
def J(x, y):
    return np.array([[2*x, 0], [0, 2*y]])

# Define the initial guess
x0, y0 = 1, 1

# Perform two iterations of the Newton-Raphson method
for _ in range(2):
    # Calculate the update using the inverse of the Jacobian
    update = np.linalg.solve(J(x0, y0), -f(x0, y0))
    
    # Update the variables
    x0 += update[0]
    y0 += update[1]

# Print the result with at least 2 decimal places
print(f"Result after two iterations: [{x0:.2f}, {y0:.2f}]")
import modern_robotics as mr
import numpy as np

def IKbody(Blist, M, T, thetalist0, eomg, ev):
    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    Vb = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(mr.FKinBody(M, Blist.T, thetalist)) * T))

    # err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    # while err && i < maxiterations
    thetalist = thetalist + np.linalg.pinv(mr.JacobianBody(Blist.T, thetalist)) * Vb;
    i = i + 1;
    Vb = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(mr.FKinBody(M, Blist.T, thetalist)) * T));
    #     err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    # end
    # success = ~ err;
    # end
    return thetalist

Blist = np.array([[0,  0,  1,  0,  3,  0],  
                  [0,  0,  1,  0,  2,  0],  
                  [0,  0,  1,  0,  1,  0]]).T

M = np.array([[1, 0, 0, 3],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

T = np.array([[-0.585, -0.811, 0, 0.076],  
              [ 0.811, -0.585, 0, 2.608], 
              [     0,      0, 1,     0], 
              [     0,      0, 0,     1]])

thetalist0 = [0.7854, 0.7854, 0.7854]

eomg = 0.001 
ev = 0.0001
mr.IKinBody

print(mr.IKinBody(Blist, M, T, thetalist0, eomg, ev))