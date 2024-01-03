import modern_robotics as mr
import numpy as np

blist = np.array([[0,0,0,0] ,[0,0,0,0] , [0,0,-1,0] ,[0,0,0,0]])

tlist = np.array([1.57, 1.57, 1])

#Js = mr.JacobianBody(blist,tlist)
Ms = mr.MatrixExp6(blist)
Adj = mr.Adjoint(Ms)
print(Adj)

b3 = np.array([0,0,0,0,0,1])
x = np.array([[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, -1],[0, 0, 0, 0]])
at = np.array([[ 1,  0,  0,  0],[ 0,  1,  0,  0], [ 0,  0,  1, -1], [ 0,  0,  0,  1]])
adj = mr.Adjoint(at)
print(adj)


slist = np.array([[ 0,  0,  0],[ 0,  0,  0], [ 1,  1,  1] ,[ 0,  0,  0],[ 0,  -1,  -2],[ 0,  0,  0]])
m = np.array([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
t = np.array([[-0.585, -0.811, 0, 0.076], [0.811, -0.585, 0, 2.608], [0, 0, 1, 0], [0, 0, 0, 1]])
tnot = np.array([[0.7854],[0.7854],[0.7854]])
eomg = 0.001
ev = 0.0001

Slist = np.array([[0, 0,  1,  4, 0,    0],
                          [0, 0,  0,  0, 1,    0],
                          [0, 0, -1, -6, 0, -0.1]]).T
M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001
Js = mr.JacobianBody(Slist,thetalist0)

ik = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
print(Js)
print(ik)

import numpy as np

# Given body Jacobian Jb
Jb = np.array([[0, -1, 0, 0, -1, 0, 0],
               [-1, 0, 0, 0, 0, 1, 0],
               [0, 0, 1, -0.105, -0.889, 0, 0.006],
               [0, 0, 1, -0.045, -0.844, 0, 0.006],
               [0, 0, 1, 0, 0, 0, 0]])

# Extract linear velocity portion Jv
Jv = Jb[3:6, :]

# Perform singular value decomposition
_, _, V = np.linalg.svd(Jv)

# The first column of V represents the direction of the longest principal semi-axis
direction_longest_axis = V[:, 0]

# Print the result
print("Direction of the longest principal semi-axis:")
print(np.round(direction_longest_axis, 2))
import numpy as np

# Assuming singular values obtained from SVD
singular_values = np.array([0.5, 0.3, 0.1])  # Replace this with the actual singular values

# Length of the longest principal semi-axis (maximum singular value)
length_longest_axis = np.max(singular_values)

# Print the result with at least 2 decimal places
print(f"Length of the longest principal semi-axis: {length_longest_axis:.2f}")
import numpy as np

# Part (a)
F_s_a = np.array([0, 0, 0, 2, 0, 0])
Slist = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 0, 1, 0, -1, 0],
                  [0, 0, 1, 0, -2, 0],
                  [1, 0, 0, 0, 0, 0]]).T
thetalist = np.array([0, np.pi/4, 0, ])

# Calculate the space Jacobian
J_s_T = np.transpose(mr.JacobianSpace(Slist, thetalist))
J_b_T = np.transpose(mr.JacobianBody(Slist, thetalist))
F_b=np.array([0, 0, 10, 10, 10, 0])
torqwue= np.dot(J_b_T, F_b)
# Calculate the joint torques for Part (a)
tau_a = np.dot(J_s_T, F_s_a)

# Part (b)
F_s_b = np.array([0, 0, 0, 0, 5, 0])

# Calculate the joint torques for Part (b)
tau_b = np.dot(J_s_T, F_s_b)

# Print the results
print("Part (a) Joint Torques:", tau_a)
print("Part (b) Joint Torques:", torqwue)
