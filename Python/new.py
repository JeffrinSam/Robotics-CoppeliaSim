import numpy as np

# Given joint values
theta1 = 0
theta2 = 0
theta3 = np.pi / 2
theta4 = -np.pi / 2

# Given link lengths
L1 = 1
L2 = 1
L3 = 1
L4 = 1

# Given wrench at the end-effector
Fb = np.array([10, 10, 10])

# Calculate sines and cosines
s2 = np.sin(theta2)
s23=np.sin(theta2+theta3)
s234=np.sin(theta2+theta3+theta4)
s3 = np.sin(theta3)
s34=np.sin(theta4+theta3)
s4 = np.sin(theta4)
c2 = np.cos(theta2)
c23=np.cos(theta2+theta3)
c234=np.cos(theta2+theta3+theta4)
c3 = np.cos(theta3)
c34=np.cos(theta4+theta3)
c4 = np.cos(theta4)

# Calculate the body Jacobian
Jb = np.array([[1,1,1,1],
[L3*s4+L2*s34+L1*s234,L3*s4+L2*s34,L3*s4,0],
[L4+L3*c4+L2*c34+L1*c234,L4+L3*c4+L2*c34,L4+L3*c4,L4]])
print(Jb.shape)
# Calculate the joint torques
tau = np.dot(Jb.T, Fb)

# Print the result
print(f"Joint Torques: {tau}")

import numpy as np

# Given numerical body Jacobian Jb
Jb = np.array([[0,-1, 0, 0,-1, 0, 0],
               [0, 0, 1, 0, 0, 1, 0],
               [1, 0, 0, 1, 0, 0, 1],
               [-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
               [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
               [0, -0.105, 0.889, 0, 0, 0, 0]])

# Extract Jw (angular velocity portion)
J_w = Jb[:3, :]

# Extract Jv (linear velocity portion)
J_v = Jb[3:6, :]
# Calculate angular velocity manipulability ellipsoid
A_v = np.dot(J_v, J_v.T)
# Calculate eigenvalues and eigenvectors
eigenvalues, eigenvectors = np.linalg.eig(A_v)

# Sort eigenvalues and corresponding eigenvectors
sorted_indices = np.argsort(eigenvalues)[::-1]
eigenvalues = eigenvalues[sorted_indices]
eigenvectors = eigenvectors[:, sorted_indices]

# Extract lengths of principal semi-axes
lengths = np.sqrt(eigenvalues)

# Extract direction of the longest principal semi-axis
max_index = np.argmax(lengths)
direction = eigenvectors[:, max_index]
unit_direction = direction / np.linalg.norm(direction)
length_of_longest_axis = lengths[max_index]

# Display the results
print("Lengths of principal semi-axes:")
print(lengths)
print("Direction of the longest principal semi-axis:")
print(np.round(unit_direction, 2))
print(f"Length of the longest principal semi-axis: {length_of_longest_axis:.2f}")
