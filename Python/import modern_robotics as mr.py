import modern_robotics as mr
import numpy as np

# Define the transformation matrix
m = np.array([[1, 0, 0, 3.732],
              [0, 1, 0, 0],
              [0, 0, 1, 2.732],
              [0, 0, 0, 1]])

# Define the screw axes for spatial and body frames
slist = np.array([[0, 0, 0, 0, 0, 0],
                  [0, 1, 1, 1, 0, 0],
                  [1, 0, 0, 0, 0, 1],
                  [0, 0, 1, -0.732, 0, 0],
                  [-1, 0, 0, 0, 0, -3.732],
                  [0, 1, 2.732, 3.732, 1, 0]])

rlist = np.array([[0, 0, 0, 0, 0, 0],
                  [0, 1, 1, 1, 0, 0],
                  [1, 0, 0, 0, 0, 1],
                  [0, 2.732, 3.732, 2, 0, 0],
                  [2.732, 0, 0, 0, 0, 0],
                  [0, -2.732, -1, 0, 1, 0]])

# Define the joint angles
tlist = np.array([-1.57, 1.57, 1.0471, -0.785, 1, 0.5235])

# Calculate the forward kinematics in the spatial frame
T_spatial = mr.FKinSpace(m, slist, tlist)
print("Spatial Frame:\n", np.round(T_spatial, decimals=3))

# Calculate the forward kinematics in the body frame
T_body = mr.FKinBody(m, rlist, tlist)
print("\nBody Frame:\n", np.round(T_body, decimals=3))
