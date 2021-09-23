# @author Videh Patel : videh.p@iitgn.ac.in : 19110192

from A3 import DHMatrix2Homo_and_Jacob
import numpy as np

#SCARA Configuration :: Refer to pdf for diagram
H = np.matrix([ [-np.pi/4, 5, 0, 0], [np.pi/2, 0, 5, 0], [np.pi/4, 0, 5, 0] ])
ee_n = np.matrix([0, 0, -2.5]).T
q_dot = np.matrix([5, 5, 5]).T #rad/s, rad/s, m/s

Homo, Jacob = DHMatrix2Homo_and_Jacob(H,ee_n, prismatic = [3]) 

position = (Homo@np.concatenate((ee_n, np.matrix([[1]]))))[:3,0]
velocity_mat = Jacob@q_dot
linear_velo = velocity_mat[:3,0]
angular_velo = velocity_mat[4:,0]

print('Homogeneous Transformation Matrix ::')
print(Homo)
print()
print("Jacobian::")
print(Jacob)
print()
print("Position for given orientation::")
print(position)
print()
print("Linear Velocity::")
print(linear_velo)
print()
print("Angular Velocity::")
print(angular_velo)
print()



# ------------------------------OUTPUT FROM A7.py of Assignment2(For this orientation)----------------------------
# PS C:\Users\videh\OneDrive\Documents\Sem 5\ME 639\Assignment2> py A7.py
# x0,y0,z0 are::
# [[3.53553391]] [[8.53553391]] [[2.5]]
# -------------------------------OUTPUT FROM This Code ------------------------------------------------
# Homogeneous Transformation Matrix ::
# [[ 4.26642159e-17 -1.00000000e+00  0.00000000e+00  3.53553391e+00]
#  [ 1.00000000e+00 -4.26642159e-17  0.00000000e+00  8.53553391e+00]
#  [ 0.00000000e+00  0.00000000e+00  1.00000000e+00  5.00000000e+00]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# Jacobian::
# [[-8.53553391 -5.          0.        ]
#  [ 3.53553391  0.          0.        ]
#  [ 0.          0.          1.        ]
#  [ 0.          0.          0.        ]
#  [ 0.          0.          0.        ]
#  [ 1.          1.          0.        ]]

# Position for given orientation::
# [[3.53553391]
#  [8.53553391]               ----------------------->>>>>>>> Thus, the answers are same
#  [2.5       ]]

# Linear Velocity::
# [[-67.67766953]
#  [ 17.67766953]
#  [  5.        ]]

# Angular Velocity::
# [[ 0.]
#  [10.]]

#  --------------------------------------------------------
# Thus, the answers from both codes are equal.