# @author Videh Patel : videh.p@iitgn.ac.in : 19110192

from A3 import DHMatrix2Homo_and_Jacob
import numpy as np

#Stanford Configuration :: Refer to pdf for diagram
H = np.matrix([ [-np.pi/4, 0, 0, 0], [np.pi/2, 5, 0, np.pi/2], [-np.pi/2, 0, 0, -np.pi/2] ])
ee_n = np.matrix([0, 0, 5+2.5]).T
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

# ------------------------------OUTPUT FROM A8.py of Assignment2(For this orientation)----------------------------
# PS C:\Users\videh\OneDrive\Documents\Sem 5\ME 639\Assignment2> py A8.py
# x0,y0,z0 are::
# [[5.30330086]] [[5.30330086]] [[5.]]
# -------------------------------OUTPUT FROM This Code ------------------------------------------------
# PS C:\Users\videh\OneDrive\Documents\Sem 5\ME 639\Assignment3> py A4_b.py
# Homogeneous Transformation Matrix ::
# [[ 8.65956056e-17 -7.07106781e-01  7.07106781e-01  0.00000000e+00]
#  [ 0.00000000e+00  7.07106781e-01  7.07106781e-01  0.00000000e+00]
#  [-1.00000000e+00 -6.12323400e-17  6.12323400e-17  5.00000000e+00]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# Jacobian::
# [[-5.30330086e+00 -6.49467042e-16  7.07106781e-01]
#  [ 5.30330086e+00 -4.93038066e-32  7.07106781e-01]
#  [ 0.00000000e+00  7.50000000e+00  6.12323400e-17]
#  [ 0.00000000e+00  7.07106781e-01  0.00000000e+00]
#  [ 0.00000000e+00 -7.07106781e-01  0.00000000e+00]
#  [ 1.00000000e+00  6.12323400e-17  0.00000000e+00]]

# Position for given orientation::
# [[5.30330086]
#  [5.30330086] ----------------------------->>>  Thus, the answers are same
#  [5.        ]]

# Linear Velocity::
# [[-22.98097039]
#  [ 30.0520382 ]
#  [ 37.5       ]]

# Angular Velocity::
# [[-3.53553391]
#  [ 5.        ]]

#  --------------------------------------------------------
# Thus, the answers from both codes are equal.