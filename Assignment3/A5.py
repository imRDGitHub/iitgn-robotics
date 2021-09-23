# @author Videh Patel : videh.p@iitgn.ac.in : 19110192

#To verify the hand derived answer in Q5 in pdf
from A3 import DHMatrix2Homo_and_Jacob
import numpy as np

#Same values as in pdf
d0 = 1
d1 = 2
d2 = 4
d3 = 9

#Cartesian Configuration :: Refer to pdf for diagram
H = np.matrix([ [0, d0, 0, 0], [0, d1, 0, -np.pi/2], [-np.pi/2, d2, 0, -np.pi/2] ])
ee_n = np.matrix([0, 0, d3]).T

Homo, Jacob = DHMatrix2Homo_and_Jacob(H,ee_n, prismatic = [1,2,3]) 

position = (Homo@np.concatenate((ee_n, np.matrix([[1]]))))[:3,0]

print("Position for given orientation::")
print(position)

# -------------------------------OUTPUT FROM This Code ------------------------------------------------
# PS C:\Users\videh\OneDrive\Documents\Sem 5\ME 639\Assignment3> py A5.py
# Position for given orientation::
# [[9.]
#  [4.]
#  [3.]]
#  --------------------------------------------------------
# The answers are same as the ones derived in the pdf.
# Hence verified!