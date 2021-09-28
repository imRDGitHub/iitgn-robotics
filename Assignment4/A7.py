#@author: Videh Patel : videh.p@iitgn.ac.in: 19110192
#Cartesian Manipulator
#Diagram attached in PDF submitted

import numpy as np

from DHMatrix2Homo_and_Jacob import DHMatrix2Homo_and_Jacob as Dh2HnJ
#Joint Evariables
d1, d2, d3 = 2, 4, 9
d1_dot, d2_dot, d3_dot = 10,20,5

#Cartesian Configuration :: Refer to pdf for diagram
H = np.matrix([ [0, d1, 0, -np.pi/2], [-np.pi/2, d2, 0, -np.pi/2], [0, d3, 0, 0] ])

Homo, Jacob = Dh2HnJ(H, prismatic = [1,2,3]) 

print("Position for given orientation::")
print((Homo@np.matrix([0,0,0,1]).T)[:3,0])
print("Velocity for given orientation::")
print((Jacob@np.matrix([d1_dot,d2_dot, d3_dot]).T)[:3,0])
