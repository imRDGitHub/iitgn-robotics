import numpy as np

#System Variables
d = 10 #Resting position of prismatic joint (m)
l1 = 10 #Link1 length (m)
l2 = 10 #Link2 length (m)
l3 = 10 #Link3 length (m)

#Joint Variables
x = 5 #Extension of primatic joint (m)
q1 = np.pi/2 #Relative angle of link1 and link2 (rad)
q2 = -np.pi/2 #Relative angle of link2 and link3 (rad)

p3 = np.matrix([[0],[0], [-(x+d)], [1]])
H32 = np.matrix([[1, 0, 0, 0], [0, 1, 0, l3], [0, 0, 1, 0], [0, 0, 0, 1]])
H21 = np.matrix([[np.cos(q2), -np.sin(q2), 0, 0],[np.sin(q2), np.cos(q2), 0, l2], [0, 0, 1, 0], [0, 0, 0, 1]])
H10 = np.matrix([[np.cos(q1), -np.sin(q1), 0, 0],[np.sin(q1), np.cos(q1), 0, 0], [0, 0, 1, l1], [0, 0, 0, 1]])

p0 = (H10@H21@H32@p3)
x0,y0,z0,*_ = p0

print('x0,y0,z0 are::')
print(x0,y0,z0)
