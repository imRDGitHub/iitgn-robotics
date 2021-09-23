import numpy as np

#System Variables
d = 10 #Resting position of prismatic joint (m)
l1 = 10 #Link1 length (m)

#Joint Variables
x = 5 #Extension of primatic joint (m)
q1 = np.pi/4 #Relative angle of link1 and link2 (rad)
q2 = np.pi/4 #Relative angle of link2 and link3 (rad)

p2 = np.matrix([[0],[(x+d)], [0], [1]])
H21 = np.matrix([[1, 0, 0, 0],[0, np.cos(q2), -np.sin(q2), 0], [0, np.sin(q2), np.cos(q2), l1], [0, 0, 0, 1]])
H10 = np.matrix([[np.cos(q1), -np.sin(q1), 0, 0],[np.sin(q1), np.cos(q1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

p0 = (H10@H21@p2)

print(p2)
print(H21)
print(H10)
print(p0)

x0,y0,z0,*_ = p0

print('x0,y0,z0 are::')
print(x0,y0,z0)
