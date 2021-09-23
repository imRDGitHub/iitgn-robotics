import numpy as np

#System Variables
d = 10 #Resting position of prismatic joint (m)
l1 = 10 #Link1 length (m)
l2 = 10 #Link2 length (m)
l3 = 10 #Link3 length (m)

#Joint Variables
x = 5 #Extension of primatic joint (m)
q1 = np.pi/4 #Relative angle of link1 and link2 (rad)
q2 = -np.pi/2 #Relative angle of link2 and link3 (rad)

#Joint motions
x_dot = 2 #(m/s)
q1_dot = 2 #(rad/s)
q2_dot = 3 #(rad/s)

J= np.matrix([[-l2*np.cos(q1)-l3*np.cos(q1+q2),-l3*np.cos(q1+q2),0],[l2*np.sin(q1)+l3*np.sin(q1+q2),l3*np.sin(q1+q2),0],[0,0,-1],[0,0,0],[0,0,0],[1,1,0]])

print("The Jacobian for SCARA is:: ")
print(J)

VW= J@np.matrix([[q1_dot], [q2_dot], [x_dot]])

print('Velocity of EE is:: ')
print(VW[[0,1,2],[0,0,0]])

print('Angular Velocity of EE is:: ')
print(VW[[3,4,5],[0,0,0]])
