import numpy as np

#System Variables
l1 = 10 #Link1 length (m)
l2 = 10 #Link2 length (m)
l3 = 10 #Link3 length (m)

#Joint Variables
q1 = np.pi/4 #Relative angle of link1 and world frame (rad)
q2 = -np.pi/2 #Relative angle of link2 and link1 (rad)
q3 = -np.pi/4 #Relative angle of link3 and link2 (rad)

#Joint motions
q1_dot = 2 #(rad/s)
q2_dot = 3 #(rad/s)
q3_dot = 3 #(rad/s)

J= np.matrix([[-(l1*np.sin(q1) + l2*np.sin(q1+q2) + l3*np.sin(q1+q2+q3)), -(l2*np.sin(q1+q2) + l3*np.sin(q1+q2+q3)), -(l3*np.sin(q1+q2+q3))],\
    [l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3), l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3), l3*np.cos(q1+q2+q3)],\
    [0, 0, 0],\
    [0, 0, 0],\
    [0, 0, 0],\
    [1, 1, 1]])

print("The Jacobian for RRR Manipulator is:: ")
print(J)

VW= J@np.matrix([[q1_dot], [q2_dot], [q3_dot]])

print('Velocity of EE is:: ')
print(VW[[0,1,2],[0,0,0]])

print('Angular Velocity of EE is:: ')
print(VW[[3,4,5],[0,0,0]])
