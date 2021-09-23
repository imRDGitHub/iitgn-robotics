# Name - Dev Patel
# Roll No - 18110113


import numpy as np

end_effector_position = list(map(float,input("Enter coordinate matrix and length d1 and d4 and link lengths(a1,a2) and alpha:").split()))
O = np.array(end_effector_position)

# Computes the joint variables for SCARA manipulator
def inverse_kinematics_SCARA():

    c2 = (O[0]**2 + O[1]**2 - O[5]**2 - O[6]**2)/(2*O[5]*O[6])
    theta2 = np.arctan2(-(1-c2**2)**0.5,c2)
    theta1 = np.arctan2(O[1],O[0]) - np.arctan2(O[6]*np.sin(theta2),O[5] + O[6]*np.cos(theta2))
    theta4 = theta1 +theta2 - np.arctan2(np.cos(O[7]),np.sin(O[7]))

    d3 = O[3] - O[2] - O[4]

    I = np.array([theta1,theta2,theta4,d3])
    return(I)

# gets the value of end effector position for the given joint variables
def check_end_Effector_position(I):
    T = np.eye(4)
    DH_parameters = np.array([[0,0,O[3],0],[I[0],O[5],0,0],[I[1],O[6],0,3.14],[0,0,I[3],0],[I[2],0,O[4],0]])
    for i in range(5):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
    return(print('x=',T[0,3],'y=',T[1,3],'z=',T[2,3]))

pos  = inverse_kinematics_SCARA()
print(pos)
check_end_Effector_position(pos)