# Name - Dev Patel
# Roll No - 18110113


import numpy as np

end_effector_position = list(map(float,input("Enter coordinate matrix and d1,d2): ").split()))
O = np.array(end_effector_position)

# Computes the joint variables for stanford type manipulator
def inverse_kinematics_Stanford():

    r = (O[0]**2 + O[1]**2)**0.5
    theta1 = np.arctan2(O[1],O[0]) - np.arctan2(O[4],(r**2 - O[4]**2)**0.5)
    theta2 = np.arctan2(O[0]*np.cos(theta1)+ O[1]*np.sin(theta1),O[2]-O[3])
    d3 = np.sin(theta2)*(O[0]*np.cos(theta1) + O[1]*np.sin(theta1)) + np.cos(theta2)*(O[2] - O[3])

    I = np.array([theta1,theta2,d3])
    return(I)

# gets the value of end effector position for the given joint variables
def check_end_Effector_position(I):
    T = np.eye(4)
    DH_parameters = np.array([[I[0],0,O[3],-1.57],[I[1],0,O[4],-1.57],[0,0,I[2],0]])
    for i in range(3):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
    return(print('x=',T[0,3],'y=',T[1,3],'z=',T[2,3]))

pos  = inverse_kinematics_Stanford()
print(pos)
check_end_Effector_position(pos)