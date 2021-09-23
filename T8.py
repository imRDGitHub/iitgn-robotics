# Name - Dev Patel
# Roll No - 18110113


import numpy as np

end_effector_position = list(map(float,input("Enter cordinate matrix: ").split()))
O = np.array(end_effector_position)

def inverse_kinematics_3Dprinter():

    d1 = O[2]
    d2 = O[1]
    d3 = O[0]

    I = np.array([d1,d2,d3])
    return(I)

# gets the value of end effector position for the given joint variables
def check_end_Effector_position(I):
    T = np.eye(4)
    DH_parameters = np.array([[0,0,I[0],-1.57],[-1.57,0,I[1],1.57],[0,0,I[2],0]])
    for i in range(3):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
    return(print('x=',T[0,3],'y=',T[1,3],'z=',T[2,3]))

position  = inverse_kinematics_3Dprinter()
print(position)
check_end_Effector_position(position)