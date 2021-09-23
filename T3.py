# Name - Dev Patel
# Roll No - 18110113


import numpy as np

links = int(input("Enter no of links:")) # User inputs for number of links

# Take values of DH parameters[theta, a, d, alpha] from user
DH_parameters = np.array([])
for i in range(links):
    if(i==0):
        DH_i = list(map(float, input("DH parameters for link {}:".format(i)).split()))
        DH_parameters = np.array(DH_i).reshape(1,4)
    else:
        DH_i = list(map(float, input("DH parameters for link {}:".format(i)).split()))
        DH_parameters = np.r_[DH_parameters,np.array(DH_i).reshape(1,4)]

print(DH_parameters)

# Get a skew symmetric matrix for a given 3x1 matrix
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

# Get the jacobian matrix for a robot
def Jacobian():
    T = np.eye(4)
    O = np.array([0,0,0])
    Z = np.array([0,0,1])
    for i in range(links):
        A = np.array([[np.cos(DH_parameters[i,0]), -np.sin(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), np.sin(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.cos(DH_parameters[i,0])],\
                [np.sin(DH_parameters[i,0]), np.cos(DH_parameters[i,0])*np.cos(DH_parameters[i,3]), -np.cos(DH_parameters[i,0])*np.sin(DH_parameters[i,3]),DH_parameters[i,1]*np.sin(DH_parameters[i,0])],\
                    [0,np.sin(DH_parameters[i,3]),np.cos(DH_parameters[i,3]),DH_parameters[i,2]],\
                        [0,0,0,1]])

        T = np.dot(T,A)
        O = np.c_[O,np.transpose(T[:3,3])]
        Z = np.c_[Z,np.transpose(T[:3,2])]
    print(O)
    print(Z)
    Ans = input("Do you want to add prismatic joints? (y/n): ")
    if(Ans=="n"):
        J = np.r_[np.dot(skew(Z[:,0]),np.transpose(O[:,links]) - np.transpose(O[:,0])), np.transpose(Z[:,0])]
        for i in range(1,links):
            J = np.c_[J,np.r_[np.dot(skew(Z[:,i]),np.transpose(O[:,links]) - np.transpose(O[:,i])), np.transpose(Z[:,i])]]
    else:
        Prismatic = list(map(float, input("Mention the prismatic joints: ").split()))
        P = np.array(Prismatic)
        J = np.r_[np.dot(skew(Z[:,0]),np.transpose(O[:,links]) - np.transpose(O[:,0])), np.transpose(Z[:,0])]
        for i in range(1,links):
            if((i in P) == True):
                J = np.c_[J,np.r_[np.transpose(Z[:,i]),np.transpose([0,0,0])]]
            else:
                J = np.c_[J,np.r_[np.dot(skew(Z[:,i]),np.transpose(O[:,links]) - np.transpose(O[:,i])), np.transpose(Z[:,i])]]
    return(J)

# Getting the joint velocity from the end - effector velocity
# # # Note that JxJ(transpose) must be invertible for this code to work.
def joint_velocity():
    J = Jacobian()
    x_dot = list(map(float, input("Velocity matrix of end effector: ").split()))
    b = list(map(float, input("b matrix(nxn): ").split()))
    J_plus = np.dot(np.transpose(J),np.linalg.inv(np.dot(J,np.transpose(J))))
    q_dot = np.dot(J_plus,x_dot) + np.dot((np.eye(links) - np.dot(J_plus,J),b))

    return(q_dot)

# Running joint velocity subroutine
joint_velocity() 
