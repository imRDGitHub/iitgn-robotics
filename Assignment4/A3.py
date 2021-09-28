#@author: Videh Patel : videh.p@iitgn.ac.in: 19110192
#Converts EEvelo[vector] to Joint_velo[vector] by multiplying psuedoinverse of Jacobian  
#Explanation attached in PDF submitted

import numpy as np

def EEvelo2JointVelo(J, EE_velo):
    #J: Jacobian [6xn np.matrix]
    #EE_velo : End Effector Velocity Matrix [linear + angular] [6x1 np.matrix]
    #returns q_dot : Joint velociy array [nx1 np.matrix]
    return np.linalg.pinv(J)@EE_velo

if __name__ == '__main__':
    #Verify for 2R Manipulator
    from DHMatrix2Homo_and_Jacob import DHMatrix2Homo_and_Jacob as DH2HnJ #To calculate Jacobian

    Dh = np.matrix([[0, 0, 5, 0], [0, 0, 5, 0]])
    H, J = DH2HnJ(Dh)

    check_velo = np.matrix([0,20,0,0,0,2]).T

    print(EEvelo2JointVelo(J, check_velo)) #Should return [2,0].T 
