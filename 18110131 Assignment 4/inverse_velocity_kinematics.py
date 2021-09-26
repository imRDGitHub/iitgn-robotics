#To calculate inverse velocities from Jacobian of dh parameterized manipulator

#imports
import numpy as np
from DH_jacobian_calculator import Jacobian_EndPoint_DH

def inverse_velocity(DH_param,X,X_dot):
    J0_P,P0=Jacobian_EndPoint_DH(DH_param,X)
    
    inv_J0_P=np.linalg.pinv(J0_P)

    Q_dot=inv_J0_P@X_dot

    return Q_dot


if __name__=="__main__":

    #Test For above code.
    linkCount=2
    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[1,0]='R'
    DH_param[0:1,1:5]=np.array([1.5,0,0,np.pi/4])
    DH_param[1:2,1:5]=np.array([1.5,0,0,np.pi/4])
    #q=pi/4,pi/4
    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(DH_param,P)
    print(J0_P)
    print(P0)

    q_dot=np.array([[1],[1]]) #each 1 rads per second

    #Velocity calculations
    v=J0_P[0:3,:]@q_dot
    w=J0_P[3:6,:]@q_dot

    # print(v)
    # print(w)
    X_dot=J0_P@q_dot
    print(X_dot)
    q_dot_new=inverse_velocity(DH_param,P,X_dot)
    print(q_dot_new)

