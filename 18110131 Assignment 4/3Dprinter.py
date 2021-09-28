#3D printer case is similar to cartesian in last assignment

#imports
import numpy as np
from DH_jacobian_calculator import Jacobian_EndPoint_DH


def Printer_DH(Q,Q_dot):
    #Difference is that all axis positive here
    d1=Q[0,0]
    d2=Q[1,0]
    d3=Q[2,0]
    linkCount=3

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='P'
    DH_param[0:1,1:5]=np.array([0,np.pi/2   ,d1 ,np.pi/2])
    DH_param[1,0]='P'
    DH_param[1:2,1:5]=np.array([0,-np.pi/2  ,d2   ,-np.pi/2])
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0         ,d3   ,0]) 
    DH_param
    P=np.array([[0],[0],[0],[1]])

    J0_P,P0=Jacobian_EndPoint_DH(DH_param,P)
    print("Jacobian :")
    print(np.round_(J0_P,decimals=5))
    print("End Position :")
    print(P0)

    print("Velocity matrix: ")
    print(J0_P@Q_dot)
    #Inverse Kinematics, based on geometry
    
    d1=P0[2,0]
    d2=P0[0,0]
    d3=P0[1,0]

    print([d1,d2,d3])



if __name__=="__main__":
    Q1=np.array([[1],[1.5],[0.5]]) #Z then X then Y
    Q1_dot=np.array([[1],[0.5],[1.5]])
    Printer_DH(Q1,Q1_dot)
