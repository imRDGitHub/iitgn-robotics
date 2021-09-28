#For SCARA

#imports
import numpy as np
import sympy as sym
from DH_jacobian_calculator import End_Position
from inverse_kinematics import inverse_kinematics





def SCARA_var_DH_param(l1,l2):
    q1 = sym.Symbol('q1',real = True)
    q2 = sym.Symbol('q2',real = True)
    d3 = sym.Symbol('d3',real = True)
    
    Q=np.array([[q1],[q2],[d3]])
    linkCount=3

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([l1,0,0,q1])
    DH_param[1,0]='R'
    DH_param[1:2,1:5]=np.array([l2,np.pi,0,q2])
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0,d3,0]) 

    return [DH_param,Q]


if __name__=="__main__":
    q1=np.pi/6
    q2=np.pi/3
    d3=0.5

    l1=5
    l2=3

    #Joint variables
    Q=np.array([[q1],[q2],[d3]])
    print(Q)
    q1=Q[0,0]
    q2=Q[1,0]
    d3=Q[2,0]
    linkCount=3

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[0:1,1:5]=np.array([l1,0,0,q1])
    DH_param[1,0]='R'
    DH_param[1:2,1:5]=np.array([l2,np.pi,0,q2])
    DH_param[2,0]='P'
    DH_param[2:3,1:5]=np.array([0,0,d3,0]) 

    P=np.array([[0],[0],[0],[1]])

    P0=End_Position(DH_param,P)
    print(P0)

    #Geometric Method:
    x=float(P0[0,0])
    y=float(P0[1,0])

    q2=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    q1=np.arctan2(y,x)-np.arctan2((l2*np.sin(q2)),(l1+l2*np.cos(q2)))
    d3=0-P0[2,0]

    print("Geometric calculated values: ")
    print([q1,q2,d3])



    #direct inverse kinemtaics using python solvers and method defined in inverse_kinematics.py
    # print("Wait for sometime as this method takes time:\n")
    # DH_param,Q=SCARA_var_DH_param(l1,l2)
    # Q_solve=inverse_kinematics(P0,P,DH_param,Q)

    # print(Q_solve)