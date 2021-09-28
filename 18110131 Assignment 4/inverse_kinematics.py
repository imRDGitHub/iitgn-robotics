#To solve inverse kinematics equations. This is direct numerical method solving. SLow processing

#imports
import numpy as np
import sympy as sym
from DH_jacobian_calculator import End_Position

def inverse_kinematics(X,P,var_DH_param,Q):
    #Works for 3dof or less systems
    P0=End_Position(var_DH_param,P)

    eq=[]
    Q=np.array(Q).reshape(-1,).tolist()#Symbolic

    for i in range(3):
        eq.append(sym.Eq(P0[i,0],X[i,0]))

    Q_solve = sym.solve(eq,Q,simplify=False,set=True)
    return Q_solve


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

    P0= End_Position(DH_param,P)
    
    print(P0)

    #Now going for inverse
    q1 = sym.Symbol('q1',real = True)
    q2 = sym.Symbol('q2',real = True)

    DH_param=np.zeros([linkCount,5],dtype=object)# First column link types, second for a, third for alpha , fourth for d and fifth for theta
    DH_param[0,0]='R'
    DH_param[1,0]='R'
    DH_param[0:1,1:5]=np.array([1.5,0,0,q1])
    DH_param[1:2,1:5]=np.array([1.5,0,0,q2])
    Q=np.array([[q1],[q2]])
    inverse_kinematics(P0,P,DH_param,Q)
    

