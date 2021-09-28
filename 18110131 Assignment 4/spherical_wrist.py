#Spherical wrist

#imports 
import numpy as np



def EulerAnglesCalc( R3_6):
    #Using the last column elements of zr3_6 column(rotation of wrist) to solve:
    #R3_6=(R0_3)^{-1} R
    if(R3_6[0,2]==0 and R3_6[1,2]==0):
        #Page 95 of second edition
        if(R3_6[2,2]>0):
            q5=0
        else:
            q5=np.pi
        
        sum_q4_q6=np.arctan2(R3_6[1,0],R3_6[0,0])
        #Assuming some q4
        q4=np.pi/4
        q6=sum_q4_q6-q4

    else:
        q5=np.arctan2(R3_6[2,2],np.sqrt(1-R3_6[2,2]**2))

        q4=np.arctan2(R3_6[1,2],R3_6[0,2])
        q6=np.arctan2( R3_6[2,1],-R3_6[2,0] ) 
    
    return [q4,q5,q6]


if __name__=="__main__":
    #q4=30 q5=45 q6=60
    R3_6=np.array([ [-0.127  ,-0.780 ,0.612    ],
                    [0.926   ,0.127  ,0.353  ],
                    [-0.353  ,0.612  ,0.707  ]])
    
    print(EulerAnglesCalc(R3_6))

    
    R3_6=np.array([ [1  ,0 ,0  ],
                    [0  ,1 ,0  ],
                    [0  ,0 ,1  ]])
    
    print(EulerAnglesCalc(R3_6))

    