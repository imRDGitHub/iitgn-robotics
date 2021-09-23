# Name - Dev Patel
# Roll No - 18110113


import numpy as np

end_effector_position = list(map(float,input("Enter the desired orientation matrix for the spherical wrist:").split()))
O = np.array(end_effector_position).reshape(3,3)

# Computes the wrist angles for the desired orientation 
def inverse_kinematics_wrist():

    
    theta1 = np.arctan2(O[1,2],O[0,2])
    theta2 = np.arctan2((1-O[2,2]**2)**0.5)
    theta3 = np.arctan2(O[2,1] - O[2,0])

    d3 = O[3] - O[2] - O[4]

    I = np.array([theta1,theta2,theta3])
    return(I)

pos  = inverse_kinematics_wrist()
print(pos)