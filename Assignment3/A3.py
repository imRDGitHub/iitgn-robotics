import numpy as np
# @author Videh Patel :: videh.p@iitgn.ac.in :: 19110192

# DH Parameter Matrix [nx4 Matrix]
# Number of links automatically calculated from the size of matrix
# theta (rotation about z), d (translation about z), a(translation about x), alpha(rotation about x)
# First columns of params are frame 0-->1 and last columns are frame n-->(n+1) :: (n+1) is the end effector

def DHMatrix2Homo_and_Jacob(Hmat, EE_n, prismatic=[]):
    #Hmat is the DH parameter matrix
    #prismatic is an array of the joints that are prismatic: Joint corresponding to qi is ith joint
    #All revolute and prismatic axes are assumed to be aligned with the respective z axis

    def d_(k):
        #Returns position vector of ee wrt k-frame in o-frame basis
        return (T[k-1][:3,:3])@(np.linalg.inv(T[k-1])@T[n-1]@np.concatenate(((EE_n), np.matrix([[1]]))))[:3,0]
        
    n = len(Hmat)
    T = []  #Array of all absolute Homogeneous Transformations
    J = []  #Manipulator Jacobian
    for params in Hmat:
        theta, d, a, alpha = np.array(params)[0]
        Ti = np.matrix([ [ np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],\
                         [ np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],\
                         [ 0            , np.sin(alpha)               , np.cos(alpha)              , d              ],\
                         [  0            , 0                           , 0                          ,1              ] ])
        
        if len(T) != 0:
            T.append(T[-1]@Ti)
        else:
            T.append(Ti)

    for i in range(1,n+1):
        if i in prismatic:
            Jv = T[i-1][:3,:3]@np.matrix([0,0,1]).T
            Jw = np.matrix([0,0,0]).T
        else: #Revolute
            Jw = T[i-1][:3,:3]@np.matrix([0,0,1]).T
            Jv = np.matrix(np.cross(Jw.T,d_(i).T).T)
        
        Ji = np.concatenate((Jv, Jw))
        
        if len(J) == 0:
            J = Ji
        else:
            J = np.concatenate((J,Ji), axis = 1)

        H = T[n-1]

    return H, J


if __name__ == "__main__":
    H = np.matrix([ [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0] ])
    ee_n = np.matrix([10, 0, 0]).T
    q_dot = np.matrix([5, 5, 5, 5]).T #rad/s, rad/s, rad/s, m/s



    Homo, Jacob = DHMatrix2Homo_and_Jacob(H,ee_n,[4]) 
    position = (Homo@np.concatenate((ee_n, np.matrix([[1]]))))[:3,0]
    velocity_mat = Jacob@q_dot
    linear_velo = velocity_mat[:3,0]
    angular_velo = velocity_mat[4:,0]

    print('Homogeneous Transformation Matrix ::')
    print(Homo)
    print()
    print("Jacobian::")
    print(Jacob)
    print()
    print("Position for given orientation::")
    print(position)
    print()
    print("Linear Velocity::")
    print(linear_velo)
    print()
    print("Angular Velocity::")
    print(angular_velo)
    print()