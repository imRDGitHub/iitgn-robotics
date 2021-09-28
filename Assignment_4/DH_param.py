import numpy as np

class manipulator():
    def __init__(self,dh_params, config=['R','R']):
        # configuration of the manipulator. User has 2 choices "R"->revolute. "P"->prismatic.
        # Default configuration is a 2R manipulator with all the angles at 0 degrees and lengths being 1 unit.
        self.config = config 
        # User must input the dh parameters in matrix form i.e. "R"->revolute
        # [[a1 , alpha1 , d1, theta1]
        #  [a2 , alpha2 , d2, theta2]
        #  .
        #  .
        #  .
        #  [an , alphan , dn, thetan]]
        # n being the nth link of the manipulator.
        self.dh=dh_params
    
    def calc_tranfMatrix(self, dh_params,i):
        # Calculating Trnasformation matrix
        a, alpha,d,theta = dh_params
        A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                      [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                      [            0,                np.sin(alpha),                np.cos(alpha),               d],
                      [            0,                            0,                            0,               1]])
        return A 

    def forward_kinematics(self,):
        tr=self.calc_tranfMatrix(self.dh[0],0)
        Trs = [tr]
        # Calculating the individual transformation matrices. And appending to the T matrix in the following form.
        # A1
        # A1A2
        # A1A2A3 ... 
        for i in range(len(self.dh)-1):
            tr = np.matmul(tr,self.calc_tranfMatrix(self.dh[i+1],i+1))
            Trs.append(tr)

        # Calculating the jacobian matrix
        h = []
        for i in range(len(self.config)):
            temp2 = np.array([0,0,0])
            if self.config[i]=='R':
                temp2 = np.array([0,0,1])
            if  i ==0:
                temp = np.array(Trs[-1])

            else:
                temp = np.array(Trs[-1]) - np.array(Trs[i-1])
            
            h.append(np.cross(temp2,temp[:3,3:].transpose()).transpose())
        
        # Velocity jacobian
        J_v = h[0]
        for i in range(len(self.config)-1):
            J_v = np.hstack((J_v,h[i+1]))
        
        
        # Angular velocity jacobian
        J_omega = np.array([[0],[0],[1]])
        if self.config[0]=='P':
            J_omega = np.array([[0],[0],[0]])

        for i in range(len(self.config)-1):
            temp = np.array([[0],[0],[1]])

            if self.config[i+1]=='P':
                temp = np.array([[0],[0],[0]])
            J_omega = np.hstack((J_omega,temp))
        
        # Overall Transformation matrix T06.
        transformation_matrix = np.array(Trs[-1])
        # Manipulator jacobian
        J = np.vstack((J_v,J_omega))

        return {'transformation_matrix':transformation_matrix,'jacobian':J}
        