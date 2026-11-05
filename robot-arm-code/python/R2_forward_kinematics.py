import numpy as np

def initialize_rnt_matrices(R1, phi1, phi2):

    rot2 = np.array([[1,0,0],
                    [0,np.cos(phi2),-np.sin(phi2)],
                    [0,np.sin(phi2),np.cos(phi2)]])
    
    trans1 = np.array([[1,0,0],
                        [0,1,0],
                        [R1,0,1]])
    
    rot1 = np.array([[1,0,0],
                        [0,np.cos(phi1),-np.sin(phi1)],
                        [0,np.sin(phi1),np.cos(phi1)]])

    return rot2, trans1, rot1


def forward_kinematics(R1, phi1, R2, phi2):

    rot2, trans1, rot1 = initialize_rnt_matrices(R1=R1, phi1=phi1, phi2=phi2)
    r = np.array([1,0,R2])
    r = rot2@r
    r=trans1@r
    r=rot1@r

    return r
    

