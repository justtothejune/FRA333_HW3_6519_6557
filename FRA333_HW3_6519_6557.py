# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ณพวิทย์ 6519
2. อนัญญา 6557
3.
'''

# import library
import numpy as np
from HW3_utils import FKHW3
from math import pi
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R,P,R_e,p_e = FKHW3(q)
    # print("Rotation Matrices (R):\n", R)
    # print("Position Vectors (P):\n", P)
    # print("End-Effector Rotation Matrix (R_e):\n", R_e)
    # print("End-Effector Position (p_e):\n", p_e)

    njoints = len(q)
    J_e = np.empty((6,njoints))


    #loop for all joints
    for i in range(njoints) :
        p_i = P[:, i]  # Position vector of joint i
        z_i = R[:, 2, i] # Z-axis (rotation axis) of joint i (column 2 of R matrix)

        # Linear velocity part (translation contribution)
        J_e[0:3, i] = np.cross(z_i, (p_e - p_i))
        
        # Angular velocity part (rotation contribution)
        J_e[3:6, i] = z_i
    
    return J_e.round(6) #round a decimals number

q = [0, 0, 0]
# Call the function to compute the Jacobian
jacobian_matrix = endEffectorJacobianHW3(q)

# Print the resulting Jacobian matrix
# print("Jacobian Matrix:\n", jacobian_matrix)

#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    # get a jacobian matriox from above
    J_e = endEffectorJacobianHW3(q)
    submatrix = J_e[0:3,:] # rows 0-2 and all columns
    # find a determinant of matrix J_e
    deter = np.linalg.det(submatrix)

    # if deter is < 0.001 mean it is singularity
    if (abs(deter) < 0.001) :
        return True
    else : 
        return False

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    # get a jacobian matriox from above
    J_e = endEffectorJacobianHW3(q)
    # transpose a matrix 
    Jacob_T = np.transpose(J_e)
    w = np.array(w).reshape(6, 1)
    #find tau dot matrix between transpose matrix and wrench
    tau = np.dot(Jacob_T,w)
    return tau.flatten()

q = [0,0,0]
w = [0,0.2,0.5,4,6,1]
# print(f"Joint torques (tau) : {computeEffortHW3(q,w)}")
#==============================================================================================================#