# FRA333_HW3_6519_6557
## Done by
- 65340500019 ณพวิทย์ อินทร์จันทร์
- 65340500057 อนัญญา อุจจธรรมรัตน์
## File
- Answer file : FRA333_HW3_6519_6557.py
- Prove file  : test_scriptHW3.ipynb

## Objective :
การบ้านนี้ถูกออกแบบขึ้นมาเพื่อให้ผู้เรียนได้ประยุกต์ใช้องค์ความรู้การหาจลนศาสตร์เชิงอนุพันธ์ (Differential kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator)

## Find DH Parameter of a robot
![IMG_1908.jpg](IMG_1908.jpg)

### Import Library
```ruby
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi

from HW3_utils import *
from FRA333_HW3_6519_6557 import *
```

### Robot DH Parameter
```ruby
#Define link length
d1 = 0.0892
a2 = -0.425

T_3_e = SE3(-0.39243,-0.093,0.109)*SE3(- 0.082,0,0)*SE3.RPY(0,-pi/2,0)
# Find MDH Parameter using Robotics toolbox
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d = d1, offset=pi),
        rtb.RevoluteMDH(alpha=pi/2),
        rtb.RevoluteMDH(a= a2)
    ],
    tool = T_3_e,
    name = "RobotHW3"
    )
print(robot)
```
## Question 1 
จงเขียนฟังก์ชั่นในการ Jacobian ของหุ่นยนต์ตัวนี้ให้อยู่ในฟังก์ชั่นต่อไปนี้
J_e = endEffectorJacobianHW3(q)

โดยวิธีการหา Jacobian หาได้จาก
Geometric Jacobian 
### Solution
```ruby
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
```
### Validation
สามารถพิสูจน์ได้จากการใช้ MDH Parameter ที่หามาจาก Robotics toolbox แล้วนำมาหา Jacobian โดยใช้วิธีใ robot.jacob0()
- ทำการ random ค่า q
```ruby
# Random q
q = np.random.rand(3)*np.pi

# take jacobian fron the matrix above
J = robot.jacob0(q)

# Print it and compare with Jacobian matrix that you calculate 
J_e = endEffectorJacobianHW3(q)
print(f"Jacobian from calculate\n {J_e.round(6)}")
print("-----------------------------------")
print(f"Jacobian from prove\n {J.round(6)}")
```
### ตัวอย่างของคำตอบ
```
Random q is : [2.67977573 2.85832378 0.73394284]
Jacobian from calculate
 [[ 0.451642 -0.153604 -0.259946]
 [ 0.662806  0.076451  0.129379]
 [-0.        0.794614  0.386552]
 [ 0.       -0.445575 -0.445575]
 [ 0.       -0.895244 -0.895244]
 [ 1.        0.        0.      ]]
-----------------------------------
Jacobian from prove
 [[ 0.451642 -0.153604 -0.259946]
 [ 0.662806  0.076451  0.129379]
 [ 0.        0.794614  0.386552]
 [-0.       -0.445575 -0.445575]
 [-0.       -0.895244 -0.895244]
 [ 1.        0.        0.      ]]
```
- โดยทำการเปรียบเทียบระหว่าง Jacobian from calculate เทียบกับ Jacobian from prove (Robotics toolbox)
- ในส่วนของ Jacobian Matrix สามารถหาได้จาก การสร้าง Jacobian Matrix ขนาด 6x3 โดยที่ 3x3 ด้านบนแทน Linear Velocity และอีก 3x3 ที่เหลือเป็นส่วนของ Angular Velocity โดยจะอยู่ในรูปแบบของการคำนวณแบบ Revolute Joint
- อ้างอิงหลักจากหุ่นยนต์ เนื่องจากเป็น 3-DOF Manipulator (RRR Robot)

## Question 2
ตรวจสอบว่าเกิดสภาวะ Singularity หรือไม่
### Solution
```ruby
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
```
- จาก Solution ทำการนำ Matrix Jacobian มาลดรูปเหลือเป็นแค่ Matrix 3x3 ที่เป็นส่วนของ Linear Velocity และนำมาทำการหา Determinant เปรียบเทียบตามโจทย์ที่กำหนดให้
- ถ้า det < 0.001 ถ้า return เป็น True แสดงก็ต่อเมื่ออยู่ตำแหน่งใกล้สภาวะ Singularity return False คือแขนกลอยู่ในสภาวะปกติ
### Validation
```ruby
# Random Q
q = np.random.rand(3)*np.pi
print(f"Random q : {q}")
# Check singularity from robotics toolbox
J = robot.jacob0(q)
# Get only 3x3 matrix determinant used only squared matrix
submatrix_J = J[0:3,:]
# Find determinant
determinant = np.linalg.det(submatrix_J)
prove_singular = abs(determinant) < 0.001
print(f"Determinant for proving : {determinant}")
print(f"ผลลัพธ์ที่ได้จาก Robotics toolbox : {prove_singular}")


#Check from calculate
prove_calculate = checkSingularityHW3(q)
print(f"ผลลัพธ์ที่ได้จากที่คำนวณ : {bool(prove_calculate)}")

#Compare prove_singular and prove_original is it the same
if (prove_calculate == prove_singular):
    print(f"ผลลัพธ์ทั้งสองมีค่าตรงกัน : {True}")
else :
    print (False)
```
- random ค่า q และทำการเปรียบเทียบว่าผลลัพธ์ที่ได้มีค่าตรงกันและเป็น Singularity หรือไม่
### ตัวอย่างของคำตอบ
```
Random q : [0.0365338  1.99253007 1.29476752]
Determinant for proving : -0.1289799454870287
ผลลัพธ์ที่ได้จาก Robotics toolbox : False
ผลลัพธ์ที่ได้จากที่คำนวณ : False
ผลลัพธ์ทั้งสองมีค่าตรงกัน : True
```

## Question 3
จงเขียนฟังก์ชั่นในการหาeffortของแต่ละข้อต่อเมื่อมี wrench มากระทำกับจุดกึ่งกลางของเฟรมพิกัด $F^e$
โดยที่ Function ComputeEffortHW3 ใช้ในการคำนวณแรงบิดที่เกิดขึ้นกับ Joint ต่างๆของหุ่นยนต์ โดยอาศัยค่า Force และ Moment ที่ทำกับ end-effector ให้อยู่ในระบบพิกัดฐาน
สามารถหาค่า Torque จากการแปลง Jacobian Matrix Transpose x wrench
### Solution
```ruby
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    # get a jacobian matrix from above
    J_e = endEffectorJacobianHW3(q)
    # transpose a matrix 
    Jacob_T = np.transpose(J_e)
    w = np.array(w).reshape(6, 1)
    #find tau dot matrix between transpose matrix and wrench
    tau = np.dot(Jacob_T,w)
    return tau.flatten()
```
- Transpose matrix จาก solution ใน question 1 และกำหนด Array ให้กับ Wrench มีขนาด 6x1
- นำทั้งสองค่ามาทำการ dot matrix กันจากสูตร $T = J^T * w$

### Validation
```ruby
def proofeffort(q:list[float], w:list[float])->list[float] :
    
    # Get a tau from function computeEffortHW3(q,w)
    calculate_tau = computeEffortHW3(q,w)
    print(f"Calculate effort is {calculate_tau.round(6)}")

    # do a jacobian matrix using robotics toolbox
    J = robot.jacob0(q)
    subJ_T = np.transpose(J)
    tau_rtb = np.dot(subJ_T,w)
    print(f"RTB effort is {tau_rtb.flatten().round(6)}")
    # Compare 2 answer is it the same
    if np.allclose(tau_rtb,calculate_tau):
        return "The answer is the same"
    else :
        return "Wrong!"

# Random q and w
q = np.random.rand(3)*np.pi
w = np.random.uniform(0,np.pi, size = 6)
print (f"Random q is {q}")
print("------------------------------------------------------")
print(proofeffort(q,w))
```
- ทำการ Random q และ w เพื่อการพิสูจน์หลายๆเคส
### ตัวอย่างคำตอบ
```
Random q is [0.77665725 2.63697076 1.89892279]
------------------------------------------------------
Calculate effort is [1.193543 2.768885 2.109066]
RTB effort is [1.193543 2.768887 2.109068]
The answer is the same
```
- เปรียบเทียบกันจาก Calculate Effort กับ RTB Effort มีค่าเหมือนหรือต่างกันหรือไม่
