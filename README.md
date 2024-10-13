# FRA333_HW3_6519_6557
## Done by
- 65340500019 ณพวิทย์ อินทร์จันทร์
- 65340500057 อนัญญา อุจจธรรมรัตน์
## File
- Answer file : FRA333_HW3_6519_6557.py
- Prove file  : test_scriptHW3.ipynb

## Objective :
การบ้านนี้ถูกออกแบบึ้นมาเพื่อให้ผู้เรียนได้ประยุกต์ใช้องค์ความรู้การหาจลนศาสตร์เชิงอนุพันธ์ (Differential kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator)

## Find DH Parameter of a robot
![IMG_1908.jpg](IMG_1908.jpg)

### Import Library
```
import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from math import pi

from HW3_utils import *
from FRA333_HW3_6519_6557 import *
```

### Robot DH Parameter
```
#define link length
d1,a2, = 0.0892,-0.425

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
