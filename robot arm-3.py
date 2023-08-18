# -*- coding: utf-8 -*-
"""
Created on Thu Aug 17 14:29:20 2023

@author: Lenovo
"""
# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import time
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
from config import *
from arm4dof import Arm4DoF
import csv



# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()
time=0
start_time=0
end_time1=0
end_time2=0
end_time3=0
end_time4=0
end_timeClaw=0
paused=True
point=0
theta_last=[0,0,0,0,0]


arm_real = Arm4DoF()
theta_now=arm_real.get_thetas()

# Some data shared in the following

base_link_link1=chrono.ChVectorD(0,0.0489,0)

link1_link2=chrono.ChVectorD(0.0022,0.021,0)

link2_link3=chrono.ChVectorD(0.081,0.00022,0)

link3_link4=chrono.ChVectorD(0.0772,0,0)
link4_left=chrono.ChVectorD(0.054,0.0006,-0.0122)
link4_right=chrono.ChVectorD(0.054,0,0.01)

bluewhite=chrono.ChVisualMaterial()
bluewhite.SetKdTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
bluewhite.SetTextureScale(30,30)

# Create the floor truss

mfloor = chrono.ChBodyEasyBox(3,0.1,3, 200)
mfloor.SetPos(chrono.ChVectorD(0,-0.05,0))
mfloor.SetBodyFixed(True)
mfloor.GetVisualShape(0).SetMaterial(0,bluewhite)
sys.Add(mfloor)

# Create the flywheel crank


# Create four stylized rods

base_link=chrono.ChBodyEasyMesh("4dofArm/base_link.obj",2000)
base_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
base_link.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
base_link.SetBodyFixed(True)
base_link.GetVisualShape(0).SetColor(chrono.ChColor(0.02, 0.02, 0.02))
sys.Add(base_link)


link1=chrono.ChBodyEasyMesh("4dofArm/link1.obj",2000)
link1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link1.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
link1.SetPos(base_link_link1)
link1.GetVisualShape(0).SetColor(chrono.ChColor(0.08, 0.08, 0.08))
sys.Add(link1)


link2=chrono.ChBodyEasyMesh("4dofArm/link2.obj",200)
link2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link2.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(0,0,0))))
link2.SetPos(base_link_link1+link1_link2)
link2.GetVisualShape(0).SetColor(chrono.ChColor(0.08, 0.08, 0.08))
sys.Add(link2)

link3=chrono.ChBodyEasyMesh("4dofArm/link3.obj",200)
link3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link3.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(0,0,0))))
link3.SetPos(base_link_link1+link1_link2+link2_link3)
link3.GetVisualShape(0).SetColor(chrono.ChColor(0.05, 0.05, 0.05))
sys.Add(link3)

link4=chrono.ChBodyEasyMesh("4dofArm/link4.obj",200)
link4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link4.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(chrono.CH_C_PI,0,0))))
link4.SetPos(base_link_link1+link1_link2+link2_link3+link3_link4)
link4.GetVisualShape(0).SetColor(chrono.ChColor(0.05, 0.05, 0.05))
sys.Add(link4)

#create a claw
left_gripper_link=chrono.ChBodyEasyMesh("4dofArm/left_gripper_link.obj",200)
left_gripper_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
left_gripper_link.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
left_gripper_link.SetPos(base_link_link1+link1_link2+link2_link3+link3_link4+link4_left)
left_gripper_link.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 0))
sys.Add(left_gripper_link)

right_gripper_link=chrono.ChBodyEasyMesh("4dofArm/right_gripper_link.obj",200)
right_gripper_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
right_gripper_link.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
right_gripper_link.SetPos(base_link_link1+link1_link2+link2_link3+link3_link4+link4_right)
right_gripper_link.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 0))
sys.Add(right_gripper_link)

# Now create constraints and motors between the bodies.


# Create joints

mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(link1,
                   base_link, 
                   chrono.ChCoordsysD(base_link_link1))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(link2,
                   link1, 
                   chrono.ChCoordsysD(base_link_link1+link1_link2))
sys.Add(mjointB)

mjointC = chrono.ChLinkLockRevolute()
mjointC.Initialize(link3,
                   link2, 
                   chrono.ChCoordsysD(base_link_link1+link1_link2+link2_link3))
sys.Add(mjointC)

mjointD = chrono.ChLinkLockRevolute()
mjointD.Initialize(link4,
                   link3, 
                   chrono.ChCoordsysD(base_link_link1+link1_link2+link2_link3+link3_link4))
sys.Add(mjointD)

mjointE = chrono.ChLinkLockRevolute()
mjointE.Initialize(left_gripper_link,
                   link4, 
                   chrono.ChCoordsysD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_left))
sys.Add(mjointE)

mjointF = chrono.ChLinkLockRevolute()
mjointF.Initialize(right_gripper_link,
                   link4, 
                   chrono.ChCoordsysD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_right))
sys.Add(mjointF)

#motorBase
'''
my_motorBase = chrono.ChLinkMotorRotationSpeed()
my_motorBase.Initialize(mcrank,   # the first connected body
                    mfloor,   # the second connected body
                    chrono.ChFrameD(base_link_link1,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedBase = chrono.ChFunction_Const(0) 
my_motorBase.SetMotorFunction(my_angularspeedBase)
sys.Add(my_motorBase)
'''
#motor1
my_motor1 = chrono.ChLinkMotorRotationSpeed()
my_motor1.Initialize(link1,   # the first connected body
                    base_link,   # the second connected body
                    chrono.ChFrameD(base_link_link1,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeed1 = chrono.ChFunction_Const((theta_now[0]-theta_last[0]))
my_motor1.SetMotorFunction(my_angularspeed1)
sys.Add(my_motor1)

#motor2
my_motor2 = chrono.ChLinkMotorRotationSpeed()
my_motor2.Initialize(link2,   # the first connected body
                    link1,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2)) # where to create the motor in abs.space
my_angularspeed2 = chrono.ChFunction_Const(-(theta_now[1]-theta_last[1]))
my_motor2.SetMotorFunction(my_angularspeed2)
sys.Add(my_motor2)

#motor3
my_motor3 = chrono.ChLinkMotorRotationSpeed()
my_motor3.Initialize(link3,   # the first connected body
                    link2,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3)) # where to create the motor in abs.space

my_angularspeed3 = chrono.ChFunction_Const(-(theta_now[2]-theta_last[2]))
my_motor3.SetMotorFunction(my_angularspeed3)
sys.Add(my_motor3)

#motor4
my_motor4 = chrono.ChLinkMotorRotationSpeed()
my_motor4.Initialize(link4,   # the first connected body
                    link3,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4)) # where to create the motor in abs.space
my_angularspeed4 = chrono.ChFunction_Const(-(theta_now[3]-theta_last[3]))
my_motor4.SetMotorFunction(my_angularspeed4)
sys.Add(my_motor4)

#motorclaw
my_motorClaw1 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw1.Initialize(left_gripper_link,   # the first connected body
                    link4,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_left,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw1 = chrono.ChFunction_Const((theta_now[4]-theta_last[4]))
my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
sys.Add(my_motorClaw1)

my_motorClaw2 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw2.Initialize(right_gripper_link,   # the first connected body
                    link4,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_right,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw2 = chrono.ChFunction_Const(-(theta_now[4]-theta_last[4]))
my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
sys.Add(my_motorClaw2)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(3840,2160)
vis.SetWindowTitle('Robot Arm demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.3,0.25,0.2), chrono.ChVectorD(0,0.01,0))
vis.AddTypicalLights()



pointmax=0
file=open("angles.txt")
angles=[]
csv_reader=csv.reader(file)
for line in csv_reader:
    angles.append(line)
    print(line)
    pointmax=pointmax+1
file.close()

theta=[0,0,0,0,0]






# --------------------------------------------------------------------
#
#  Run the simulation
#

# Run the interactive simulation loop
while vis.Run():
    
    # here happens the visualization and step time integration
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    print(sys.GetChTime())
    if sys.GetChTime()<1:
        continue
    if sys.GetChTime()>1 and sys.GetChTime()<1.5:
        my_angularspeed1 = chrono.ChFunction_Const(0)
        my_motor1.SetMotorFunction(my_angularspeed1)
        my_angularspeed2 = chrono.ChFunction_Const(0)
        my_motor2.SetMotorFunction(my_angularspeed2)
        my_angularspeed3 = chrono.ChFunction_Const(0)
        my_motor3.SetMotorFunction(my_angularspeed3)
        my_angularspeed4 = chrono.ChFunction_Const(0)
        my_motor4.SetMotorFunction(my_angularspeed4)
        my_angularspeedClaw1 = chrono.ChFunction_Const(0)
        my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
        my_angularspeedClaw2 = chrono.ChFunction_Const(0)
        my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
        continue
    b=my_motor1.GetMotorRot()/chrono.CH_C_PI*180
    c=my_motor2.GetMotorRot()/chrono.CH_C_PI*180
    d=my_motor3.GetMotorRot()/chrono.CH_C_PI*180
    e=my_motor4.GetMotorRot()/chrono.CH_C_PI*180
    f=my_motorClaw2.GetMotorRot()/chrono.CH_C_PI*180
    print(b,c,d,e,f)
    #input angle
    if paused==True:
        if point<pointmax:
            #a=float(input("angle Base : "))
            b=float(angles[point][0])-b
            c=float(angles[point][1])-c
            d=float(angles[point][2])-d
            e=float(angles[point][3])-e
            f=float(angles[point][4])-f
            point=point+1
            time=sys.GetChTime()
            '''
            if a!=0:
                if a>0:
                    my_angularspeedBase = chrono.ChFunction_Const(chrono.CH_C_PI/2)
                    end_timeBase=a/90
                else:
                    my_angularspeedBase = chrono.ChFunction_Const(-chrono.CH_C_PI/2)
                    end_timeBase=-a/90
                my_motorBase.SetMotorFunction(my_angularspeedBase)
                paused=False
                start_time=time
            '''
            if b!=0:
                if b>0:
                    my_angularspeed1 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    end_time1=b/180
                else:
                    my_angularspeed1 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    end_time1=-b/180
                my_motor1.SetMotorFunction(my_angularspeed1)
                paused=False
                start_time=time
            if c!=0:
                if c>0:
                    my_angularspeed2 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    end_time2=c/180
                else:
                    my_angularspeed2 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    end_time2=-c/180
                my_motor2.SetMotorFunction(my_angularspeed2)
                paused=False
                start_time=time
            if d!=0:
                if d>0:
                    my_angularspeed3 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    end_time3=d/180
                else:
                    my_angularspeed3 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    end_time3=-d/180
                my_motor3.SetMotorFunction(my_angularspeed3)
                paused=False
                start_time=time
            if e!=0:
                if e>0:
                    my_angularspeed4 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    end_time4=e/180
                else:
                    my_angularspeed4 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    end_time4=-e/180
                my_motor4.SetMotorFunction(my_angularspeed4)
                paused=False
                start_time=time
            if f!=0:
                if f>0:
                    my_angularspeedClaw1 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    my_angularspeedClaw2 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    end_timeClaw=f/180
                else:
                    my_angularspeedClaw1 = chrono.ChFunction_Const(chrono.CH_C_PI)
                    my_angularspeedClaw2 = chrono.ChFunction_Const(-chrono.CH_C_PI)
                    end_timeClaw=-f/180
                my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
                my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
                paused=False
                start_time=time
            
            
            
    else:
        '''
        if sys.GetChTime()-start_time>end_timeBase:
            my_angularspeedBase = chrono.ChFunction_Const(0)
            my_motorBase.SetMotorFunction(my_angularspeedBase)
        '''
        theta[0]=my_motor1.GetMotorRot()
        theta[1]=-my_motor2.GetMotorRot()
        theta[2]=-my_motor3.GetMotorRot()
        theta[3]=-my_motor4.GetMotorRot()
        theta[4]=my_motorClaw1.GetMotorRot()
        arm_real.set_joint(theta, interval=0, power=0)
        if sys.GetChTime()-start_time>end_time1:
            my_angularspeed1 = chrono.ChFunction_Const(0)
            my_motor1.SetMotorFunction(my_angularspeed1)
        if sys.GetChTime()-start_time>end_time2:
            my_angularspeed2 = chrono.ChFunction_Const(0)
            my_motor2.SetMotorFunction(my_angularspeed2)
        if sys.GetChTime()-start_time>end_time3:
            my_angularspeed3 = chrono.ChFunction_Const(0)
            my_motor3.SetMotorFunction(my_angularspeed3)
        if sys.GetChTime()-start_time>end_time4:
            my_angularspeed4 = chrono.ChFunction_Const(0)
            my_motor4.SetMotorFunction(my_angularspeed4)
        if sys.GetChTime()-start_time>end_timeClaw:
            my_angularspeedClaw1 = chrono.ChFunction_Const(0)
            my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
            my_angularspeedClaw2 = chrono.ChFunction_Const(0)
            my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
        if sys.GetChTime()-start_time>max(end_time1,end_time2,end_time3,end_time4,end_timeClaw):
            paused=True        
            


            


