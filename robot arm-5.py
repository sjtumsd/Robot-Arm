# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 10:49:30 2023

@author: 28622
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
import matplotlib.pyplot as plt
import numpy as np
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


arm_real = Arm4DoF(is_init_pose=False)


sys      = chrono.ChSystemNSC()

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0005);
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0000001);

time=0
start_time=0
end_time1=0
end_time2=0
end_time3=0
end_time4=0
end_timeClaw=0
paused=True
point=0
box_mat = chrono.ChMaterialSurfaceNSC()
box_mat.SetFriction(1)
claw_mat = chrono.ChMaterialSurfaceNSC()
claw_mat.SetFriction(1)


# Some data shared in the following

base_link_link1=chrono.ChVectorD(0,0.0489+0.015,0)
link1_link2=chrono.ChVectorD(0.0025,0.021,0)
link2_link3=chrono.ChVectorD(0.081,0.00025,0)
link3_link4=chrono.ChVectorD(0.0775,0,0)
link4_left=chrono.ChVectorD(0.054,0.0006,-0.0125)
link4_right=chrono.ChVectorD(0.054,0,0.01)

bluewhite=chrono.ChVisualMaterial()
bluewhite.SetKdTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
bluewhite.SetTextureScale(30,30)

redwhite=chrono.ChVisualMaterial()
redwhite.SetKdTexture(chrono.GetChronoDataFile("textures/redwhite.png"))
# Create the floor truss
mfloor = chrono.ChBodyEasyBox(3,0.1,3, 1000,True,True,box_mat)
mfloor.SetPos(chrono.ChVectorD(0,-0.05,0))
mfloor.SetBodyFixed(True)
mfloor.GetVisualShape(0).SetMaterial(0,bluewhite)
sys.Add(mfloor)
'''
cylinder=chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.05, 0.006, 1000,True,True,box_mat)
cylinder.SetPos(chrono.ChVectorD(0.216506,0.003,0.125))
cylinder.SetBodyFixed(True)
cylinder.GetVisualShape(0).SetMaterial(0,bluewhite)
sys.Add(cylinder)
'''
# Create the box

box=chrono.ChBodyEasyBox(0.0288,0.0288,0.0288, 600,True,True,box_mat)
box.SetPos(chrono.ChVectorD(0.2,0.0144,0))
#box.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(0,-chrono.CH_C_PI/6,0)))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/cubetexture_pinkwhite.png"))
sys.Add(box)

# Create four stylized rods

base_link=chrono.ChBodyAuxRef()
base_link.SetMass(0.13137)
base_link.SetInertiaXX(chrono.ChVectorD(0.00013449,0.000085546,0.00020424))
base_link.SetInertiaXY(chrono.ChVectorD(0.00000083293,0.000000020089,0.00000000034179))
mesh_base= chrono.ChTriangleMeshConnected()
mesh_base.LoadWavefrontMesh("4dofArm/base_link.obj")
visualision_base=chrono.ChTriangleMeshShape()
visualision_base.SetMesh(mesh_base)
visualision_base.SetColor(chrono.ChColor(0.05, 0.05, 0.05))
base_link.AddVisualShape(visualision_base)
base_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
base_link.SetFrame_REF_to_abs(chrono.ChFrameD(chrono.ChVectorD(0, 0.015, 0),chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
base_link.SetBodyFixed(True)
sys.Add(base_link)
print(base_link.GetInertiaXX())
print(base_link.GetInertiaXY())


link1=chrono.ChBodyAuxRef()
link1.SetMass(0.03785)
link1.SetInertiaXX(chrono.ChVectorD(3.3801e-6,6.4346e-6,7.0732e-6))
link1.SetInertiaXY(chrono.ChVectorD(-1.1646e-8,1.7331e-7,6.1209e-22))
mesh_link1= chrono.ChTriangleMeshConnected()
mesh_link1.LoadWavefrontMesh("4dofArm/link1.obj")
visualision_link1=chrono.ChTriangleMeshShape()
visualision_link1.SetMesh(mesh_link1)
visualision_link1.SetColor(chrono.ChColor(0.08, 0.08, 0.08))
link1.AddVisualShape(visualision_link1)
link1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link1.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1,chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
sys.Add(link1)
print(link1.GetInertiaXX())
print(link1.GetInertiaXY())

link2=chrono.ChBodyAuxRef()
link2.SetMass(0.013916)
link2.SetInertiaXX(chrono.ChVectorD(5.3188e-6,7.1757e-6,2.7556e-6))
link2.SetInertiaXY(chrono.ChVectorD(6.3983e-21,-3.6987e-21,-7.0513e-20))
mesh_link2= chrono.ChTriangleMeshConnected()
mesh_link2.LoadWavefrontMesh("4dofArm/link2.obj")
visualision_link2=chrono.ChTriangleMeshShape()
visualision_link2.SetMesh(mesh_link2)
visualision_link2.SetColor(chrono.ChColor(0.08, 0.08, 0.08))
link2.AddVisualShape(visualision_link2)
link2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link2.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1+link1_link2,chrono.Q_from_Euler123(chrono.ChVectorD(0,0,0))))
sys.Add(link2)
print(link2.GetInertiaXX())
print(link2.GetInertiaXY())

link3=chrono.ChBodyAuxRef()
link3.SetMass(0.042611)
link3.SetInertiaXX(chrono.ChVectorD(5.6329e-6,9.8802e-6,7.0504e-6))
link3.SetInertiaXY(chrono.ChVectorD(-6.8971e-22,1.1646e-8,-1.2028e-23))
mesh_link3= chrono.ChTriangleMeshConnected()
mesh_link3.LoadWavefrontMesh("4dofArm/link3.obj")
visualision_link3=chrono.ChTriangleMeshShape()
visualision_link3.SetMesh(mesh_link3)
visualision_link3.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
link3.AddVisualShape(visualision_link3)
link3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link3.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1+link1_link2+link2_link3,chrono.Q_from_Euler123(chrono.ChVectorD(0,0,0))))
sys.Add(link3)
print(link3.GetInertiaXX())
print(link3.GetInertiaXY())

link4=chrono.ChBodyAuxRef()
link4.SetMass(0.075884)
link4.SetInertiaXX(chrono.ChVectorD(1.1425e-5,1.3159e-5,1.0421e-5))
link4.SetInertiaXY(chrono.ChVectorD(-3.8837e-21,-2.7541e-7,-1.1646e-8))
mesh_link4= chrono.ChTriangleMeshConnected()
mesh_link4.LoadWavefrontMesh("4dofArm/link4.obj")
visualision_link4=chrono.ChTriangleMeshShape()
visualision_link4.SetMesh(mesh_link4)
visualision_link4.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
link4.AddVisualShape(visualision_link4)
link4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
link4.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4,chrono.Q_from_Euler123(chrono.ChVectorD(chrono.CH_C_PI,0,0))))
sys.Add(link4)
print(link4.GetInertiaXX())
print(link4.GetInertiaXY())

#create a claw
left_gripper_link=chrono.ChBodyAuxRef()
left_gripper_link.SetMass(0.00958)
left_gripper_link.SetInertiaXX(chrono.ChVectorD(3.2066e-6,9.9703e-6,7.3612e-6))
left_gripper_link.SetInertiaXY(chrono.ChVectorD(2.7765e-7,1.3563e-13,-9.1518e-15))

mesh_left= chrono.ChTriangleMeshConnected()
mesh_left.LoadWavefrontMesh("4dofArm/left_gripper_link.obj")
visualision_left=chrono.ChTriangleMeshShape()
visualision_left.SetMesh(mesh_left)
visualision_left.SetColor(chrono.ChColor(0, 0, 0))
left_gripper_link.AddVisualShape(visualision_left)

left_gripper_link.GetCollisionModel().ClearModel()
left_gripper_link.GetCollisionModel().AddTriangleMesh(
            claw_mat, # contact material
            mesh_left, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            # , mpos, mr,  # pos of mesh respect to REF and rotation matr.respect to REF 
            # 0.01) # 'inflating' radiust for triangles for increased robustness
left_gripper_link.GetCollisionModel().BuildModel()
left_gripper_link.SetCollide(True)

left_gripper_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
left_gripper_link.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_left,chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))
sys.Add(left_gripper_link)
print(left_gripper_link.GetInertiaXX())
print(left_gripper_link.GetInertiaXY())


right_gripper_link=chrono.ChBodyAuxRef()
right_gripper_link.SetMass(0.011394)
right_gripper_link.SetInertiaXX(chrono.ChVectorD(3.2567e-6,1.002e-5,7.457e-6))
right_gripper_link.SetInertiaXY(chrono.ChVectorD(-2.7765e-7,-1.3563e-13,-9.1517e-15))

mesh_right= chrono.ChTriangleMeshConnected()
mesh_right.LoadWavefrontMesh("4dofArm/right_gripper_link.obj")
visualision_right=chrono.ChTriangleMeshShape()
visualision_right.SetMesh(mesh_right)
visualision_right.SetColor(chrono.ChColor(0, 0, 0))
right_gripper_link.AddVisualShape(visualision_right)

right_gripper_link.GetCollisionModel().ClearModel()
right_gripper_link.GetCollisionModel().AddTriangleMesh(
            claw_mat, # contact material
            mesh_right, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            # , mpos, mr,  # pos of mesh respect to REF and rotation matr.respect to REF 
            # 0.01) # 'inflating' radiust for triangles for increased robustness
right_gripper_link.GetCollisionModel().BuildModel()
right_gripper_link.SetCollide(True)

right_gripper_link.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
right_gripper_link.SetFrame_REF_to_abs(chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_right,chrono.Q_from_Euler123(chrono.ChVectorD(-chrono.CH_C_PI/2,0,0))))

sys.Add(right_gripper_link)
print(right_gripper_link.GetInertiaXX())
print(right_gripper_link.GetInertiaXY())
#box.GetCollisionModel().SetContactable(left_gripper_link.GetCollisionModel().GetContactable())

#left_gripper_link.GetCollisionModel().SetContactable(mfloor.GetCollisionModel().GetContactable())
#left_gripper_link.GetCollisionModel().SetContactable(box.GetCollisionModel().GetContactable())
#left_gripper_link.GetCollisionModel().SetContactable(right_gripper_link.GetCollisionModel().GetContactable())
#left_gripper_link.GetCollisionModel().SetContactable(mfloor.GetCollisionModel().GetContactable())

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
my_angularspeed1 = chrono.ChFunction_Const(0) 
my_motor1.SetMotorFunction(my_angularspeed1)
sys.Add(my_motor1)

#motor2
my_motor2 = chrono.ChLinkMotorRotationSpeed()
my_motor2.Initialize(link2,   # the first connected body
                    link1,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2)) # where to create the motor in abs.space
my_angularspeed2 = chrono.ChFunction_Const(0) 
my_motor2.SetMotorFunction(my_angularspeed2)
sys.Add(my_motor2)

#motor3
my_motor3 = chrono.ChLinkMotorRotationSpeed()
my_motor3.Initialize(link3,   # the first connected body
                    link2,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3)) # where to create the motor in abs.space
my_angularspeed3 = chrono.ChFunction_Const(0) 
my_motor3.SetMotorFunction(my_angularspeed3)
sys.Add(my_motor3)

#motor4
my_motor4 = chrono.ChLinkMotorRotationSpeed()
my_motor4.Initialize(link4,   # the first connected body
                    link3,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4)) # where to create the motor in abs.space
my_angularspeed4 = chrono.ChFunction_Const(0) 
my_motor4.SetMotorFunction(my_angularspeed4)
sys.Add(my_motor4)

#motorclaw
my_motorClaw1 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw1.Initialize(left_gripper_link,   # the first connected body
                    link4,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_left,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw1 = chrono.ChFunction_Const(0) 
my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
sys.Add(my_motorClaw1)

my_motorClaw2 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw2.Initialize(right_gripper_link,   # the first connected body
                    link4,   # the second connected body
                    chrono.ChFrameD(base_link_link1+link1_link2+link2_link3+link3_link4+link4_right,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw2 = chrono.ChFunction_Const(0) 
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
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5,0.3,0.4), chrono.ChVectorD(0,0.1,0))
vis.AddTypicalLights()


theta_last=[0,0,0,0,0]

ep=1e-3
array_time=[]
motor1_torque=[]
motor2_torque=[]
motor3_torque=[]
motor4_torque=[]
motorClaw_torque=[]
arm_real.set_damping(1)

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
    theta_now=arm_real.get_thetas()
    b=my_motor1.GetMotorRot()
    c=my_motor2.GetMotorRot()
    d=my_motor3.GetMotorRot()
    e=my_motor4.GetMotorRot()
    f=my_motorClaw2.GetMotorRot()
    g=my_motorClaw1.GetMotorRot()
    if abs(theta_now[0]-b)>ep: 
        my_angularspeed1 = chrono.ChFunction_Const((theta_now[0]-b)/(4*1e-4))
        my_motor1.SetMotorFunction(my_angularspeed1)
    else:
        my_angularspeed1 = chrono.ChFunction_Const(0)
        my_motor1.SetMotorFunction(my_angularspeed1)
    if abs(theta_now[1]+c)>ep:
        my_angularspeed2 = chrono.ChFunction_Const(-(theta_now[1]+c)/(4*1e-4))
        my_motor2.SetMotorFunction(my_angularspeed2)
    else:
        my_angularspeed2 = chrono.ChFunction_Const(0)
        my_motor2.SetMotorFunction(my_angularspeed2)
    if abs(theta_now[2]+d)>ep:
        my_angularspeed3 = chrono.ChFunction_Const(-(theta_now[2]+d)/(4*1e-4))
        my_motor3.SetMotorFunction(my_angularspeed3)
    else:
        my_angularspeed3 = chrono.ChFunction_Const(0)
        my_motor3.SetMotorFunction(my_angularspeed3)
    if abs(theta_now[3]+e)>ep:
        my_angularspeed4 = chrono.ChFunction_Const(-(theta_now[3]+e)/(4*1e-4))
        my_motor4.SetMotorFunction(my_angularspeed4)
    else:
        my_angularspeed4 = chrono.ChFunction_Const(0)
        my_motor4.SetMotorFunction(my_angularspeed4)
    if abs(theta_now[4]-g)>ep:
        my_angularspeedClaw1 = chrono.ChFunction_Const((theta_now[4]-g)/(4*1e-4))
        my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
    else:
        my_angularspeedClaw1 = chrono.ChFunction_Const(0)
        my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
    if abs(theta_now[4]+f)>ep:
        my_angularspeedClaw2 = chrono.ChFunction_Const(-(theta_now[4]+f)/(4*1e-4))
        my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
    else:
        my_angularspeedClaw2 = chrono.ChFunction_Const(0)
        my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
    print(b,c,d,e,f,g)     
    sys.DoStepDynamics(4*1e-4)
        
fig, (ax1, ax2,ax3,ax4,ax5) = plt.subplots(5, sharex = True)

ax1.plot(array_time, motor1_torque)
ax1.set(ylabel='Motor1[N*m]')
ax1.grid()

ax2.plot(array_time, motor2_torque)
ax2.set(ylabel='Motor2[N*m]')
ax2.grid()

ax3.plot(array_time, motor3_torque)
ax3.set(ylabel='Motor3[N*m]')
ax3.grid()

ax4.plot(array_time, motor4_torque)
ax4.set(ylabel='Motor4[N*m]')
ax4.grid()

ax5.plot(array_time, motorClaw_torque)
ax5.set(ylabel='MotorClaw[N*m]',xlabel='time[s]')
ax5.grid()
