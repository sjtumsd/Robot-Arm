# -*- coding: utf-8 -*-
"""
Created on Wed Aug  2 17:10:19 2023

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


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np



print ("Please input steering engines' angles below:");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()

# Some data shared in the following
crank_center = chrono.ChVectorD(0,0.2,0)
crank_rad    = 0.4
crank_thick  = 0.1
rod_length   = 1
time=0
start_time=0
end_time1=0
end_time2=0
end_time3=0
end_time4=0
end_timeBase=0
end_timeClaw=0
paused=True



# Create the floor truss

mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVectorD(0,-0.5,0))
mfloor.SetBodyFixed(True)
sys.Add(mfloor)

# Create the flywheel crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center)
sys.Add(mcrank)

# Create four stylized rods
mrod1 = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod1.SetPos(crank_center + chrono.ChVectorD(rod_length/2 , 0, 0))

sys.Add(mrod1)

mrod2 = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod2.SetPos(crank_center + chrono.ChVectorD(rod_length*1.5 , 0, 0))

sys.Add(mrod2)

mrod3 = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod3.SetPos(crank_center + chrono.ChVectorD(rod_length*2.5 , 0, 0))
sys.Add(mrod3)

mrod4 = chrono.ChBodyEasyBox(rod_length*0.1, 0.1, 0.2, 1000)
mrod4.SetPos(crank_center + chrono.ChVectorD(rod_length*3.05 , 0, 0))
sys.Add(mrod4)

#create a claw
mClaw1 = chrono.ChBodyEasyBox(rod_length*0.4, 0.1, 0.05, 1000)
mClaw1.SetPos(crank_center + chrono.ChVectorD(rod_length*3.3 , 0, 0.075))
sys.Add(mClaw1)

mClaw2 = chrono.ChBodyEasyBox(rod_length*0.4, 0.1, 0.05, 1000)
mClaw2.SetPos(crank_center + chrono.ChVectorD(rod_length*3.3 , 0, -0.075))
sys.Add(mClaw2)

# Now create constraints and motors between the bodies.


# Create joints
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod1,
                   mcrank, 
                   chrono.ChCoordsysD( crank_center))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mrod2,
                   mrod1, 
                   chrono.ChCoordsysD( crank_center+chrono.ChVectorD(rod_length,0,0)))
sys.Add(mjointB)

mjointC = chrono.ChLinkLockRevolute()
mjointC.Initialize(mrod3,
                   mrod2, 
                   chrono.ChCoordsysD( crank_center+chrono.ChVectorD(rod_length*2,0,0)))
sys.Add(mjointC)

mjointD = chrono.ChLinkLockRevolute()
mjointD.Initialize(mrod4,
                   mrod3, 
                   chrono.ChCoordsysD( crank_center+chrono.ChVectorD(rod_length*3,0,0)))
sys.Add(mjointD)

mjointE = chrono.ChLinkLockRevolute()
mjointE.Initialize(mClaw1,
                   mrod4, 
                   chrono.ChCoordsysD( crank_center+chrono.ChVectorD(rod_length*3.1,0,0.05),chrono.Q_ROTATE_Z_TO_Y))
sys.Add(mjointE)

mjointF = chrono.ChLinkLockRevolute()
mjointF.Initialize(mClaw2,
                   mrod4, 
                   chrono.ChCoordsysD( crank_center+chrono.ChVectorD(rod_length*3.1,0,-0.05),chrono.Q_ROTATE_Z_TO_Y))
sys.Add(mjointF)

#motorBase
my_motorBase = chrono.ChLinkMotorRotationSpeed()
my_motorBase.Initialize(mcrank,   # the first connected body
                    mfloor,   # the second connected body
                    chrono.ChFrameD(crank_center,chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedBase = chrono.ChFunction_Const(0) 
my_motorBase.SetMotorFunction(my_angularspeedBase)
sys.Add(my_motorBase)
#motor1
my_motor1 = chrono.ChLinkMotorRotationSpeed()
my_motor1.Initialize(mrod1,   # the first connected body
                    mcrank,   # the second connected body
                    chrono.ChFrameD(crank_center)) # where to create the motor in abs.space
my_angularspeed1 = chrono.ChFunction_Const(0) 
my_motor1.SetMotorFunction(my_angularspeed1)
sys.Add(my_motor1)

#motor2
my_motor2 = chrono.ChLinkMotorRotationSpeed()
my_motor2.Initialize(mrod2,   # the first connected body
                    mrod1,   # the second connected body
                    chrono.ChFrameD(crank_center+chrono.ChVectorD(rod_length,0,0))) # where to create the motor in abs.space
my_angularspeed2 = chrono.ChFunction_Const(0) 
my_motor2.SetMotorFunction(my_angularspeed2)
sys.Add(my_motor2)

#motor3
my_motor3 = chrono.ChLinkMotorRotationSpeed()
my_motor3.Initialize(mrod3,   # the first connected body
                    mrod2,   # the second connected body
                    chrono.ChFrameD(crank_center+chrono.ChVectorD(rod_length*2,0,0))) # where to create the motor in abs.space
my_angularspeed3 = chrono.ChFunction_Const(0) 
my_motor3.SetMotorFunction(my_angularspeed3)
sys.Add(my_motor3)

#motor4
my_motor4 = chrono.ChLinkMotorRotationSpeed()
my_motor4.Initialize(mrod4,   # the first connected body
                    mrod3,   # the second connected body
                    chrono.ChFrameD(crank_center+chrono.ChVectorD(rod_length*3,0,0))) # where to create the motor in abs.space
my_angularspeed4 = chrono.ChFunction_Const(0) 
my_motor4.SetMotorFunction(my_angularspeed4)
sys.Add(my_motor4)

#motorclaw
my_motorClaw1 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw1.Initialize(mClaw1,   # the first connected body
                    mrod4,   # the second connected body
                    chrono.ChFrameD(crank_center+chrono.ChVectorD(rod_length*3.1,0,0.05),chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw1 = chrono.ChFunction_Const(0) 
my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
sys.Add(my_motorClaw1)

my_motorClaw2 = chrono.ChLinkMotorRotationSpeed()
my_motorClaw2.Initialize(mClaw2,   # the first connected body
                    mrod4,   # the second connected body
                    chrono.ChFrameD(crank_center+chrono.ChVectorD(rod_length*3.1,0,-0.05),chrono.Q_ROTATE_Z_TO_Y)) # where to create the motor in abs.space
my_angularspeedClaw2 = chrono.ChFunction_Const(0) 
my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
sys.Add(my_motorClaw2)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Robot Arm demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4,1,5), chrono.ChVectorD(0,1,0))
vis.AddTypicalLights()



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

  
    #input angle
    if paused==True:
        if sys.GetChTime()-time>3:
            a=float(input("angle Base : "))
            b=float(input("angle 1 : "))
            c=float(input("angle 2 : "))
            d=float(input("angle 3 : "))
            e=float(input("angle 4 : "))
            f=float(input("angle claw : "))
            time=sys.GetChTime()
            if a!=0:
                if a>0:
                    my_angularspeedBase = chrono.ChFunction_Const(chrono.CH_C_PI/10)
                    end_timeBase=a/18
                else:
                    my_angularspeedBase = chrono.ChFunction_Const(-chrono.CH_C_PI/10)
                    end_timeBase=-a/18
                my_motorBase.SetMotorFunction(my_angularspeedBase)
                paused=False
                start_time=time
            if b!=0:
                if b>0:
                    my_angularspeed1 = chrono.ChFunction_Const(chrono.CH_C_PI/10)
                    end_time1=b/18
                else:
                    my_angularspeed1 = chrono.ChFunction_Const(-chrono.CH_C_PI/10)
                    end_time1=-b/18
                my_motor1.SetMotorFunction(my_angularspeed1)
                paused=False
                start_time=time
            if c!=0:
                if c>0:
                    my_angularspeed2 = chrono.ChFunction_Const(chrono.CH_C_PI/10)
                    end_time2=c/18
                else:
                    my_angularspeed2 = chrono.ChFunction_Const(-chrono.CH_C_PI/10)
                    end_time2=-c/18
                my_motor2.SetMotorFunction(my_angularspeed2)
                paused=False
                start_time=time
            if d!=0:
                if d>0:
                    my_angularspeed3 = chrono.ChFunction_Const(chrono.CH_C_PI/10)
                    end_time3=d/18
                else:
                    my_angularspeed3 = chrono.ChFunction_Const(-chrono.CH_C_PI/10)
                    end_time3=-d/18
                my_motor3.SetMotorFunction(my_angularspeed3)
                paused=False
                start_time=time
            if e!=0:
                if e>0:
                    my_angularspeed4 = chrono.ChFunction_Const(chrono.CH_C_PI/10)
                    end_time4=e/18
                else:
                    my_angularspeed4 = chrono.ChFunction_Const(-chrono.CH_C_PI/10)
                    end_time4=-e/18
                my_motor4.SetMotorFunction(my_angularspeed4)
                paused=False
                start_time=time
            if f!=0:
                if f>0:
                    my_angularspeedClaw1 = chrono.ChFunction_Const(chrono.CH_C_PI/20)
                    my_angularspeedClaw2 = chrono.ChFunction_Const(-chrono.CH_C_PI/20)
                    end_timeClaw=f/18
                else:
                    my_angularspeedClaw1 = chrono.ChFunction_Const(-chrono.CH_C_PI/20)
                    my_angularspeedClaw2 = chrono.ChFunction_Const(chrono.CH_C_PI/20)
                    end_timeClaw=-f/18
                my_motorClaw1.SetMotorFunction(my_angularspeedClaw1)
                my_motorClaw2.SetMotorFunction(my_angularspeedClaw2)
                paused=False
                start_time=time
            
            
    else:
        if sys.GetChTime()-start_time>end_timeBase:
            my_angularspeedBase = chrono.ChFunction_Const(0)
            my_motorBase.SetMotorFunction(my_angularspeedBase)
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
        if sys.GetChTime()-start_time>max(end_timeBase,end_time1,end_time2,end_time3,end_time4,end_timeClaw):
            paused=True
        
            
    





