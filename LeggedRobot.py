# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 19:14:37 2019

@author: toyas
"""

import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
plane = p.loadURDF("ModelData/plane.urdf")
p.setGravity(0, 0, -9.8)
p.setTimeStep(1. / 500)

urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("ModelData/LeggedRobot.urdf", [0, 0, .5], [0, 0.5, 0.5, 0], flags=urdfFlags, useFixedBase=False)

# enable collision between lower legs
lower_legs = [2, 5, 8, 11]
for l0 in lower_legs:
    for l1 in lower_legs:
        if (l1 > l0):
            enableCollision = 1
            p.setCollisionFilterPair(quadruped, quadruped, 2, 5, enableCollision)

jointIds = []
paramIds = []
jointOffsets = []
jointDirections = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1]
jointAngles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

for i in range(4):
    jointOffsets.append(0)
    jointOffsets.append(-0.7)
    jointOffsets.append(0.7)

maxForceId = p.addUserDebugParameter("maxForce", 0, 100, 20)

for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)

p.getCameraImage(480, 320)
p.setRealTimeSimulation(0)

joints = []

with open("data.txt", "r") as filestream:
    for line in filestream:
        maxForce = p.readUserDebugParameter(maxForceId)
        currentline = line.split(",")
        frame = currentline[0]
        t = currentline[1]
        joints = currentline[2:14]
        for j in range(12):
            targetPos = float(joints[j])
            p.setJointMotorControl2(quadruped, jointIds[j], p.POSITION_CONTROL,
                                    jointDirections[j] * targetPos + jointOffsets[j], force=maxForce)
        p.stepSimulation()
        time.sleep(1. / 500.)

#for j in range(p.getNumJoints(quadruped)):
#    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
#    info = p.getJointInfo(quadruped, j)
#    js = p.getJointState(quadruped, j)
#    jointName = info[1]
#    jointType = info[2]
#    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
#        paramIds.append(
#            p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, (js[0] - getJointState[j]) / jointDirections[j]))
#
#p.setRealTimeSimulation(1)
#
#while (1):
#
#    for i in range(len(paramIds)):
#        c = paramIds[i]
#        targetPos = p.readUserDebugParameter(c)
#        maxForce = p.readUserDebugParameter(maxForceId)
#        p.setJointMotorControl2(quadruped, jointIds[i], p.POSITION_CONTROL,
#                                jointDirections[i] * targetPos + jointOffsets[i], force=maxForce)
