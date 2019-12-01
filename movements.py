import time

def stepUp1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets):
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 1, 1.2)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 2, -2.6)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 1, 0.253)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 2, -1.76)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 10, 0.463)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -1.137)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 4, 1.2)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 5, -2.6)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 4, 0.253)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 5, -1.76)


def actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, steps, jointID, movement):
    diff = movement - (p.getJointState(quadruped, jointID)[0] - jointOffsets[jointID]) / jointDirections[jointID]
    pos = (p.getJointState(quadruped, jointID)[0] - jointOffsets[jointID]) / jointDirections[jointID]
    for x in range(steps):
        maxForce = p.readUserDebugParameter(maxForceId)
        pos += (diff / steps)
        targetPos = pos
        p.setJointMotorControl2(quadruped, jointIds[jointID], p.POSITION_CONTROL,
                                jointDirections[jointID] * targetPos + jointOffsets[jointID], force=maxForce)
        p.stepSimulation()
        time.sleep(timeStep)

def moveForward1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets):
    for _ in range(5):
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -1.2)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 10, 0.4)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -1.3)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 8, -1.2)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 7, 0.4)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 8, -1.3)
        # actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 42, 10, 0.2)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 8, -0.6)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -0.6)
    for _ in range(400):
        time.sleep(timeStep)
        p.stepSimulation()
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 10, 1.1)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -2.3)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 10, 0.4)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 11, -1.7)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 7, 1.2)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 8, -2.3)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 7, 0.4)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 8, -1.7)

def standUp(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets ):
    joints = [1, 2, 4, 5, 7, 8, 10, 11]
    move = [0.848, -1.548, 0.848, -1.548, 0.848, -1.548, 0.848, -1.548]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints,move)

def standUp1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, diffd, dumbell, startOriD ):
    joints = [1, 2, 4, 5, 7, 8, 10, 11]
    move = [0.848, -1.548, 0.848, -1.548, 0.848, -1.548, 0.848, -1.548]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints,move, diffd, dumbell, startOriD)


def actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, steps, jointID, movement, diffd, dumbell, startOriD):
    diff = []
    pos = []
    for i in range(len(jointID)):
        diff.append(movement[i] - (p.getJointState(quadruped, jointID[i])[0] - jointOffsets[jointID[i]]) / jointDirections[jointID[i]])
        pos.append((p.getJointState(quadruped, jointID[i])[0] - jointOffsets[jointID[i]]) / jointDirections[jointID[i]])
    for x in range(steps):
        for i in range(len(jointID)):

            maxForce = p.readUserDebugParameter(maxForceId)
            pos[i] += diff[i] / steps
            targetPos = pos[i]
            p.setJointMotorControl2(quadruped, jointIds[jointID[i]], p.POSITION_CONTROL,
                                    jointDirections[jointID[i]] * targetPos + jointOffsets[jointID[i]], force=maxForce)

            pos1, ori = p.getBasePositionAndOrientation(quadruped)
            p.resetBasePositionAndOrientation(dumbell, [pos1[0] + diffd[0], pos1[1] + diffd[1], pos1[2] + diffd[2]],
                                              startOriD)
            p.stepSimulation()
            time.sleep(timeStep)



def actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, steps, jointID, movement):
    diff = []
    pos = []
    for i in range(len(jointID)):
        diff.append(movement[i] - (p.getJointState(quadruped, jointID[i])[0] - jointOffsets[jointID[i]]) / jointDirections[jointID[i]])
        pos.append((p.getJointState(quadruped, jointID[i])[0] - jointOffsets[jointID[i]]) / jointDirections[jointID[i]])
    for x in range(steps):
        for i in range(len(jointID)):

            maxForce = p.readUserDebugParameter(maxForceId)
            pos[i] += (diff[i] / steps)
            targetPos = pos[i]
            p.setJointMotorControl2(quadruped, jointIds[jointID[i]], p.POSITION_CONTROL,
                                    jointDirections[jointID[i]] * targetPos + jointOffsets[jointID[i]], force=maxForce)


            p.stepSimulation()
            time.sleep(timeStep)

    for _ in range(200):
        time.sleep(timeStep)
        p.stepSimulation()

def sitDown(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets):
    joints = [1, 2, 4, 5, 7, 8, 10, 11]
    move = [1.558, -2.863, 1.558, -2.863, 1.558, -2.863, 1.558, -2.863]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 50, joints, move)

def grab(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets ):
    joints = [13, 14]
    move = [3.158, -3.158]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 16, -3.142)
    joints = [12, 15]
    move = [1.726, -1.57]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 400, 16, 0)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 400, 15, 0)
    joints = [12, 13, 14]
    move = [0, 0, 0]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move)

def halfSit(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, diffd, obj, startOriD):
    joints = [1, 2, 4, 5, 7, 8, 10, 11]
    move = [1.168, -2.145, 1.168, -2.145, 1.168, -2.145, 1.168, -2.145]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 50, joints, move, diffd, obj, startOriD)

# def halfSit(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets):
#     joints = [1, 2, 4, 5, 7, 8, 10, 11]
#     move = [1.168, -2.145, 1.168, -2.145, 1.168, -2.145, 1.168, -2.145]
#     actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 50, joints, move)

def getDown(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, diffd, dumbell, startOriD):
    joints = [1, 4]
    move = [-2.526, -2.516]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, joints, move, diffd, dumbell, startOriD)
    joints = [2, 5]
    move = [0, 0]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, joints, move, diffd, dumbell, startOriD)
    joints = [1, 4]
    move = [-0.968, -0.968]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, joints, move, diffd, dumbell, startOriD)
    joints = [8, 7, 11, 10]
    move = [0, -0.968, 0, -0.968]
    actuateBulk1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move, diffd, dumbell, startOriD)

def backLegDown(p,timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, diffd, dumbell, startOriD):
    for _ in range(20):
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 2, -1.15)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 1, 0.4)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 2, -1.3)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 5, -1.15)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 4, 0.4)
        actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 5, -1.3)


def drop(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets ):
    joints = [12, 13, 14, 15]
    move = [1.726, 3.158, -3.158, 1.726]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move)
    actuate(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 21, 16, -3.142)
    for _ in range(400):
        time.sleep(timeStep)
        p.stepSimulation()
    joints = [12, 13, 14, 15, 16]
    move = [0, 0, 0, 0, 0]
    actuateBulk(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets, 100, joints, move)
