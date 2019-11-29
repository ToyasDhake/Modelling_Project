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

def moveForward1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets ):
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
    for _ in range(500):
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


