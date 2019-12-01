import pybullet as p1
from pybullet_utils import bullet_client
import pybullet_data
from pybullet_utils import pd_controller_stable
import movements
import time
import motion_capture_data
import quadrupedPoseInterpolator

useKinematic = False
useConstraints = True

p = bullet_client.BulletClient(connection_mode=p1.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
timeStep = 1. / 500
p.setTimeStep(timeStep)
# p.setDefaultContactERP(0)
# urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
urdfFlags = p.URDF_USE_SELF_COLLISION

startPos = [0.007714045256424385, 2.4832953726079623, 0.47220767977764067]
startOrn = [-0.015613979977597187, 0.7128507268398393, 0.7009593948969438, -0.01599911181542653]
quadruped = p.loadURDF("ModelData/LeggedRobot.urdf",
                       startPos,
                       startOrn,
                       flags=urdfFlags,
                       useFixedBase=False)
obstacle = p.loadURDF("ModelData/obstacle.urdf", useFixedBase=True)
p.resetBasePositionAndOrientation(quadruped, startPos, startOrn)
if not useConstraints:
    for j in range(p.getNumJoints(quadruped)):
        p.setJointMotorControl2(quadruped, j, p.POSITION_CONTROL, force=0)

# This cube is added as a soft constraint to keep the laikago from falling
# since we didn't train it yet, it doesn't balance
cube = p.loadURDF("cube_no_rotation.urdf", [0, 0, -0.5], [0, 0.5, 0.5, 0])
p.setCollisionFilterGroupMask(cube, -1, 0, 0)
for j in range(p.getNumJoints(cube)):
    p.setJointMotorControl2(cube, j, p.POSITION_CONTROL, force=0)
    p.setCollisionFilterGroupMask(cube, j, 0, 0)
    p.changeVisualShape(cube, j, rgbaColor=[1, 0, 0, 0])
cid = p.createConstraint(cube,
                         p.getNumJoints(cube) - 1, quadruped, -1, p.JOINT_FIXED, [0, 0, 0],
                         [0, 1, 0], [0, 0, 0])
p.changeConstraint(cid, maxForce=10)

jointIds = []
paramIds = []
jointOffsets = []
jointDirections = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
jointAngles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

for i in range(4):
    jointOffsets.append(0)
    jointOffsets.append(-0.7)
    jointOffsets.append(0.7)
for _ in range(5):
    jointOffsets.append(0)

maxForceId = p.addUserDebugParameter("maxForce", 0, 100, 120)

for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)

startQ = [
    0.08389, 0.8482, -1.547832, -0.068933, 0.625726, -1.272086, 0.074398, 0.61135, -1.255892,
    -0.068262, 0.836745, -1.534517, 0, 0, 0, 0, 0
]

for j in range(p.getNumJoints(quadruped)):
    p.resetJointState(quadruped, jointIds[j], jointDirections[j] * startQ[j] + jointOffsets[j])

qpi = quadrupedPoseInterpolator.QuadrupedPoseInterpolator()

# enable collision between lower legs

for j in range(p.getNumJoints(quadruped)):
    print(p.getJointInfo(quadruped, j))

# 2,5,8 and 11 are the lower legs
lower_legs = [2, 5, 8, 11]
for l0 in lower_legs:
    for l1 in lower_legs:
        if (l1 > l0):
            enableCollision = 1
            print("collision for pair", l0, l1,
                  p.getJointInfo(quadruped, l0)[12],
                  p.getJointInfo(quadruped, l1)[12], "enabled=", enableCollision)
            p.setCollisionFilterPair(quadruped, quadruped, 2, 5, enableCollision)


maxUpForceId = p.addUserDebugParameter("maxUpForce", 0, 100, 20)

for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)

p.getCameraImage(480, 320)
p.setRealTimeSimulation(0)

joints = []

mocapData = motion_capture_data.MotionCaptureData()

motionPath = "laikago_walk.txt"

mocapData.Load(motionPath)
print("mocapData.NumFrames=", mocapData.NumFrames())
print("mocapData.KeyFrameDuraction=", mocapData.KeyFrameDuraction())
print("mocapData.getCycleTime=", mocapData.getCycleTime())
print("mocapData.computeCycleOffset=", mocapData.computeCycleOffset())

stablePD = pd_controller_stable.PDControllerStable(p)

cycleTime = mocapData.getCycleTime()






movements.stepUp1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.moveForward1(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
for _ in range(600):
    time.sleep(timeStep)
    p.stepSimulation()
movements.standUp(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)


########################################################################################################################

t = 0
count = 0
while t < 5. * cycleTime:
    # get interpolated joint
    keyFrameDuration = mocapData.KeyFrameDuraction()
    cycleTime = mocapData.getCycleTime()
    cycleCount = mocapData.calcCycleCount(t, cycleTime)

    # print("cycleTime=",cycleTime)
    # print("cycleCount=",cycleCount)

    # print("cycles=",cycles)
    frameTime = t - cycleCount * cycleTime
    # print("frameTime=",frameTime)
    if (frameTime < 0):
        frameTime += cycleTime

    frame = int(frameTime / keyFrameDuration)
    frameNext = frame + 1
    if (frameNext >= mocapData.NumFrames()):
        frameNext = frame
    frameFraction = (frameTime - frame * keyFrameDuration) / (keyFrameDuration)
    # print("frame=",frame)
    # print("frameFraction=",frameFraction)
    frameData = mocapData._motion_data['Frames'][frame]
    frameDataNext = mocapData._motion_data['Frames'][frameNext]

    jointsStr, qdot = qpi.Slerp(frameFraction, frameData, frameDataNext, p)

    maxForce = p.readUserDebugParameter(maxForceId)
    # print("jointIds=", jointIds)

    maxUpForce = p.readUserDebugParameter(maxUpForceId)
    p.changeConstraint(cid, maxForce=maxUpForce)
    jointsStr[7] = 0
    jointsStr[10] = 0
    jointsStr[13] = 0
    jointsStr[16] = 0
    if useConstraints:
        for j in range(12):
            # skip the base positional dofs
            targetPos = float(jointsStr[j + 7])
            p.setJointMotorControl2(quadruped,
                                    jointIds[j],
                                    p.POSITION_CONTROL,
                                    jointDirections[j] * targetPos + jointOffsets[j],
                                    force=maxForce)
        count += 1
        if count % 42 == 0:
            pos, ori = p.getBasePositionAndOrientation(quadruped)
            temp = []
            temp.append(startPos[0])
            temp.append(pos[1])
            temp.append(pos[2])
            p.resetBasePositionAndOrientation(quadruped, temp, startOrn)


    else:
        desiredPositions = []
        for j in range(7):
            targetPosUnmodified = float(jointsStr[j])
            desiredPositions.append(targetPosUnmodified)
        for j in range(12):
            targetPosUnmodified = float(jointsStr[j + 7])
            targetPos = jointDirections[j] * targetPosUnmodified + jointOffsets[j]
            desiredPositions.append(targetPos)
        for _ in range(5):
            desiredPositions.append(0)
        numBaseDofs = 6
        totalDofs = 17 + numBaseDofs
        desiredVelocities = None
        if desiredVelocities == None:
            desiredVelocities = [0] * totalDofs
        taus = stablePD.computePD(bodyUniqueId=quadruped,
                                  jointIndices=jointIds,
                                  desiredPositions=desiredPositions,
                                  desiredVelocities=desiredVelocities,
                                  kps=[4000] * totalDofs,
                                  kds=[40] * totalDofs,
                                  maxForces=[maxForce] * totalDofs,
                                  timeStep=timeStep)

        dofIndex = 6
        scaling = 1
        for index in range(len(jointIds)):
            jointIndex = jointIds[index]
            force = [scaling * taus[dofIndex]]
            # print("force[", jointIndex, "]=", force)
            p.setJointMotorControlMultiDof(quadruped,
                                           jointIndex,
                                           controlMode=p.TORQUE_CONTROL,
                                           force=force)
            dofIndex += 1


    p.stepSimulation()
    t += timeStep
    time.sleep(timeStep)


########################################################################################################################

movements.halfSit(p,timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.getDown(p,timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.standUp(p,timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.backLegDown(p,timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)

p.setRealTimeSimulation(1)



for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    js = p.getJointState(quadruped, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        paramIds.append(
            p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4,
                                    (js[0] - jointOffsets[j]) / jointDirections[j]))

while (1):
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        maxForce = p.readUserDebugParameter(maxForceId)
        p.setJointMotorControl2(quadruped,
                                jointIds[i],
                                p.POSITION_CONTROL,
                                jointDirections[i] * targetPos + jointOffsets[i],
                                force=maxForce)
