import pybullet as p1
from pybullet_utils import bullet_client
import pybullet_data
from pybullet_utils import pd_controller_stable

import time
import motion_capture_data
import quadrupedPoseInterpolator
import movements

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

startPos = [0.007058990464444105, 0.03149299192130908, 0.4918981912395484]
startOrn = [0.005934649695708604, 0.7065453990917289, 0.7076373820553712, -0.0027774940359030264]
quadruped = p.loadURDF("ModelData/LeggedRobot.urdf",
                       startPos,
                       startOrn,
                       flags=urdfFlags,
                       useFixedBase=False)
obstacle = p.loadURDF("ModelData/obstacle.urdf", useFixedBase=True)

dumbell = p.loadURDF("ModelData/squareDumbell.urdf", [-1,0,0.5], [0.707, 0, 0.707, 0])
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


movements.sitDown(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.grab(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)
movements.standUp(p, timeStep, maxForceId, quadruped, jointIds, jointDirections, jointOffsets)

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

p.setRealTimeSimulation(1)

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
