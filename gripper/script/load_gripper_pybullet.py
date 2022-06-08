# yale open hand project - gripper model T urdf
import os
from re import X
import pybullet as p
import pybullet_data
import time
import math
import numpy
from pathlib import Path

# parameters
STABLE_MIN_FORCE = 1
STABLE_MAX_FORCE = 10

MU = 1
B_GRIPPER = 10.
B_OBJECT = 1000.
M_OBJECT = 0.01

INIT_POSE = 0.8  # gripper initial pose
OBJECT_HEIGHT = 0.15
OBJECT_SCALE = 0.7

CONSTRAINT_ERP = 0.00001
CONSTRAINT_MAX_FORCE = 0.1  # gripper soft constraint maximum force
GRIPPER_GEAR_RATIO = 0.7  # synergistic movement ratio at free motion

# pybullet setup
client = p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetDebugVisualizerCamera(cameraDistance=0.2,
                             cameraYaw=0,
                             cameraPitch=0,
                             cameraTargetPosition=[0.0, -0.15, 0.1])
num_iter = 1000
p.setPhysicsEngineParameter(numSolverIterations=num_iter)

# load plane
planeID = p.loadURDF("plane.urdf")

# load gripper
hand_file_name = "ohp_model_t_model/urdf/model_t_mesh.urdf"
handID = p.loadURDF(hand_file_name,
                    baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14]),
                    useFixedBase=True)

# load object
objectID = p.loadURDF("sphere_small.urdf",
                      [0, 0, OBJECT_HEIGHT],
                      globalScaling=OBJECT_SCALE)
p.changeDynamics(objectID, -1, mass=M_OBJECT)

# print joint info
nJoints = p.getNumJoints(handID)
jointNameToID = {}
for i in range(nJoints):
    jointInfo = p.getJointInfo(handID, i)
    jointNameToID[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    print('joint num {}: {}'.format(jointInfo[0], jointInfo[1]))

# calculate contact stiffness
def cal_Kc(oID, jID, b):
    mass = p.getDynamicsInfo(oID, jID)[0]
    K_c = b*b/(4*mass)
    return K_c


# set contact parameters
for i in range(nJoints):
    p.changeDynamics(handID, i, lateralFriction=MU)
    p.changeDynamics(handID,
                     i,
                     contactStiffness=cal_Kc(handID, i, B_GRIPPER),
                     contactDamping=B_GRIPPER)
p.changeDynamics(objectID, -1, lateralFriction=MU)
p.changeDynamics(objectID,
                 -1,
                 contactStiffness=cal_Kc(objectID, -1, B_OBJECT),
                 contactDamping=B_OBJECT)

# reset joint position before starting episode
def ResetJointPos(init_pose):
    PosInput(0, init_pose)
    PosInput(2, init_pose)
    return init_pose

# position control for joint 0, 2
def PosInput(jointID, desiredPos):
    if desiredPos < 0:
        desiredPos = 0
    p.setJointMotorControl2(handID,
                            jointID,
                            p.POSITION_CONTROL,
                            targetPosition=desiredPos)
    p.setJointMotorControl2(handID,
                            jointID+1,
                            p.POSITION_CONTROL,
                            targetPosition=desiredPos * GRIPPER_GEAR_RATIO,
                            force=CONSTRAINT_MAX_FORCE)

# count contact


def contact_count():
    contactPoints = p.getContactPoints()
    contact_num = len(contactPoints)
    return contact_num

# minimum contact force


def min_contact_force():
    num_contact = contact_count()
    if (num_contact < 2):
        return 0
    else:
        min_force = 1000
        contactPoints = p.getContactPoints()
        for i in range(len(contactPoints)):
            if (contactPoints[i][9] < min_force):
                min_force = contactPoints[i][9]
        return min_force

# maximum contact force


def max_contact_force():
    num_contact = contact_count()
    if (num_contact < 1):
        return 0
    else:
        max_force = 0
        contactPoints = p.getContactPoints()
        for i in range(len(contactPoints)):
            if (contactPoints[i][9] > max_force):
                max_force = contactPoints[i][9]
        return max_force

# close gripper


def gripper_close(pos_L, pos_R, dT, speed):
    new_L = pos_L + dT*speed
    new_R = pos_R + dT*speed
    PosInput(0, new_L)
    PosInput(2, new_R)
    return [new_L, new_R]

# open gripper


def gripper_open(pos_L, pos_R, dT, speed):
    new_L = pos_L - dT*speed
    new_R = pos_R - dT*speed
    PosInput(0, new_L)
    PosInput(2, new_R)
    return [new_L, new_R]

# move right


def gripper_right(pos_L, pos_R, dT, speed):
    new_L = pos_L + dT*speed
    new_R = pos_R - dT*speed
    PosInput(0, new_L)
    PosInput(2, new_R)
    return [new_L, new_R]

# heuristic method


def control_heuristic(pos_L, pos_R, dT, speed, episode_start):
    if (episode_start):
        gravity = True
    else:
        gravity = False

    if (min_contact_force() < STABLE_MIN_FORCE):
        [newL, newR] = gripper_close(pos_L, pos_R, dT, speed)
        # print("no contact")
    elif (max_contact_force() > STABLE_MAX_FORCE):
        [newL, newR] = gripper_open(pos_L, pos_R, dT, speed)
        # print("contact force too strong")
    else:
        if (~episode_start):
            p.setGravity(0, 0, -9.8)
            gravity = True
            # print("gravity on")
        [newL, newR] = gripper_right(pos_L, pos_R, dT, speed)

    return [gravity, newL, newR]

# print out gear ratio
def print_gear_ratio():
    q1_1 = p.getJointState(handID, 0)[0]
    q1_2 = p.getJointState(handID, 1)[0]
    q2_1 = p.getJointState(handID, 2)[0]
    q2_2 = p.getJointState(handID, 3)[0]
    if ((q1_1 != 0) & (q2_1 != 0)):
        r1 = round((q1_2/q1_1), 4)
        r2 = round((q2_2/q2_1), 4)
        print("joint angle ratio: L: {}, R: {}".format(r1, r2))

# get observations - object
def get_object_state():
    objState = p.getBasePositionAndOrientation(objectID)
    objVel = p.getBaseVelocity(objectID)
    pos = objState[0]
    ori = objState[1]
    vel = objVel[0]

    print('<object>  pos: {}, ori: {}, vel: {}'.format(pos, ori, vel))
    return [pos, ori, vel]

# get observations - gripper
def get_joint_state(joint_num):
    linkState = p.getLinkState(handID, joint_num, computeLinkVelocity=1)
    jointState = p.getJointState(handID, joint_num)

    pos = linkState[0] # 3 floats
    ori = linkState[1] # 4 floats - quaternion
    vel = linkState[6] # 3 floats
    jointAngle = jointState[0] # 1
    jointTorque = jointState[1] # 1 float

    #print('<gripper link{} >: pos: {}, ori: {}, vel: {}, angle: {}, torque: {}'
    #        .format(joint_num, pos, ori, vel, jointAngle, jointTorque))
    return [pos, ori, vel, jointAngle, jointTorque]

# timing
startTime = time.time()
lastTime = startTime
gravity_on = False

# initial pose
target_init = INIT_POSE

# (for testing)
closing_speed = 0.1

target_init = ResetJointPos(target_init)
targetL = target_init
targetR = target_init

# enable joint torque sensor
for i in range(nJoints):
    p.enableJointForceTorqueSensor(handID, i, enableSensor = 1)

while True:
    deltaTime = time.time() - lastTime
    lastTime = lastTime + deltaTime

    [gravity_on, targetL, targetR] = control_heuristic(targetL,
                                                       targetR,
                                                       deltaTime,
                                                       closing_speed,
                                                       gravity_on)

    # print_gear_ratio()
    get_object_state()
    for i in range(4):
        get_joint_state(i)

    p.stepSimulation()
