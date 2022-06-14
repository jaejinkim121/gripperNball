import pybullet as p
import os
import math
from gripper.resources.common import Config
from gripper.resources import common
import numpy as np


class Gripper:
    def __init__(self, client, initial_position):
        self.client = client
        f_name = './gripper/resources/ohp_model_t_model/urdf/model_t_mesh.urdf'
        self.gripper = p.loadURDF(
            fileName=f_name,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14]),
            useFixedBase=True,
            physicsClientId=client)

        self.nJoints = p.getNumJoints(self.gripper)
        self.jointNameToID = {}
        self.input_joints = [0, 2]
        self.initial_pos = initial_position

        # set contact parameters
        for i in range(self.nJoints):
            p.changeDynamics(self.gripper, i, lateralFriction=Config.MU)
            p.changeDynamics(self.gripper,
                             i,
                             contactStiffness=common.cal_Kc(self.gripper, i, Config.B_GRIPPER),
                             contactDamping=Config.B_GRIPPER)
        self.reset_joint_pos()

    # Action : 2 target joint angle command.
    def apply_action(self, action):
        self.pos_input(self.input_joints[0], action[0])
        self.pos_input(self.input_joints[1], action[1])
        return

    def pos_input(self, joint_id, desired_pos):
        if desired_pos < 0:
            desired_pos = 0
        p.setJointMotorControl2(self.gripper,
                                joint_id,
                                p.POSITION_CONTROL,
                                targetPosition=desired_pos)
        p.setJointMotorControl2(self.gripper,
                                joint_id + 1,
                                p.POSITION_CONTROL,
                                targetPosition=desired_pos * Config.GRIPPER_GEAR_RATIO,
                                force=Config.CONSTRAINT_MAX_FORCE)

    def reset_joint_pos(self):
        self.apply_action([self.initial_pos, self.initial_pos])
        for _ in range(1000):
            p.stepSimulation()

            max_contact_force = common.get_max_contact_force()
            if max_contact_force > Config.STABLE_MIN_FORCE:
                break
        p.setGravity(0, 0, -9.8)

    def get_observation(self):
        joint_angle = [0, 0, 0, 0]
        for i in range(4):
            joint_angle[i] = self.get_joint_angle(i)
        return joint_angle

    def get_joint_state(self, joint_num):
        linkState = p.getLinkState(self.gripper, joint_num, computeLinkVelocity=1)
        jointState = p.getJointState(self.gripper, joint_num)

        pos = linkState[0]  # 3 floats
        ori = linkState[1]  # 4 floats - quaternion
        vel = linkState[6]  # 3 floats
        jointAngle = jointState[0]  # 1
        jointTorque = jointState[1]  # 1 float

        return [pos, ori, vel, jointAngle, jointTorque]

    def get_joint_angle(self, joint_num):
        joint_state = p.getJointState(self.gripper, joint_num)

        return joint_state[0]

