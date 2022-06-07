import pybullet as p
import os
import math
from common import Config
import common


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

        # Get Joint Info
        for i in range(self.nJoints):
            self.jointInfo = p.getJointInfo(self.gripper, i)
            self.jointNameToID[self.jointInfo[1].decode('UTF-8')] = self.jointInfo[0]
            print('joint num {}: {}'.format(self.jointInfo[0], self.jointInfo[1]))

        # set contact parameters
        for i in range(self.nJoints):
            p.changeDynamics(self.gripper, i, lateralFriction=Config.MU)
            p.changeDynamics(self.gripper,
                             i,
                             contactStiffness=cal_Kc(self.gripper, i, Config.B_GRIPPER),
                             contactDamping=Config.B_GRIPPER)

        self.apply_action(initial_position)

    def apply_action(self, action):
        p_command_left, p_command_right = action
        p.setJointMotorControlArray(self.gripper, self.input_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[p_command_left, p_command_right],
                                    physicsClientId=self.client)

    def get_observation(self):
        pos, ang = p.getBasePositionAndOrientation(self.gripper, self.client)
        return 1.0


def cal_Kc(oID, jID, b):
    mass = p.getDynamicsInfo(oID, jID)[0]
    K_c = b * b / (4 * mass)
    return K_c
