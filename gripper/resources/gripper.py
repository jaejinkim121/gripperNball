import pybullet as p
import os
import math


class Gripper:
    def __init__(self, client, initial_position):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'gripper_model.urdf')
        self.gripper = p.loadURDF(fileName=f_name,
                                  basePosition=[0, 0, 0.1],
                                  physicsClientId=client)
        self.input_joints = [0, 1]
        self.apply_action(initial_position)

    def apply_action(self, action):
        p_command_left, p_command_right = action
        p.setJointMotorControlArray(self.gripper, self.input_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[p_command_left, p_command_right],
                                    physicsClientId=self.client)

