import pybullet as p
import pybullet_data
import os
import math
from gripper.resources.common import Config
from gripper.resources import common
import random


class Ball:
    def __init__(self, client):
        self.client = client
        f_name = './gripper/resources/sphere_small.urdf'
        self.ball = \
            p.loadURDF(fileName=f_name,
                       basePosition=[
                           (0.5 - random.random())*Config.OBJECT_PERTURBATION,
                           0,
                           Config.OBJECT_HEIGHT
                       ],
                       globalScaling=Config.OBJECT_SCALE,
                       physicsClientId=client)
        p.changeDynamics(self.ball, -1, mass=Config.M_OBJECT)
        p.changeDynamics(self.ball, -1, lateralFriction=Config.MU)
        p.changeDynamics(self.ball,
                         -1,
                         contactStiffness=common.cal_Kc(self.ball, -1, Config.B_OBJECT),
                         contactDamping=Config.B_OBJECT)

    # Get all of object states
    def get_object_state(self):
        objState = p.getBasePositionAndOrientation(self.ball)
        objVel = p.getBaseVelocity(self.ball)
        pos = objState[0]
        ori = objState[1]
        vel = objVel[0]

        return [pos, ori, vel]

    # Get object orientation only
    def get_observation(self):
        obj_state = p.getBasePositionAndOrientation(self.ball)
        joint_angle = 2 * math.acos(obj_state[1][3])

        return [joint_angle]
