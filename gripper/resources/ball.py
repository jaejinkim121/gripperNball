import pybullet as p
import os
import math
from common import Config


class Ball:
    def __init__(self, client):
        self.client = client
        f_name = "sphere_small.urdf"
        self.ball = p.loadURDF(fileName=f_name,
                               basePosition=[0, 0, Config.OBJECT_HEIGHT],
                               globalScaling=Config.OBJECT_SCALE,
                               physicsClientId=client)
        p.changeDynamics(self.ball, -1, mass=Config.M_OBJECT)
        p.changeDynamics(self.ball, -1, lateralFriction=Config.MU)
        p.changeDynamics(self.ball,
                         -1,
                         contactStiffness=cal_Kc(self.ball, -1, Config.B_OBJECT),
                         contactDamping=Config.B_OBJECT)


def cal_Kc(oID, jID, b):
    mass = p.getDynamicsInfo(oID, jID)[0]
    K_c = b * b / (4 * mass)
    return K_c

