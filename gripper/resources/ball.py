import pybullet as p
import os
import math


class Ball:
    def __init__(self, client):
        self.client = client
        f_name = './gripper/resources/gripper_model.urdf'
        self.ball = p.loadURDF(fileName=f_name,
                               basePosition=[0, 0, 2],
                               physicsClientId=client)
