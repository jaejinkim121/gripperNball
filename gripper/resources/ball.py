import pybullet as p
import os
import math


class Ball:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'ball.urdf')
        self.ball = p.loadURDF(fileName=f_name,
                               basePosition=[0, 0, 0.1],
                               physicsClientId=client)
