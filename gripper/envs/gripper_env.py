import gym
import numpy as np
import math
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from gripper.resources.gripper import Gripper
from gripper.resources.ball import Ball
from gripper.resources.common import Config
from gripper.resources import common

TPI = 2 * math.pi

class GripperEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'],
                'video.frames_per_second': 50}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([0, 0], dtype=np.float32),
            high=np.array([TPI, TPI], dtype=np.float32)
        )
        self.observation_space = gym.spaces.Box(
            low=np.array([0, 0, 0, 0, 0, 0], dtype=np.float32),
            high=np.array([TPI, TPI, TPI, TPI, 2E5, TPI], dtype=np.float32)
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)
        p.resetDebugVisualizerCamera(cameraDistance=0.2,
                                     cameraYaw=0,
                                     cameraPitch=0,
                                     cameraTargetPosition=[0.0, -0.15, 0.1])
        p.setTimeStep(1/200, self.client)

        self.gripper = None
        self.ball = None
        self.plane = None
        self.done = False
        self.goal = None

        self.reward_coeff = [1, 1]

        self.reset()

    def step(self, action):
        self.gripper.apply_action(action)
        p.stepSimulation()
        ob_gripper = self.gripper.get_observation()
        ob_ball = self.ball.get_observation()
        ob_contact_force = [common.get_max_contact_force()]
        ob = ob_gripper + ob_contact_force + ob_ball
        reward = - self.reward_coeff[0] * (self.goal - ob[5]) ** 2 \
                 - self.reward_coeff[1] * ob[4]
        # Early Ending Condition Setup
        if reward > -1:
            self.done = True
        return ob, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=Config.NUM_ITER)
        self.done = False
        self.plane = p.loadURDF("./gripper/resources/plane.urdf")
        self.ball = Ball(client=self.client)
        self.gripper = Gripper(client=self.client,
                               initial_position=Config.INIT_POSE)
        self.goal = self.np_random.uniform(0, 2 * math.pi)
        ob_gripper = self.gripper.get_observation()
        ob_ball = self.ball.get_observation()
        ob_contact_force = [common.get_max_contact_force()]
        ob = ob_gripper + ob_contact_force + ob_ball

        return ob

    def close(self):
        p.disconnect(self.client)

    def render(self):
        pass

    def seed(self, seed=None):
        pass
