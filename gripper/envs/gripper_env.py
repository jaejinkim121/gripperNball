import gym
import numpy as np
import math
import pybullet as p
import matplotlib.pyplot as plt
from gripper.resources.gripper import Gripper
from gripper.resources.ball import Ball


class GripperEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'],
                'video.frames_per_second': 50}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([0, 0], dtype=np.float32),
            high=np.array([1, 1], dtype=np.float32)
        )
        self.observation_space = gym.spaces.Box(
            low=0,
            high=2 * math.pi,
            shape=(1, )
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)
        p.setTimeStep(1/200, self.client)

        self.gripper = None
        self.ball = None
        self.done = False
        self.goal = None

        self.reward_coeff = 1

        self.reset()

    def step(self, action):
        self.gripper.apply_action(action)
        p.stepSimulation()
        ob = self.gripper.get_observation()

        reward = -self.reward_coeff * (self.goal - ob) ** 2

        self.done = True

        return ob, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        initial_position = [1.0, 2.0]
        self.done = False

        self.gripper = Gripper(client=self.client,
                               initial_position=initial_position)
        self.ball = Ball(client=self.client)
        self.goal = self.np_random.uniform(0, 2 * math.pi)

        return np.array(0)

    def close(self):
        p.disconnect(self.client)

    def render(self):
        pass

    def seed(self, seed=None):
        pass
