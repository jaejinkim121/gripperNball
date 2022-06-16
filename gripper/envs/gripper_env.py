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
            low=np.array(
                [-Config.Input_perturbation, -Config.Input_perturbation],
                dtype=np.float32),
            high=np.array(
                [Config.Input_perturbation, Config.Input_perturbation],
                dtype=np.float32)
        )
        self.observation_space = gym.spaces.Box(
            low=np.array([0, 0, 0, 0, 0, 0, 0, -TPI/2],
                         dtype=np.float32),
            high=np.array([TPI, TPI, TPI, TPI, 1000, 1000, 0.2, TPI/2],
                          dtype=np.float32)
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        if Config.IS_GUI:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.resetDebugVisualizerCamera(cameraDistance=0.2,
                                     cameraYaw=0,
                                     cameraPitch=0,
                                     cameraTargetPosition=[0, -0.15, 0.1])
        p.setTimeStep(1/200, self.client)

        self.current_input = [Config.INIT_POSE, Config.INIT_POSE]
        self.current_step = 0
        self.total_step = 0

        self.gripper = None
        self.ball = None
        self.plane = None
        self.done = False
        self.goal = None
        self.contact = False
        self.reward_coeff = [Config.REWARD_ACHIEVE_GOAL,
                             Config.PENALTY_GOAL_DIST,
                             Config.PENALTY_LOST_OBJECT,
                             Config.PENALTY_OVER_GRASPING,
                             Config.PENALTY_OVER_GRASPING_HARD]
        self.reset()

    def step(self, action):
        self.current_step += 1
        object_lost = False

        if self.current_step == Config.MAX_STEP_SINGLE_EPISODE:
            self.done = True

        action[0] = max(min(action[0] + self.current_input[0], TPI), 0)
        action[1] = max(min(action[1] + self.current_input[1], TPI), 0)
        self.current_input[0] = action[0]
        self.current_input[1] = action[1]
        self.gripper.apply_action(action)
        p.stepSimulation()
        ob_gripper = self.gripper.get_observation()
        ob_ball = self.ball.get_observation()
        contact_force = \
            [common.contact_force_link(
                self.gripper.gripper,
                self.ball.ball,
                0
            ),
                common.contact_force_link(
                self.gripper.gripper,
                self.ball.ball,
                1
            ),
                common.contact_force_link(
                self.gripper.gripper,
                self.ball.ball,
                2
            ),
                common.contact_force_link(
                self.gripper.gripper,
                self.ball.ball,
                3
            )]

        ob_contact_force = [contact_force[1], contact_force[3]]

        ob = ob_gripper + ob_contact_force + ob_ball

        # Penalty for target orientation difference
        reward = self.reward_coeff[1] * abs(self.goal - ob_ball[1])

        # Penalty for over-grasping
        if ob_contact_force[0] > Config.STABLE_MAX_FORCE:
            reward += self.reward_coeff[3] * \
                (ob_contact_force[0] - Config.STABLE_MAX_FORCE)
        if ob_contact_force[1] > Config.STABLE_MAX_FORCE:
            reward += self.reward_coeff[3] * \
                (ob_contact_force[1] - Config.STABLE_MAX_FORCE)

        # Penalty for object lost
        if (ob_ball[0] > 0.2) or (ob_ball[0] < 0.08):
            self.done = True
            object_lost = True

        # youngseon lee edited
        if (contact_force[1] == 0 or contact_force[3] == 0):
            self.done = True
            object_lost = True

        if max(contact_force) > Config.HARD_MAX_FORCE:
            self.done = True
            print("END Condition - Too high contact force")
            reward = self.reward_coeff[4]

        if object_lost:
            print("END Condition - Object Out of range")
            reward = self.reward_coeff[2]

        if reward > -1E-6:
            self.done = True
            reward = self.reward_coeff[0]
            print("END Condition - Objective Clear")

        return ob, reward, self.done, dict()

    def reset(self):
        self.total_step = self.total_step+self.current_step
        self.current_step = 0
        self.contact = False
        p.resetSimulation(self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=Config.NUM_ITER)
        self.done = False
        self.plane = p.loadURDF("./gripper/resources/plane.urdf")
        self.ball = Ball(client=self.client)
        self.gripper = Gripper(client=self.client,
                               initial_position=Config.INIT_POSE)

        self.current_input = [Config.INIT_POSE, Config.INIT_POSE]

        # self.goal = self.np_random.uniform(-math.pi * Config.GOAL_MAX,
        #                                    math.pi * Config.GOAL_MAX)
        

        ob_gripper = self.gripper.get_observation()
        ob_ball = self.ball.get_observation()
        self.goal = -math.pi/10
        ob_contact_force = \
            [common.contact_force_link(
                self.gripper.gripper,
                self.ball.ball,
                1

            ),
                common.contact_force_link(
                    self.gripper.gripper,
                    self.ball.ball,
                    3
            )]
        ob = ob_gripper + ob_contact_force + ob_ball
        print("[{}/{}]: Current trial's goal = {}, Current ball ori = {}".format(
            self.total_step, Config.TOTAL_TIMESTEPS,self.goal, ob_ball[1]), end="\r")
        return ob

    def close(self):
        p.disconnect(self.client)

    def render(self):
        pass

    def seed(self, seed=None):
        pass
