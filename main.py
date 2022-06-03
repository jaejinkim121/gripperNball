import gym
import gripper
from stable_baselines3 import A2C

env = gym.make('gripper-v0')

model = A2C('MlpPolicy', env, verbose=1)

model.learn(total_timesteps=10000)

obs = env.reset()

