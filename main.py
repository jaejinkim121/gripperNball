import gym
import gripper
from stable_baselines3 import PPO

env = gym.make('gripper-v0')

model = PPO('MlpPolicy', env, verbose=0,
            n_epochs=20,
            batch_size=256,
            )

model.learn(total_timesteps=25_000,
            )


model.save("PPO_gripper")
del model

model = PPO.load("PPO_gripper")
obs = env.reset()

while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)


