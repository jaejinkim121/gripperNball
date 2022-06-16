import gym
import gripper
from stable_baselines3 import PPO
from gripper.resources.common import Config

env = gym.make('gripper-v0')

model = PPO('MlpPolicy', env, verbose=0,
            n_epochs=Config.N_EPOCHS,
            batch_size=Config.BATCH_SIZE,
            )

model.learn(total_timesteps=Config.TOTAL_TIMESTEPS,
            )
print("Training End")
print("Type '1' to test learned model")
input()
model.save("PPO_gripper_positionGoal")
del model

model = PPO.load("PPO_gripper_positionGoal")
obs = env.reset()

while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    if dones:
        break
