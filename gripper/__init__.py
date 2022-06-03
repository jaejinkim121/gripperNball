from gym.envs.registration import register
register(
    id='gripper-v0',
    entry_point='gripper.envs:GripperEnv'
)