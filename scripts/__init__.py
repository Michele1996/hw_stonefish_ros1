from gym.envs.registration import register

register(
    id='CustomEnv-v0',
    entry_point='ros_env.custom_env:CustomEnv',
)