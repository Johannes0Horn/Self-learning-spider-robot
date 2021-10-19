from gym.envs.registration import register


register(
    id='robert-v0', #passed to gym.make()
    entry_point='gym_robert.envs:RobertEnv',
    max_episode_steps=500,
    reward_threshold=5,
)