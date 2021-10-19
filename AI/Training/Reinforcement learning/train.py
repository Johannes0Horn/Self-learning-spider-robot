from BugAgent import BugAgent
ba=BugAgent(name="rs100.lr3e-4.nl6.efficiency",
            num_iterations=3000000,
            sel_agent="sac",
            env_name="robert",
            initial_collect_steps=100,
            num_threads=1,
            log_interval=5000,
            tfsummary_interval=5000,
            save_interval=100000,
            critic_learning_rate=3e-4,
            actor_learning_rate=3e-4,
            alpha_learning_rate=3e-4,
            num_eval_episodes=1,
            reward_scale_factor=100)

ba.train()