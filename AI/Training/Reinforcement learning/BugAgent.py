from __future__ import absolute_import, division, print_function
import functools
import base64
import imageio
import IPython
import matplotlib
import matplotlib.pyplot as plt
import PIL.Image
import gym
import gym_robert
#from tqdm import tqdm_notebook as tqdm
from tqdm import tqdm
import tensorflow as tf
from tf_agents.agents.dqn import dqn_agent
from tf_agents.drivers import dynamic_step_driver
from tf_agents.environments import suite_gym
from tf_agents.environments import tf_py_environment,parallel_py_environment
from tf_agents.eval import metric_utils
from tf_agents.metrics import tf_metrics
from tf_agents.networks import q_network,q_rnn_network
from tf_agents.policies import random_tf_policy,policy_saver
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.trajectories import trajectory
from tf_agents.utils import common
from tf_agents.policies import greedy_policy,py_epsilon_greedy_policy
from tf_agents.agents.ddpg import critic_network
from tf_agents.agents.sac import sac_agent
from tf_agents.drivers import dynamic_step_driver
from tf_agents.networks import actor_distribution_network
from tf_agents.networks import normal_projection_network
from tf_agents.environments import suite_pybullet
from datetime import datetime
from collections import deque
import numpy as np
import os

class LinearSchedule():
    def __init__(self, start_epsilon, final_epsilon, pre_train_steps, final_exploration_step):
        self.start_epsilon = start_epsilon
        self.final_epsilon = final_epsilon
        self.pre_train_steps = pre_train_steps
        self.final_exploration_step = final_exploration_step
    def value(self, t):
        if t < self.pre_train_steps:
            return  self.start_epsilon
        else:
            return self.start_epsilon - ( (self.start_epsilon-self.final_epsilon) * min( (t-self.pre_train_steps) / (self.final_exploration_step-self.pre_train_steps),1))

class BugAgent():
    def __init__(self,
                 ############################
                 # General
                 sel_agent="sac",
                 env_name="robert",
                 num_iterations = 1000000,
                 initial_collect_steps= 0,
                 collect_steps_per_iteration = 1,
                 replay_buffer_max_length = 1000000,
                 batch_size = 256,#256
                 tfsummary_interval=1000,
                 log_interval = 1000,
                 num_eval_episodes = 5,
                 save_interval = 10000, 
                 num_threads=1,#1
                 checkpoint_directory="__ckpt__",
                 name="",
                 res=[384,384],
                 ############################
                 # Qlearning
                 learning_rate = 1e-4,
                 fc_layer_params=(256,256),
                 lstm_size=(50,50),
                 ############################
                 # SAC
                 critic_learning_rate = 3e-4,#4
                 actor_learning_rate = 3e-4,#4
                 alpha_learning_rate = 3e-4,#4
                 target_update_tau = 0.005,
                 target_update_period = 1,
                 gamma = 0.99,
                 reward_scale_factor = 1.0,
                 gradient_clipping = None,
                 actor_fc_layer_params = (256, 256),
                 critic_joint_fc_layer_params = (256, 256),       
                 ):
        self.name=name
        logdir = "__logs__/scalars/" +datetime.now().strftime("%d.%m.%Y-%H:%M:%S")+"▮"+self.name
        self.file_writer = tf.summary.create_file_writer(logdir + "/metrics")
        #file_writer.set_as_default()
        
        ########################################################
        # General
        self.num_iterations=num_iterations
        self.initial_collect_steps=initial_collect_steps
        self.collect_steps_per_iteration=collect_steps_per_iteration
        self.replay_buffer_max_length=replay_buffer_max_length
        self.batch_size=batch_size
        self.log_interval=log_interval
        self.tfsummary_interval=tfsummary_interval
        self.num_eval_episodes=num_eval_episodes
        self.save_interval=save_interval
        self.num_threads=num_threads
        self.checkpoint_directory=checkpoint_directory+"/"+self.name
        self.checkpoint_prefix = os.path.join(self.checkpoint_directory, "ckpt")
        self.res=res
        ########################################################
        # Qlearning
        self.learning_rate=learning_rate
        self.fc_layer_params=fc_layer_params
        self.lstm_size=lstm_size
        ########################################################
        # SAC
        self.critic_learning_rate=critic_learning_rate
        self.actor_learning_rate=actor_learning_rate
        self.alpha_learning_rate=alpha_learning_rate
        self.target_update_tau=target_update_tau
        self.target_update_period=target_update_period
        self.gamma=gamma
        self.reward_scale_factor=reward_scale_factor
        self.gradient_clipping=gradient_clipping
        self.actor_fc_layer_params=actor_fc_layer_params
        self.critic_joint_fc_layer_params=critic_joint_fc_layer_params
        ########################################################
        # Environment
        if env_name=="robert":
            self.eval_py_env=suite_gym.wrap_env(gym.make('robert-v0',res=self.res))
            self.eval_env = tf_py_environment.TFPyEnvironment(self.eval_py_env)
            def createenv():
                return suite_gym.wrap_env(gym.make('robert-v0',res=self.res))
            if num_threads>1:
                print("starting "+str(num_threads) +" parallel envs")
                self.train_env = tf_py_environment.TFPyEnvironment(
                    parallel_py_environment.ParallelPyEnvironment([createenv for _ in range(self.num_threads)]))
            else:
                self.train_env=tf_py_environment.TFPyEnvironment(suite_gym.wrap_env(gym.make('robert-v0',res=self.res)))
        elif env_name=="MinitaurBulletEnv-v0":
            self.train_py_env = suite_pybullet.load(env_name)
            self.eval_py_env = suite_pybullet.load(env_name)
            self.eval_env = tf_py_environment.TFPyEnvironment(self.eval_py_env)
            def createenv():
                return suite_pybullet.load(env_name)
            if num_threads>1:
                print("starting "+str(num_threads) +" parallel envs")
                self.train_env = tf_py_environment.TFPyEnvironment(
                    parallel_py_environment.ParallelPyEnvironment([createenv for _ in range(self.num_threads)]))
            else:
                self.train_env= tf_py_environment.TFPyEnvironment(self.train_py_env)
            
        ########################################################
        # Agent 
        if sel_agent=="q":
            self.q_net=self.__build_qnet()
            self.agent=self.__build_DqnAgent()
            self.agent.initialize()
            self.checkpoint = tf.train.Checkpoint(q_net=self.q_net)

        elif sel_agent=="sac":
            global_step = tf.compat.v1.train.get_or_create_global_step()
            with tf.compat.v2.summary.record_if(lambda: tf.math.equal(global_step % 1000, 0)):
                self.critic_net=self.__build_critic_net()
                self.actor_net=self.__build_actor_net()
                self.agent=self.__build_SACAgent(global_step)
                self.agent.initialize()
                self.checkpoint = tf.train.Checkpoint(actor_net=self.actor_net, critic_net=self.critic_net)
            
        status = self.checkpoint.restore(tf.train.latest_checkpoint(self.checkpoint_directory))

        ########################################################
        # Policy 
        self.random_policy = random_tf_policy.RandomTFPolicy(self.train_env.time_step_spec(),
                                                             self.train_env.action_spec())
        ########################################################
        # Buffer 
        self.replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(data_spec=self.agent.collect_data_spec,                                                                      batch_size=self.train_env.batch_size, max_length=self.replay_buffer_max_length)
        
        ########################################################
        # collect 
        #self.__collect_data(self.train_env, self.random_policy, self.replay_buffer, steps=int(self.initial_collect_steps/self.num_threads))
        # Collect with driver:
        initial_collect_driver = dynamic_step_driver.DynamicStepDriver(self.train_env,
                                                                       self.agent.collect_policy,
                                                                       observers=[self.replay_buffer.add_batch],
                                                                       num_steps=self.initial_collect_steps)
        
        initial_collect_driver.run()
        
        self.dataset = self.replay_buffer.as_dataset(num_parallel_calls=3, 
                                                     sample_batch_size=self.batch_size, 
                                                     num_steps=2).prefetch(3)
        self.dataset_iterator = iter(self.dataset)
        
    def __build_qnet(self):
        q_net=q_network.QNetwork(self.train_env.observation_spec(),
                                 self.train_env.action_spec(),
                                 fc_layer_params=self.fc_layer_params)
        return q_net
    
    def __build_critic_net(self):
        critic_net = critic_network.CriticNetwork((self.train_env.observation_spec(),
                                                   self.train_env.action_spec()),
                                                  observation_fc_layer_params=None,
                                                  action_fc_layer_params=None,
                                                  joint_fc_layer_params=self.critic_joint_fc_layer_params)
        return critic_net
    
    def __build_actor_net(self):
        
        def normal_projection_net(init_means_output_factor=0.1):
            return normal_projection_network.NormalProjectionNetwork(
                self.train_env.action_spec(),
                mean_transform=None,
                state_dependent_std=True,
                std_transform=sac_agent.std_clip_transform,
                scale_distribution=True)
        
        actor_net = actor_distribution_network.ActorDistributionNetwork(self.train_env.observation_spec(),
                                                                        self.train_env.action_spec(),
                                                                        fc_layer_params=self.actor_fc_layer_params,
                                                                        continuous_projection_net=normal_projection_net)
        
        return actor_net
    
    
    def __build_DqnAgent(self):
        optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=self.learning_rate)
        train_step_counter = tf.Variable(0)
        agent = dqn_agent.DqnAgent(self.train_env.time_step_spec(),
                                   self.train_env.action_spec(),
                                   q_network=self.q_net,
                                   optimizer=optimizer,
                                   td_errors_loss_fn=common.element_wise_squared_loss,
                                   train_step_counter=train_step_counter,
                                   epsilon_greedy=0)
        return agent
    
    def __build_SACAgent(self,global_step):
        agent = sac_agent.SacAgent(self.train_env.time_step_spec(),
                                   self.train_env.action_spec(),
                                   actor_network=self.actor_net,
                                   critic_network=self.critic_net,
                                   actor_optimizer=tf.compat.v1.train.AdamOptimizer(
                                       learning_rate=self.actor_learning_rate),
                                   critic_optimizer=tf.compat.v1.train.AdamOptimizer(
                                       learning_rate=self.critic_learning_rate),
                                   alpha_optimizer=tf.compat.v1.train.AdamOptimizer(
                                       learning_rate=self.alpha_learning_rate),
                                   target_update_tau=self.target_update_tau,
                                   target_update_period=self.target_update_period,
                                   td_errors_loss_fn=tf.compat.v1.losses.mean_squared_error,
                                   gamma=self.gamma,
                                   reward_scale_factor=self.reward_scale_factor,
                                   gradient_clipping=self.gradient_clipping,
                                   train_step_counter=global_step,
                                   summarize_grads_and_vars=False,
                                   debug_summaries=False
                                  )
        
        return agent
    def compute_avg_return(self,environment, policy, num_episodes=2):
        total_return = 0.0
        for _ in tqdm(range(num_episodes),desc="compute_avg_return: "):
            time_step = environment.reset()
            episode_return = 0.0
            while not time_step.is_last():
                action_step = policy.action(time_step)
                time_step = environment.step(action_step.action)
                episode_return += time_step.reward
            total_return += episode_return

        avg_return = total_return / num_episodes
        return avg_return.numpy()[0]
    
    def collect_step(self,environment, policy, buffer):
        
        time_step = environment.current_time_step()
        action_step = policy.action(time_step)
        next_time_step = environment.step(action_step.action)
        traj = trajectory.from_transition(time_step, action_step, next_time_step)
        buffer.add_batch(traj)
        
    def __collect_data(self,env, policy, buffer, steps):
        pbar = tqdm(total=steps*self.num_threads,desc="Fill Buffer with "+str(type(policy)).split(".")[-1].rstrip("'>")+": ")
        for _ in range(steps):
            pbar.update(self.num_threads)
            self.collect_step(env, policy, buffer)

        
    def train(self):
        maxreward=0
        collect_driver = dynamic_step_driver.DynamicStepDriver(
            self.train_env,
            self.agent.collect_policy,
            observers=[self.replay_buffer.add_batch],
            num_steps=self.collect_steps_per_iteration)
        #schedule = LinearSchedule(0.8, 0.1, 1000, 50000) 
        saver=policy_saver.PolicySaver(self.agent.policy)
        # (Optional) Optimize by wrapping some of the code in a graph using TF function.
        self.agent.train = common.function(self.agent.train)
        collect_driver.run = common.function(collect_driver.run)
        # Reset the train step
        self.agent.train_step_counter.assign(0)
        # Evaluate the agent's policy once before training.
        avg_return = self.compute_avg_return(self.eval_env, self.agent.policy, self.num_eval_episodes)
        print('step = {0}: Average Return = {1}'.format(0, avg_return))
        self.returns = [avg_return]
        pbar = tqdm(total=self.num_iterations*self.collect_steps_per_iteration,desc="Training:")
        for i in range(self.num_iterations):
            pbar.update(1)
            
            # Collect a few steps using collect_policy and save to the replay buffer.
            for j in range(self.collect_steps_per_iteration):
                collect_driver.run() ## returns reward and observation... useful for tensorboard logging TODO

            experience, unused_info = next(self.dataset_iterator)
            train_loss = self.agent.train(experience)
            step = self.agent.train_step_counter.numpy()
            
            if step % self.tfsummary_interval ==0:
                #tf.summary.scalar('reward', data=np.mean(experience.reward.numpy()), step=step)
                avg_return = self.compute_avg_return(self.eval_env, greedy_policy.GreedyPolicy(self.agent.policy),1)
                
                
                if avg_return>maxreward:
                    maxreward=avg_return
                    saver.save("__mods__/" + self.name)
                    
                with self.file_writer.as_default():
                    tf.summary.scalar('reward', data=float(avg_return), step=step)
                    tf.summary.scalar('maxreward', data=float(maxreward), step=step)
                
            #print("step: ",step)     
            if step % self.log_interval ==0:
                print(self.name+'▮ step = {0}: loss = {1}: reward = {2}'.format(step, train_loss.loss,np.mean(experience.reward.numpy())))#,schedule.value(i+self.collect_steps_per_iteration))
                
            if step % self.save_interval ==0:
                print("Saving ...")
                self.create_policy_eval_video(greedy_policy.GreedyPolicy(self.agent.policy),
                                             self.eval_env,
                                             self.eval_py_env,
                                             self.name+"-"+str(step))
                self.checkpoint.save(file_prefix=self.checkpoint_prefix)
                #saver.save("__mods__/" + self.name)

    def compute_avg_return_actionlist(self,environment, policy, num_episodes=1):
        total_return = 0.0
        actionlist=[]
        for _ in range(num_episodes):
            time_step = environment.reset()
            episode_return = 0.0
            while not time_step.is_last():
                action_step = policy.action(time_step)
                actionlist.append(action_step.action)
                time_step = environment.step(action_step.action)
                episode_return += time_step.reward
            total_return += episode_return

        avg_return = total_return / num_episodes
        return avg_return.numpy()[0],actionlist
    
    
    def render_from_model(self,name):
        saved_policy = tf.compat.v2.saved_model.load("__mods__/" + name)
        avg_return=0
        #actionlist=[]
        #for _ in tqdm(range(1)):
        #    temp_avg_return,temp_actionlist = self.compute_avg_return_actionlist(self.eval_env, saved_policy, 1)
        #    if temp_avg_return>avg_return:
        #        avg_return=temp_avg_return
        #        actionlist=temp_actionlist
        #        print(name,avg_return)
                
        #self.create_video_from_actionlist(actionlist,self.eval_env,self.eval_py_env,"1_"+name+"▮"+str(avg_return))
        #self.create_video_from_actionlist(actionlist,self.eval_env,self.eval_py_env,"2_"+name+"▮"+str(avg_return))
        actionlist=np.array(self.create_policy_eval_video(saved_policy,self.eval_env,self.eval_py_env,name+"▮"))
        #actionlistcsv=[]
        #for x in actionlist:
        #    actionlistcsv.append(x[0])
        #np.savetxt(name+"▮"+str(avg_return)+".csv", actionlistcsv, delimiter=",")
        np.savetxt(name+"▮.csv", actionlist, delimiter=",")
    def create_policy_eval_video(self,policy,env,eval_py_env, filename, num_episodes=1, fps=30):
        filename = "__vids__/"+filename + ".mp4"
        actionlist=[]
        with imageio.get_writer(filename, fps=fps) as video:
            for _ in range(num_episodes):
                time_step = env.reset()
                for frame in eval_py_env.render():
                    #print(frame)
                    video.append_data((frame[:,:,:3]*255).astype('uint8'))
                
                #while not time_step.is_last():
                for i in tqdm(range(50),desc="Render Video"):
                    action_step = policy.action(time_step)
                    actionlist.append(action_step.action[0])
                    #print(action_step)
                    time_step = env.step(action_step.action)
                    for frame in eval_py_env.render():
                        video.append_data((frame[:,:,:3]*255).astype('uint8'))
        return actionlist
    
    def create_video_from_actionlist(self,actionlist,env,eval_py_env, filename):
        filename = "__vids__/"+filename + ".mp4"
        with imageio.get_writer(filename, fps=30) as video:
            time_step = env.reset()
            for frame in eval_py_env.render():
                video.append_data((frame[:,:,:3]*255).astype('uint8'))
            for action in tqdm(actionlist[:200],desc="Render Video"):
                time_step = env.step(action)
                for frame in eval_py_env.render():
                    video.append_data((frame[:,:,:3]*255).astype('uint8'))
        return actionlist
                        
