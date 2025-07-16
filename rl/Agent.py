import gym
from gym import spaces
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from rl.SImulation_rl import Simulation_rl
import os
log_dir ="logs"
model_dir ="models"

os.makedirs(log_dir,exist_ok=True)
os.makedirs(model_dir,exist_ok=True)

class Agent (gym.Env):
    def __init__(self):
        super(Agent, self).__init__()
        self.simulation= Simulation_rl()
        self.simulation._init_simulation()
        self.action_space = spaces.box.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.box.Box(low=-1, high=1, shape=(6,), dtype=np.float32)

    def step(self,action):
        self.simulation.draw()
        steering_action=action[0]
        acceleration_action=action[1]
        self.simulation.move(steering_action,acceleration_action)
        state=self.simulation.get_state()
        done =self.simulation.is_finished()
        reward=np.random.uniform(-1,1)
        return state,reward,done,{}

    def reset(self):
        self.simulation.reset()

    def render(self, mode='human'):
        self.simulation.draw()


def train_TD3(env):

    n_actions = env.action_space.shape[0]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.006 * np.ones(n_actions))
    td3 = TD3(
        "MlpPolicy", env,
        action_noise=action_noise,
        verbose=1,
        device='cpu',
        tensorboard_log=log_dir,
        learning_starts=1000,
        train_freq=100,
        gradient_steps=100,
    )

    timesteps=100000
    iters=0
    while True:
        iters +=1
        td3.learn(total_timesteps=timesteps,reset_num_timesteps=False)
        td3.save(f"{model_dir}/td3 _ {timesteps*iters}")


if __name__ == "__main__":
    env = Agent()
    train_TD3(env)