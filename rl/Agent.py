from typing import Optional, Union, List

import gym
from gym import spaces
from gym.core import RenderFrame
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
        self.observation_space = spaces.box.Box(low=-1, high=1, shape=(10,), dtype=np.float32)
        self.previous_state= None

    def step(self,action):
        self.simulation.draw()
        steering_action=action[0]
        acceleration_action=action[1]
        self.simulation.move(steering_action,acceleration_action)
        state=self.simulation.get_state()
        scaled_state_value=state.scale_state_values()

        reward =self.simulation.get_reward(self.previous_state,state)
        self.previous_state=state
        done =self.simulation.is_finished()
        if self.simulation.does_car_collide_with_obstacle():
            self.simulation.reset_car()
            reward =0
            done=True
        elif self.simulation.does_car_collide_with_border():
            self.simulation.reset_car()
            reward =0
            done = True
        elif self.simulation.has_reached_destination():
            print("Reached destination")
            self.simulation.reset_car()
            reward=1
            done=True
        # elif self.simulation.

        # reward=np.random.uniform(-1,1)
        return scaled_state_value,reward,done,{}

    def reset(self):
        self.previous_state = None
        self.simulation.reset()
        state= self.simulation.get_state().scale_state_values()
        return state

    def render(self) -> Optional[Union[RenderFrame, List[RenderFrame]]]:
        pass


def train_TD3(env):
    n_actions = env.action_space.shape[0]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.2 * np.ones(n_actions))
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