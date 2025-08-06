import time
from datetime import datetime
from typing import Optional, Union, List

import gym
from gym import spaces
from gym.core import RenderFrame
import numpy as np
from stable_baselines3 import TD3, SAC
from stable_baselines3.common.noise import NormalActionNoise

from rl.SImulation_rl import Simulation_rl
import os
from rl.Controllers import Controllers

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
        self.observation_space = spaces.box.Box(low=-1, high=1, shape=(16,), dtype=np.float32)
        self.previous_state= None
        self.log_file=  os.environ.get("TRAIN_PARAMS_NAME","default")+"_resetting/resetting_logs.txt"
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

    def step(self,action):
        self.simulation.draw()
        steering_action=action[0]
        acceleration_action=action[1]
        self.simulation.move(steering_action,acceleration_action)
        state=self.simulation.get_state()
        scaled_state_value=state.scale_state_values()
        info={}
        reward =self.simulation.get_reward(self.previous_state,state)
        self.previous_state=state
        done =self.simulation.is_finished()
        if self.simulation.does_car_collide_with_obstacle():
            self.simulation.reset_car()
            reward =-1
            done=True
        elif self.simulation.does_car_collide_with_border():
            self.simulation.reset_car()
            reward =-1
            done = True
        elif self.simulation.has_reached_destination():
            print("Reached destination")
            info["completed_a_delivery"] = True
            self.simulation.mark_delivery_completed()
            if self.simulation.is_finished():
                info["successfully_completed_deliveries"]=True
                self.simulation.are_all_deliveries_completed_flag=True
                # self.previous_state = None
                # self.simulation.reset()
                done=True
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                message = f"[{timestamp}] resetting environment"
                print(message)
                with open(self.log_file,"a+") as f:
                    f.write(message+"\n")
            else:
                self.simulation.next_delivery()
            reward=1


            # done=True
        # elif self.simulation.

        # reward=np.random.uniform(-1,1)
        return scaled_state_value,reward,done,info

    def reset(self):
        self.previous_state = None
        self.simulation.reset()

        state= self.simulation.get_state().scale_state_values()
        return state

    def render(self) -> Optional[Union[RenderFrame, List[RenderFrame]]]:
        pass

    def mark_all_deliveries_as_completed(self):
        self.simulation.are_all_deliveries_completed_flag=True


def train_TD3(env):
    n_actions = env.action_space.shape[0]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.2 * np.ones(n_actions))

    td3 = TD3(
        "MlpPolicy", env,
        action_noise=action_noise,
        verbose=1,
        device='cpu',
        tensorboard_log=log_dir,
        learning_rate=0.00001,
        learning_starts=2000,
        train_freq=50,
        gradient_steps=50,
    )
    # tensorboard - -logdir
    timesteps=100000
    iters=0
    while True:
        iters +=1
        td3.learn(total_timesteps=timesteps,reset_num_timesteps=False)
        td3.save(f"{model_dir}/td3 _ {timesteps*iters}")
# td3 _ 30000.zip
def evaluate_agent(env, model_path="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/train_rl_bc_sac_imitationrl/models/sac _ 600000.zip"):
    model = SAC.load(model_path, env=env)
    state = env.reset()
    episodes=400
    done = False
    successful_deliveries=0
    successful_delivery_sequences=0
    controller = Controllers()
    while episodes>0:
        reward_sum=0
        steps=0
        # time.sleep(5)
        start_time=time.time()
        while not done:

            supervisor_action=controller.get_supervisors_action()
            # action=controller.get_manual_input()

            if supervisor_action is not None:
                action = supervisor_action
            else:
                action, _ = model.predict(state)

            state, reward, done, info = env.step(action)
            reward_sum+=reward
            if info.get("completed_a_delivery", False):
                successful_deliveries += 1
            if info.get("successfully_completed_deliveries", False):
                successful_delivery_sequences += 1
            if steps%15==0:
                pass
                # print(f"This reward {reward}")
                # print(f"Reward for this episode: {reward_sum}")
            steps+=1
        episodes-=1
        done = False
        env.mark_all_deliveries_as_completed()
        print(f"Reward for this episode: {reward_sum}")
        end_time=time.time()
        print(f"Time taken for this episode: {end_time-start_time}")
        state=env.reset()
    print(f"Sucessful deliveries: {successful_deliveries}")
    print(f"Sucessful delivery sequences: {successful_delivery_sequences}")





if __name__ == "__main__":
    train =False
    env = Agent()
    if train:
        train_TD3(env)
    else:
        evaluate_agent(env)
