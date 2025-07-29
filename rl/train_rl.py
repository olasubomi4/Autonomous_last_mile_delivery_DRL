from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
import numpy as np
from dotenv import load_dotenv
import os
load_dotenv()
from rl.Agent import Agent

model_dir="models"
log_dir ="logs"

def train_TD3(env):
    n_actions = env.action_space.shape[0]
    train_params_name= os.environ.get("TRAIN_PARAMS_NAME","default")
    noise_value= float(os.environ.get("NOISE_VALUE",0.2))
    learning_rate= float(os.environ.get("LEARNING_RATE",0.0003))
    learning_starts=int(os.environ.get("LEARNING_STARTS",3000))
    train_freq= int(os.environ.get("TRAIN_FREQ",100))
    gradient_steps= int(os.environ.get("GRADIENT_STEPS",100))


    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=noise_value * np.ones(n_actions))
    td3 = TD3(
        "MlpPolicy", env,
        action_noise=action_noise,
        verbose=0,
        device='cpu',
        tensorboard_log=train_params_name+log_dir,
        learning_rate=learning_rate,
        learning_starts=learning_starts,
        train_freq=train_freq,
        gradient_steps=gradient_steps,
        # policy_kwargs=dict(net_arch=[256, 256])
    )

    timesteps = 100000
    iters = 0
    while True:
        iters += 1
        td3.learn(total_timesteps=timesteps, reset_num_timesteps=False)
        td3.save(f"{train_params_name+model_dir}/td3 _ {timesteps * iters}")

if __name__ == "__main__":
    train =True
    env = Agent()
    if train:
        train_TD3(env)
    else:
        pass
        # evaluate_agent(env)