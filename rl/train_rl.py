import random
from pathlib import Path

import joblib
from stable_baselines3 import TD3, DDPG, SAC, PPO
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
import numpy as np
from dotenv import load_dotenv
import os

from rl.Controllers import Controllers
from utils import str_to_bool

load_dotenv()
from rl.Agent import Agent

model_dir="rl/models"
log_dir ="rl/logs"

def train_TD3(env):
    n_actions = env.action_space.shape[0]
    train_params_name= os.environ.get("TRAIN_PARAMS_NAME","default")
    noise_value= float(os.environ.get("NOISE_VALUE",0.2))
    learning_rate= float(os.environ.get("LEARNING_RATE",0.0003))
    learning_starts=int(os.environ.get("LEARNING_STARTS",3000))
    train_freq= int(os.environ.get("TRAIN_FREQ",100))
    gradient_steps= int(os.environ.get("GRADIENT_STEPS",100))
    experience_name = os.environ.get("EXPERIENCE_NAME","experience"+ str(random.randint(0, 1000)))
    use_deeper_network=str_to_bool(os.environ.get("USE_DEEPER_NETWORK","False"))
    imitate=str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE","False"))
    replay_buffer_size=int(os.environ.get("REPLAY_BUFFER_SIZE",1000000))
    batch_size=int(os.environ.get("BATCH_SIZE",128))
    is_normal_action_noise=str_to_bool(os.environ.get("IS_NORMAL_ACTION_NOISE","True"))
    insert_experiences_to_replay_buffer=str_to_bool(os.environ.get("INSERT_EXPERIENCES_TO_REPLAY_BUFFER","False"))


    if use_deeper_network:
        policy_kwargs = dict(net_arch=dict(pi=[400, 300], qf=[400, 300]))
    else:
        policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256]))


    transitions = []
    if imitate:
        controller = Controllers()
        obs = env.reset()
        done = False
        i=100
        while i>0:
            while not done:
                action = controller.get_manual_input()
                next_obs, reward, done, info = env.step(action)
                transitions.append((obs, action, reward, next_obs, done,[info]))
                obs = next_obs
            i=i-1
            done = False
            env.reset()

    if is_normal_action_noise:
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=noise_value * np.ones(n_actions))
    else:
        action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=noise_value * np.ones(n_actions)
        )


    td3 = TD3(
        "MlpPolicy", env,
        action_noise=action_noise,
        verbose=1,
        device='cpu',
        tensorboard_log=train_params_name+log_dir,
        learning_rate=learning_rate,
        learning_starts=learning_starts,
        train_freq=(train_freq, "step"),
        gradient_steps=gradient_steps,
        batch_size=batch_size,
        policy_kwargs=policy_kwargs,
        buffer_size=replay_buffer_size
    )


    if not imitate:
        transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_imitationimitatet3.pkl")

        # transitions= joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_random_reverse_v1imitatet2.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_reverse_1imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_v1_2imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/Experience/train_rl_bc_rl15imitatet1.pkl")

        # transitions = transitions[:len(transitions) // 2]
        #
        # transitions = joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/train_rl_bc_rl3experience552.pkl")

    if insert_experiences_to_replay_buffer:
        for obs, action, reward, next_obs, done,infos in transitions:
            td3.replay_buffer.add(obs, next_obs, action, reward, done,infos)
    if imitate:
        try:
            save_path = Path(
                "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions")
            save_path.mkdir(parents=True, exist_ok=True)
            joblib.dump(transitions,save_path / train_params_name + experience_name + ".pkl")

        except Exception as e:
            print(f"Failed to save transition: in desired path due to {e}")
            joblib.dump(transitions, "rl/imitation_transitions/" + train_params_name + experience_name + ".pkl")


    if not imitate:
        timesteps = int(os.environ.get("TIMESTEPS", 100000))
        iters = 0
        while True:
            print(f"Transitions in replay buffer: {td3.replay_buffer.size()}  after {iters} iterations")
            iters += 1
            current_step = td3.num_timesteps
            decayed_sigma = get_decayed_sigma(current_step, initial_sigma=noise_value, final_sigma=0.05)
            action_noise.sigma = decayed_sigma * np.ones(n_actions)
            td3.action_noise = action_noise
            print(f"Action noise sigma dropped to {action_noise.sigma} after {iters} iterations")
            td3.learn(total_timesteps=timesteps, reset_num_timesteps=False)
            td3.save(f"{train_params_name+model_dir}/td3 _ {timesteps * iters}")



def get_decayed_sigma(timestep, initial_sigma=0.1, final_sigma=0.05, decay_steps=1e6):
    return max(final_sigma, initial_sigma * (1 - timestep / decay_steps))



def train_DDPG(env):
    n_actions = env.action_space.shape[0]
    train_params_name= os.environ.get("TRAIN_PARAMS_NAME","default")
    noise_value= float(os.environ.get("NOISE_VALUE",0.2))
    learning_rate= 0.0001
    learning_starts=int(os.environ.get("LEARNING_STARTS",3000))
    train_freq= int(os.environ.get("TRAIN_FREQ",100))
    gradient_steps= int(os.environ.get("GRADIENT_STEPS",100))
    experience_name = os.environ.get("EXPERIENCE_NAME","experience"+ str(random.randint(0, 1000)))
    use_deeper_network=str_to_bool(os.environ.get("USE_DEEPER_NETWORK","False"))
    imitate=str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE","False"))
    replay_buffer_size=int(os.environ.get("REPLAY_BUFFER_SIZE",1000000))
    batch_size=int(os.environ.get("BATCH_SIZE",128))
    is_normal_action_noise=str_to_bool(os.environ.get("IS_NORMAL_ACTION_NOISE","True"))
    insert_experiences_to_replay_buffer=str_to_bool(os.environ.get("INSERT_EXPERIENCES_TO_REPLAY_BUFFER","False"))


    if use_deeper_network:
        policy_kwargs = dict(net_arch=dict(pi=[400, 300], qf=[400, 300]))
    else:
        policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256]))


    transitions = []
    if imitate:
        controller = Controllers()
        obs = env.reset()
        done = False
        i=100
        while i>0:
            while not done:
                action = controller.get_manual_input()
                next_obs, reward, done, info = env.step(action)
                transitions.append((obs, action, reward, next_obs, done,[info]))
                obs = next_obs
            i=i-1
            done = False
            env.reset()

    if is_normal_action_noise:
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=noise_value * np.ones(n_actions))
    else:
        action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=noise_value * np.ones(n_actions)
        )

    ddpg = DDPG(
        "MlpPolicy", env,
        action_noise=action_noise,
        verbose=1,
        device='cpu',
        tensorboard_log=train_params_name + log_dir,
        learning_rate=learning_rate,
        learning_starts=learning_starts,
        train_freq=(train_freq, "step"),
        gradient_steps=gradient_steps,
        batch_size=batch_size,
        policy_kwargs=policy_kwargs,
        buffer_size=replay_buffer_size
    )


    if not imitate:
        transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_imitationimitatet3.pkl")
        # transitions= joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_random_reverse_v1imitatet2.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_reverse_1imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_v1_2imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/Experience/train_rl_bc_rl15imitatet1.pkl")

        # transitions = transitions[:len(transitions) // 2]
        #
        # transitions = joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/train_rl_bc_rl3experience552.pkl")

    if insert_experiences_to_replay_buffer:
        for obs, action, reward, next_obs, done,infos in transitions:
            ddpg.replay_buffer.add(obs, next_obs, action, reward, done,infos)
    if imitate:
        try:
            save_path = Path(
                "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions")
            save_path.mkdir(parents=True, exist_ok=True)
            joblib.dump(transitions,save_path / train_params_name + experience_name + ".pkl")

        except Exception as e:
            print(f"Failed to save transition: in desired path due to {e}")
            joblib.dump(transitions, "rl/imitation_transitions/" + train_params_name + experience_name + ".pkl")


    if not imitate:
        timesteps = int(os.environ.get("TIMESTEPS", 100000))
        iters = 0
        while True:
            print(f"Transitions in replay buffer: {ddpg.replay_buffer.size()}  after {iters} iterations")
            iters += 1
            current_step = ddpg.num_timesteps
            decayed_sigma = get_decayed_sigma(current_step, initial_sigma=noise_value, final_sigma=0.05)
            action_noise.sigma = decayed_sigma * np.ones(n_actions)
            ddpg.action_noise = action_noise
            print(f"Action noise sigma dropped to {action_noise.sigma} after {iters} iterations")
            ddpg.learn(total_timesteps=timesteps, reset_num_timesteps=False)
            ddpg.save(f"{train_params_name+model_dir}/ddpg _ {timesteps * iters}")


def train_SAC(env):
    n_actions = env.action_space.shape[0]
    train_params_name= os.environ.get("TRAIN_PARAMS_NAME","default")
    noise_value= float(os.environ.get("NOISE_VALUE",0.2))
    learning_rate= float(os.environ.get("LEARNING_RATE",0.0001))
    learning_starts=int(os.environ.get("LEARNING_STARTS",3000))
    train_freq= int(os.environ.get("TRAIN_FREQ",100))
    gradient_steps= int(os.environ.get("GRADIENT_STEPS",100))
    experience_name = os.environ.get("EXPERIENCE_NAME","experience"+ str(random.randint(0, 1000)))
    use_deeper_network=str_to_bool(os.environ.get("USE_DEEPER_NETWORK","False"))
    imitate=str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE","False"))
    replay_buffer_size=int(os.environ.get("REPLAY_BUFFER_SIZE",1000000))
    batch_size=int(os.environ.get("BATCH_SIZE",128))
    is_normal_action_noise=str_to_bool(os.environ.get("IS_NORMAL_ACTION_NOISE","True"))
    insert_experiences_to_replay_buffer=str_to_bool(os.environ.get("INSERT_EXPERIENCES_TO_REPLAY_BUFFER","False"))

    if use_deeper_network:
        policy_kwargs = dict(net_arch=dict(pi=[400, 300], qf=[400, 300]))
    else:
        policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256]))


    transitions = []
    if imitate:
        controller = Controllers()
        obs = env.reset()
        done = False
        i=100
        while i>0:
            while not done:
                action = controller.get_manual_input()
                next_obs, reward, done, info = env.step(action)
                transitions.append((obs, action, reward, next_obs, done,[info]))
                obs = next_obs
            i=i-1
            done = False
            env.reset()

    sac = SAC(
        "MlpPolicy", env,
        verbose=1,
        device='cpu',
        tensorboard_log=train_params_name + log_dir,
        learning_rate=learning_rate,
        learning_starts=learning_starts,
        train_freq=(train_freq, "step"),
        gradient_steps=gradient_steps,
        batch_size=batch_size,
        policy_kwargs=policy_kwargs,
        buffer_size=replay_buffer_size
    )


    if not imitate:
        transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_imitationimitatet3.pkl")
        # transitions= joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_random_reverse_v1imitatet2.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_reverse_1imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions/train_rl_bc_v1_2imitatet1.pkl")
        # transitions=joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/Experience/train_rl_bc_rl15imitatet1.pkl")

        # transitions = transitions[:len(transitions) // 2]
        #
        # transitions = joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/train_rl_bc_rl3experience552.pkl")

    if insert_experiences_to_replay_buffer:
        for obs, action, reward, next_obs, done,infos in transitions:
            sac.replay_buffer.add(obs, next_obs, action, reward, done,infos)
    if imitate:
        try:
            save_path = Path(
                "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/imitation_transitions")
            save_path.mkdir(parents=True, exist_ok=True)
            joblib.dump(transitions,save_path / train_params_name + experience_name + ".pkl")

        except Exception as e:
            print(f"Failed to save transition: in desired path due to {e}")
            joblib.dump(transitions, "rl/imitation_transitions/" + train_params_name + experience_name + ".pkl")


    if not imitate:
        timesteps = int(os.environ.get("TIMESTEPS", 100000))
        iters = 0
        while True:
            print(f"Transitions in replay buffer: {sac.replay_buffer.size()}  after {iters} iterations")
            iters += 1
            sac.learn(total_timesteps=timesteps, reset_num_timesteps=False)
            sac.save(f"{train_params_name+model_dir}/sac _ {timesteps * iters}")


def train_PPO(env):
    train_params_name= os.environ.get("TRAIN_PARAMS_NAME","default")
    learning_rate= 0.0001
    use_deeper_network=str_to_bool(os.environ.get("USE_DEEPER_NETWORK","False"))
    imitate=str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE","False"))
    batch_size=int(os.environ.get("BATCH_SIZE",128))


    if use_deeper_network:
        policy_kwargs = dict(net_arch=dict(pi=[400, 300], qf=[400, 300]))
    else:
        policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256])),
    ppo = PPO(
        "MlpPolicy", env,
        verbose=1,
        device='cpu',
        tensorboard_log=train_params_name + log_dir,
        learning_rate=learning_rate,
        batch_size=batch_size,
        policy_kwargs=policy_kwargs
    )

    if not imitate:
        timesteps = int(os.environ.get("TIMESTEPS", 100000))
        iters = 0
        while True:
            iters += 1
            ppo.learn(total_timesteps=timesteps, reset_num_timesteps=False)
            ppo.save(f"{train_params_name+model_dir}/ppo _ {timesteps * iters}")


if __name__ == "__main__":
    train =True
    env = Agent()
    model=os.environ.get("MODEL","TD3")
    if train:
        if model.lower()=="td3":
            train_TD3(env)
        elif model.lower()=="ddpg":
            train_DDPG(env)
        elif model.lower()=="sac":
            train_SAC(env)
        elif model.lower()=="ppo":
            train_PPO(env)
    else:
        pass
        # evaluate_agent(env)