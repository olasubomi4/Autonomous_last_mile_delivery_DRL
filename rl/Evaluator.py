import copy
from collections import Counter
from collections import Counter
import numpy as np

import time

from dotenv import load_dotenv
from matplotlib import pyplot as plt
import os

from stable_baselines3 import TD3, SAC, DDPG, PPO

from rl.Agent import Agent
from rl.Controllers import Controllers
from rl.EpisodeEndReason import EpisodeEndReason
import joblib

from rl.EvaluatorDto import EvaluatorDto


def delivery_list():
    difficulty_level = os.environ.get("DELIVERY_DIFFICULTY_LEVEL", "Easy")
    if difficulty_level.lower() == "medium":
        test_cache_file_path = "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences66.pkl"

    elif difficulty_level.lower() == "hard":
        test_cache_file_path = "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences18.pkl"
    else:
        test_cache_file_path = "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences75.pkl"
    if test_cache_file_path is None:
        return []
    return joblib.load(test_cache_file_path)


def create_model(env):
    model_location= os.environ.get("EVALUATION_MODEL_LOCATION","default")
    model_name= os.environ.get("EVALUATION_MODEL_NAME","default")

    if model_name.lower()=="td3":
        return TD3.load(model_location,env=env)
    elif model_name.lower()=="sac":
        return SAC.load(model_location,env=env)
    elif model_name.lower()=="DDPG":
        return DDPG.load(model_location,env=env)
    elif model_name.lower()=="ppo":
        return PPO.load(model_location,env=env)
    else:
        return TD3.load(model_location,env=env)

def evaluate_agent(env,model, delivery_sequences, show_plot=True):
    evaluation_name= os.environ.get("EVALUATION_NAME","default")
    state = env.reset()

    rewards_per_delivery = []
    rewards_per_sequence = []
    time_taken_per_delivery = []
    time_taken_per_sequence = []
    terminal_reason_per_delivery = []
    terminal_reason_per_sequence= []
    per_sequence_reward=0
    per_delivery_reward=0
    delivery_start_time= time.time()
    sequence_start_time=time.time()
    total_reward=0
    done = False
    controller = Controllers()
    delivery_per_sequence=len(delivery_sequences[0])
    episode_number=len(delivery_sequences)*delivery_per_sequence
    # episode_number=4
    while episode_number>0:
        info={}
        while not done:
            supervisor_action=controller.get_supervisors_action()
            # action = controller.get_manual_input()

            if supervisor_action is not None:
                action = supervisor_action
            else:
                action, _ = model.predict(state)
            state, reward, done, info = env.step(action)

            per_delivery_reward+=reward
            per_sequence_reward+=reward

            total_reward += reward
        episode_number-=1
        done=False
        if info.get("successfully_completed_deliveries", False):
            current_sequence_time = time.time()
            time_taken_per_sequence.append(current_sequence_time - sequence_start_time)
            sequence_start_time = time.time()

            rewards_per_sequence.append(per_sequence_reward)
            per_sequence_reward = 0;

            last_three_terminal_reasons = terminal_reason_per_delivery[-3:]
            last_three_terminal_reasons = copy.deepcopy(last_three_terminal_reasons)
            terminal_reason_per_sequence.append(last_three_terminal_reasons.append(info["terminal_reason"]))

        if info.get("completed_a_delivery", False):
            current_delivery_time=time.time()
            time_taken_per_delivery.append(current_delivery_time-delivery_start_time)
            delivery_start_time=time.time()

            rewards_per_delivery.append(per_delivery_reward)
            per_delivery_reward = 0

            terminal_reason_per_delivery.append(info["terminal_reason"])
        #collison or stagnation
        if info["terminal_reason"] == EpisodeEndReason.COLLISION.value or info["terminal_reason"] == EpisodeEndReason.STAGNATION.value:
            current_delivery_time=time.time()
            time_taken_per_delivery.append(current_delivery_time-delivery_start_time)
            delivery_start_time=time.time()

            rewards_per_delivery.append(per_delivery_reward)
            per_delivery_reward = 0

            terminal_reason_per_delivery.append(info["terminal_reason"])


            if info["terminal_reason"] == EpisodeEndReason.COLLISION.value:
                env.simulation.mark_delivery_completed()
                if not env.simulation.is_finished():
                    env.simulation.next_delivery()
            if env.simulation.is_finished():
                current_sequence_time = time.time()
                time_taken_per_sequence.append(current_sequence_time - sequence_start_time)
                sequence_start_time = time.time()

                rewards_per_sequence.append(per_sequence_reward)
                per_sequence_reward = 0;

                last_three_terminal_reasons = terminal_reason_per_delivery[-3:]
                last_three_terminal_reasons = copy.deepcopy(last_three_terminal_reasons)
                terminal_reason_per_sequence.append(last_three_terminal_reasons.append(info["terminal_reason"]))
                if info["terminal_reason"] == EpisodeEndReason.COLLISION.value:
                    env.mark_all_deliveries_as_completed()

        env.reset_counters()
        state = env.reset()

    if show_plot:


        # 2) Reward per completed delivery
        if rewards_per_delivery:
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.plot(range(1, len(rewards_per_delivery) + 1), rewards_per_delivery, marker="o")
            ax.set_title("Reward per completed delivery")
            ax.set_xlabel("Completed delivery index")
            ax.set_ylabel("Reward")
            plt.title(f"{evaluation_name}")
            plt.tight_layout()
            plt.show()

        # 3) Reward per completed sequence
        if rewards_per_sequence:
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.plot(range(1, len(rewards_per_sequence) + 1), rewards_per_sequence, marker="o")
            ax.set_title("Reward per completed sequence")
            ax.set_xlabel("Sequence index")
            ax.set_ylabel("Reward")
            plt.tight_layout()
            plt.title(f"{evaluation_name}")
            plt.show()

        # 4) Time per delivery (steps)
        if time_taken_per_delivery:
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.plot(range(1, len(time_taken_per_delivery) + 1), time_taken_per_delivery, marker="o")
            ax.set_title("Time per completed delivery (seconds)")
            ax.set_xlabel("Completed delivery index")
            ax.set_ylabel("Time/Seconds")
            plt.title(f"{evaluation_name}")
            plt.tight_layout()
            plt.show()

        # 5) Time per sequence (steps)
        if time_taken_per_sequence:
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.plot(range(1, len(time_taken_per_sequence) + 1), time_taken_per_sequence, marker="o")
            ax.set_title("Time per sequence")
            ax.set_xlabel("Sequence index")
            ax.set_ylabel("Time/seconds")
            plt.title(f"{evaluation_name}")
            plt.tight_layout()
            plt.show()

        # 1) Terminal reasons (stacked bars) across sequences
        counts = Counter(terminal_reason_per_delivery)
        fig, ax = plt.subplots(figsize=(8, 4))
        labels = ["success", "collision", "stagnation"]
        values = [counts.get(EpisodeEndReason.SUCCESS, 0),
                  counts.get(EpisodeEndReason.COLLISION, 0),
                  counts.get(EpisodeEndReason.STAGNATION, 0)]
        colors = {
            "success": "green",
            "collision": "red",
            "stagnation": "orange"
        }
        ax.bar(labels, values, color=[colors[label] for label in labels])
        ax.set_title("Terminal reasons (evaluation)")
        ax.set_ylabel("Count")
        plt.tight_layout()
        plt.show()

    print(f"Rewards per deliveries: {len(rewards_per_delivery)}")
    print(f"Rewards per sequences: {len(rewards_per_sequence)}")
    print(f"Average reward per delivery: {sum(rewards_per_delivery)/len(rewards_per_delivery)}")
    print(f"Average reward per sequence: {sum(rewards_per_sequence)/len(rewards_per_sequence)}")
    print(f"Average time taken per delivery: {sum(time_taken_per_delivery)/len(time_taken_per_delivery)}")
    print(f"Average time taken per sequence: {sum(time_taken_per_sequence)/len(time_taken_per_sequence)}")
    print(f"Terminal reasons per delivery: {terminal_reason_per_delivery}")
    print(f"Terminal reasons per sequence: {terminal_reason_per_sequence}")
    return EvaluatorDto(rewards_per_delivery,rewards_per_sequence,time_taken_per_delivery,time_taken_per_sequence,terminal_reason_per_delivery,terminal_reason_per_sequence,per_sequence_reward,per_delivery_reward,total_reward,episode_number)


def plot_reward_per_delivery(results, title="Reward per completed delivery"):
    fig, ax = plt.subplots(figsize=(8, 4))
    for model_name, dto in results.items():
        rewards = dto.rewards_per_delivery
        if not rewards:  # skip empty
            continue
        ax.plot(range(1, len(rewards) + 1), rewards, marker="o", label=model_name)
    ax.set_title(title)
    ax.set_xlabel("Completed delivery index")
    ax.set_ylabel("Reward")
    ax.legend()
    plt.tight_layout()
    plt.show()

def plot_reward_per_sequence(results, title="Reward per completed sequence"):
    fig, ax = plt.subplots(figsize=(8, 4))
    for model_name, dto in results.items():
        rewards = dto.rewards_per_sequence
        if not rewards:
            continue
        ax.plot(range(1, len(rewards) + 1), rewards, marker="o", label=model_name)
    ax.set_title(title)
    ax.set_xlabel("Sequence index")
    ax.set_ylabel("Reward")
    ax.legend()
    plt.tight_layout()
    plt.show()

def plot_time_per_delivery(results, title="Time per completed delivery (seconds)"):
    fig, ax = plt.subplots(figsize=(8, 4))
    for model_name, dto in results.items():
        times = dto.time_taken_per_delivery
        if not times:
            continue
        ax.plot(range(1, len(times) + 1), times, marker="o", label=model_name)
    ax.set_title(title)
    ax.set_xlabel("Completed delivery index")
    ax.set_ylabel("Time (s)")
    ax.legend()
    plt.tight_layout()
    plt.show()

def plot_time_per_sequence(results, title="Time per sequence (seconds)"):
    fig, ax = plt.subplots(figsize=(8, 4))
    for model_name, dto in results.items():
        times = dto.time_taken_per_sequence
        if not times:
            continue
        ax.plot(range(1, len(times) + 1), times, marker="o", label=model_name)
    ax.set_title(title)
    ax.set_xlabel("Sequence index")
    ax.set_ylabel("Time (s)")
    ax.legend()
    plt.tight_layout()
    plt.show()

def plot_terminal_reasons(results, title="Terminal reasons (evaluation)"):
    labels = ["success", "collision", "stagnation"]
    reason_map = {
        "success": EpisodeEndReason.SUCCESS,
        "collision": EpisodeEndReason.COLLISION,
        "stagnation": EpisodeEndReason.STAGNATION,
    }

    model_names = list(results.keys())
    counts_per_model = []
    for model_name in model_names:
        dto = results[model_name]
        c = Counter(dto.terminal_reason_per_delivery)
        counts_per_model.append([c.get(reason_map[l], 0) for l in labels])

    counts_arr = np.array(counts_per_model)  # shape: (num_models, 3)

    x = np.arange(len(model_names))
    width = 0.25

    fig, ax = plt.subplots(figsize=(10, 4))
    for i, label in enumerate(labels):
        ax.bar(x + (i - 1)*width, counts_arr[:, i], width=width, label=label)

    ax.set_xticks(x)
    ax.set_xticklabels(model_names)
    ax.set_title(title)
    ax.set_ylabel("Count")
    ax.legend()
    plt.tight_layout()
    plt.show()

def print_summary(results, EpisodeEndReason=None):
    for name, dto in results.items():
        d = dto.rewards_per_delivery
        s = dto.rewards_per_sequence
        td = dto.time_taken_per_delivery
        ts = dto.time_taken_per_sequence
        reasons = dto.terminal_reason_per_delivery

        # averages
        avg_d = sum(d)/len(d) if d else float("nan")
        avg_s = sum(s)/len(s) if s else float("nan")
        avg_td = sum(td)/len(td) if td else float("nan")
        avg_ts = sum(ts)/len(ts) if ts else float("nan")

        # count terminal reasons
        counts = Counter(reasons)

        # successful vs unsuccessful deliveries
        success_count = counts.get(EpisodeEndReason.SUCCESS, 0) if EpisodeEndReason else counts.get("success", 0)
        total_deliveries = len(reasons)
        unsuccessful_count = total_deliveries - success_count

        print(f"\n=== {name} ===")
        print(f"Total deliveries: {total_deliveries}")
        print(f"Successful deliveries: {success_count}")
        print(f"Unsuccessful deliveries: {unsuccessful_count}")
        if unsuccessful_count > 0:
            print("  Breakdown of failures:")
            for reason, count in counts.items():
                if reason != (EpisodeEndReason.SUCCESS if EpisodeEndReason else "success"):
                    print(f"   - {reason}: {count}")

        print(f"Successful sequences: {len(s)}")
        print(f"Avg reward/delivery: {avg_d:.2f}")
        print(f"Avg reward/sequence: {avg_s:.2f}")
        print(f"Avg time/delivery (s): {avg_td:.2f}")
        print(f"Avg time/sequence (s): {avg_ts:.2f}")


def compare_models(results):
    plot_reward_per_delivery(results)
    plot_reward_per_sequence(results)
    plot_time_per_delivery(results)
    plot_time_per_sequence(results)
    plot_terminal_reasons(results)
    print_summary(results)

if __name__ == "__main__":

    model_comparison_mode=False

    if model_comparison_mode:
        results={
            "td3+cl+easy": joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/EvaluationResult/TD3+CL+Easy.pkl"),
            "sac+IL+CL+EASY": joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/EvaluationResult/SAC+IL+CL+EASY.pkl"),
            "td3+IL+CL+EASY": joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/EvaluationResult/TD3+IL+CL+EASY.pkl"),
            "ddpg+IL+CL+EASY":joblib.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/EvaluationResult/DDPG+IL+CL+EASY.pkl")
        }
        compare_models(results)
    else:
        load_dotenv()
        env = Agent(True)
        model = create_model(env)
        delivery_sequences = delivery_list()
        result=evaluate_agent(env, model, delivery_sequences, show_plot=True)
        result.save_to_file()


