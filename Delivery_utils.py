import random

import joblib
from rl.SImulation_rl import Simulation_rl
from dotenv import load_dotenv
load_dotenv()
from itertools import combinations
from pathlib import Path
import os

def str_to_bool(value):
    return value.lower() in ("true", "1", "yes", "on")

def generate_n_amounts_of_delivery_locations(n,counter):
    simulation = Simulation_rl()
    simulation._init_simulation()
    save_path="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_locations"
    name="delivery_locations"+str(counter)+".pkl"
    delivery_locations=simulation.env.generate_deliveries(simulation.grid, simulation.player_start_pos, n)

    joblib.dump(delivery_locations,save_path+"/"+name)
    return delivery_locations
# def generate_n_sequence_of_deliveries(n,possible_delivery):
#     save_path="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences"
#     name = ""
#     is_imitation_learning = str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE", "False"))
#     if is_imitation_learning:
#         name = "imitation_delivery_sequences" + str(random.randint(0, 100)) + ".pkl"
#     else:
#         is_train_sequence=str_to_bool(os.environ.get("IS_TRAIN_MODE","YES"))
#         if is_train_sequence:
#             name="train_delivery_sequences"+str(random.randint(0,100))+".pkl"
#         else:
#             name="test_delivery_sequences"+str(random.randint(0, 100)) + ".pkl"
#     number_of_deliveries_in_a_sequence=int(os.environ.get("NUMBER_OF_DELIVERIES_IN_A_SEQUENCE",4))
#     sequence_of_deliveries=[]
#
#     if number_of_deliveries_in_a_sequence > len(possible_delivery):
#         raise ValueError("Not enough unique deliveries in 'possible_delivery' to form a sequence.")
#
#     for i in range(n):
#         selected_deliveries=[]
#         while len(selected_deliveries)< number_of_deliveries_in_a_sequence:
#             selected_delivery_number=random.randint(0,len(possible_delivery)-1)
#             if possible_delivery[selected_delivery_number] in selected_deliveries:
#                 continue
#             selected_deliveries.append(possible_delivery[selected_delivery_number])
#
#         sequence_of_deliveries.append(selected_deliveries)
#     joblib.dump(sequence_of_deliveries,save_path + "/" + name)
#     return sequence_of_deliveries


def generate_n_sequence_of_deliveries(n, possible_delivery,counter):


    save_path = Path("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences")
    save_path.mkdir(parents=True, exist_ok=True)

    # is_imitation_learning = str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE", "False"))
    # is_train_sequence = str_to_bool(os.environ.get("IS_TRAIN_MODE", "YES"))
    imitation_file_name = f"imitation_delivery_sequences{counter}.pkl"
    # elif is_train_sequence:
    train_file_name = f"train_delivery_sequences{counter}.pkl"
    # else:
    test_file_name = f"test_delivery_sequences{counter}.pkl"

    number_of_deliveries_in_a_sequence = int(os.environ.get("NUMBER_OF_DELIVERIES_IN_A_SEQUENCE", 4))

    if number_of_deliveries_in_a_sequence > len(possible_delivery):
        raise ValueError("Not enough unique deliveries in 'possible_delivery' to form a sequence.")

    # Step 1: Generate all combinations (order doesn't matter)
    all_combinations = list(combinations(possible_delivery, number_of_deliveries_in_a_sequence))

    if n > len(all_combinations):
        raise ValueError(f"Requested {n} sequences, but only {len(all_combinations)} possible combinations exist.")

    # Step 2: Sample n diverse (unique) sequences
    sequence_of_deliveries = random.sample(all_combinations, n)

    sequence_of_deliveries = [list(seq) for seq in sequence_of_deliveries]

    split_size= len(sequence_of_deliveries)//3

    imitation_seqs = sequence_of_deliveries[:split_size]
    test_seqs = sequence_of_deliveries[split_size:2*split_size]
    train_seqs = sequence_of_deliveries[2*split_size:]

    # Step 3: Save to file
    joblib.dump(imitation_seqs, save_path / imitation_file_name)
    joblib.dump(train_seqs, save_path / train_file_name)
    joblib.dump(test_seqs, save_path / test_file_name)


    return sequence_of_deliveries

def generate_n_sequence_of_diverse_deliveries(n,counter):


    save_path = Path("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences")
    save_path.mkdir(parents=True, exist_ok=True)

    # is_imitation_learning = str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE", "False"))
    # is_train_sequence = str_to_bool(os.environ.get("IS_TRAIN_MODE", "YES"))
    imitation_file_name = f"imitation_delivery_sequences{counter}.pkl"
    # elif is_train_sequence:
    train_file_name = f"train_delivery_sequences{counter}.pkl"
    # else:
    test_file_name = f"test_delivery_sequences{counter}.pkl"

    number_of_deliveries_in_a_sequence = int(os.environ.get("NUMBER_OF_DELIVERIES_IN_A_SEQUENCE", 4))
    simulation = Simulation_rl()
    simulation._init_simulation()
    sequence_of_deliveries=[]
    for i in range(n):
        delivery_locations = simulation.env.generate_deliveries(simulation.grid, simulation.player_start_pos, number_of_deliveries_in_a_sequence)
        sequence_of_deliveries.append(delivery_locations)

    split_size= len(sequence_of_deliveries)//3

    imitation_seqs = sequence_of_deliveries[:split_size]
    test_seqs = sequence_of_deliveries[split_size:2*split_size]
    train_seqs = sequence_of_deliveries[2*split_size:]

    # Step 3: Save to file
    joblib.dump(imitation_seqs, save_path / imitation_file_name)
    joblib.dump(train_seqs, save_path / train_file_name)
    joblib.dump(test_seqs, save_path / test_file_name)


    return sequence_of_deliveries




if __name__ == "__main__":
    number=random.randint(0, 100)
    # unique_deliveries= generate_n_amounts_of_delivery_locations(20,number)
    seqence=generate_n_sequence_of_diverse_deliveries(2000,number)
    print("abc")

