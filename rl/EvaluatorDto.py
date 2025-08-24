import os
from pathlib import Path

import joblib
class EvaluatorDto:
    def __init__(self, rewards_per_delivery,rewards_per_sequence,time_taken_per_delivery,time_taken_per_sequence,terminal_reason_per_delivery,terminal_reason_per_sequence,per_sequence_reward,per_delivery_reward,total_reward,episode_number,):
        self.rewards_per_delivery = rewards_per_delivery
        self.rewards_per_sequence = rewards_per_sequence
        self.time_taken_per_delivery = time_taken_per_delivery
        self.time_taken_per_sequence = time_taken_per_sequence
        self.terminal_reason_per_delivery = terminal_reason_per_delivery
        self.terminal_reason_per_sequence = terminal_reason_per_sequence
        self.per_sequence_reward = per_sequence_reward
        self.per_delivery_reward = per_delivery_reward
        self.total_reward = total_reward
        self.episode_number = episode_number

    def save_to_file(self):
        eval_name = os.environ.get("EVALUATION_NAME", "default_eval")
        out_dir = Path("EvaluationResult")
        out_dir.mkdir(parents=True, exist_ok=True)
        file_path = out_dir / f"{eval_name}.pkl"
        try:
            joblib.dump(self, file_path)
        except Exception as e:
            print(f"Failed to save evaluation result due to {e}")