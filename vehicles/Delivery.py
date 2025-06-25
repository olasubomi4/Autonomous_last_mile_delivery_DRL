import time
from DeliveryStates import DeliveryStates
from utils import id_generator
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Delivery:

    def __init__(self,delivery_destination):
        self.id:str = id_generator();
        self.delivery_creation_time = time.time()
        self.delivery_state:DeliveryStates = DeliveryStates.PREPARING
        self.delivery_destination: Tuple[int, int] = delivery_destination
        self.delivery_completed_time = None







