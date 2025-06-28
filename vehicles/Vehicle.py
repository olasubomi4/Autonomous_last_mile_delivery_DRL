from enum import Enum
from collections import namedtuple

VehicleInfo = namedtuple('VehicleInfo', ['name', 'high_way_max_speed', 'street_max_speed'])
class Vehicles(Enum):
    CAR=VehicleInfo("car",65,60)
    BIKE=VehicleInfo("bike",20,10)