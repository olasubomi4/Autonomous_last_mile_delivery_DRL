from Environment.Color import Color
from vehicles.Vehicle import Vehicles

class GridNode:
    max_speed = 70
    def __init__(self,x, y, color,is_road=False,is_blocked=True):
        self.x = x
        self.y = y
        self.color: tuple[int, int, int, int] = color
        self.landmark_location = None
        self.is_road = is_road
        self.is_blocked=is_blocked

    def get_speed_limit(self, vehicle: Vehicles):
        if self.color==Color.BLACK.value:
            return vehicle.value.street_max_speed
        else:
            return vehicle.value.high_way_max_speed

    def get_node_weight(self, vehicle: Vehicles):
        if self.color==Color.BLACK.value:
            return GridNode.max_speed-vehicle.value.street_max_speed
        else:
            return GridNode.max_speed-vehicle.value.high_way_max_speed

# if __name__=="__main__":
#     grid_node = GridNode(1,2,(0, 0, 0,255))
#     print(grid_node.get_speed_limit(Vehicles.CAR))
#     print(grid_node.get_node_weight(Vehicles.CAR))
