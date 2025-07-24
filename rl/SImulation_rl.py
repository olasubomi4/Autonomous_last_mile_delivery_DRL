import pygame
import time
import numpy as np

from Constant import Constant
from DeliveryStates import DeliveryStates
from Environment.Grid import Grid
from rl.State import State
from utils import scale_image, manhattan_distance
from Environment.DublinCityCenter  import  DublinCityCenter
from vehicles.Car import Car

class Simulation_rl:
    def __init__(self,render_mode=False):
        self.render_mode=render_mode
        pass

    def _init_simulation(self):
        pygame.init()
        basepath="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/"
        self.TRACK = scale_image(pygame.image.load(basepath+"imgs/track4.png"), 1)
        self.TRACK_BORDER = scale_image(pygame.image.load(basepath+"imgs/track-border4.png"), 1)
        self.OBSTACLE = scale_image(pygame.image.load(basepath+"imgs/obstacle8.png"), 0.02)
        self.DELIVERY_LOCATION = scale_image(pygame.image.load(basepath+"imgs/delivery-locations-icon.png"), 0.05)
        self.RED_CAR = scale_image(pygame.image.load(basepath+"imgs/red-car.png"), 0.2)

        # Create masks
        self.TRACK_BORDER_MASK = pygame.mask.from_surface(self.TRACK_BORDER)
        self.OBSTACLE_MASK = pygame.mask.from_surface(self.OBSTACLE)
        self.DELIVERY_LOCATION_MASK = pygame.mask.from_surface(self.DELIVERY_LOCATION)

        self.WIDTH, self.HEIGHT = self.TRACK.get_width(), self.TRACK.get_height()

        self.WIN = pygame.display.set_mode((1300, 800))
        pygame.display.set_caption("Dublin city center")


        self.env = DublinCityCenter(self.TRACK, self.TRACK_BORDER, self.OBSTACLE,
                                    self.DELIVERY_LOCATION, self.RED_CAR,
                                    self.WIN, self.WIDTH, self.HEIGHT)

        self.grid = Grid(self.WIDTH, self.HEIGHT, self.TRACK)
        self.grid.generate_grid(self.RED_CAR, self.TRACK_BORDER_MASK)
        self.images = [(self.TRACK, (0, 0)), (self.TRACK_BORDER, (0, 0))]

        self.player_start_pos = [220, 370]
        self.agent_checkpoint_position = [220, 370,0]

        self.delivery_vehicle = Car(self.RED_CAR, self.player_start_pos, 3, 8)

        self.obstacles = self.env.generate_obstacles(self.grid, self.player_start_pos, num_obstacles=0)
        self.deliveries = self.env.generate_deliveries(self.grid, self.player_start_pos, num_deliveries=0)

        self.env.init_delivery_queue(self.deliveries, self.delivery_vehicle, self.grid)

        #show all delivery for a specific number of seconds
        self.env.draw(self.WIN, self.images, self.delivery_vehicle, self.obstacles, self.deliveries)
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # run = False
                break
        time.sleep(10)

        self.target_delivery = self.env.get_closest_delivery(self.delivery_vehicle)
        target_delivery_path=self.target_delivery[2]

        self.env.update_delivery_state_after_finding_tagret(self.deliveries, self.target_delivery)
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries, path=target_delivery_path)
        pygame.display.update()

        self.deliveries_pending = len(self.deliveries)
        self.start_time = time.time()
        self.are_all_deliveries_completed_flag = False
        self.next_delivery_flag=False
        self.clock = pygame.time.Clock()
        self.FPS = 60
        self.previous_vehicle_to_target_delivery_distance= Constant.MAX_INT_SIZE

    def next_delivery(self):
        self.agent_checkpoint_position = [self.delivery_vehicle.x, self.delivery_vehicle.y,self.delivery_vehicle.angle]
        self.delivery_vehicle.update_vehicle_start_position(self.agent_checkpoint_position)
        self.env.init_delivery_queue(self.deliveries, self.delivery_vehicle, self.grid)
        self.target_delivery = self.env.get_closest_delivery(self.delivery_vehicle)
        target_delivery_path=self.target_delivery[2]
        self.env.update_delivery_state_after_finding_tagret(self.deliveries, self.target_delivery)
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries, path=target_delivery_path)

    def move(self,steering_action,acceleration_action):
        self.env.move_vehicle_rl(self.delivery_vehicle,self.grid,steering_action, acceleration_action)

    def get_sensor_result(self,delivery_vehicle,grid):
        sensor_targets = [(self.OBSTACLE_MASK, self.obstacles), (self.TRACK_BORDER_MASK, [grid.grid[0][0]])]
        sensor_result = delivery_vehicle.update_sensors(sensor_targets)
        return sensor_result

    def draw_sensors(self,delivery_vehicle):
        delivery_vehicle.draw_sensors(self.WIN)


    def does_car_collide_with_obstacle(self):
        result= self.delivery_vehicle.collide_with_obstacle(self.OBSTACLE_MASK,self.obstacles)
        if result:
            self.delivery_vehicle.reset()
        return result

    def does_car_collide_with_border(self):
        result=  self.delivery_vehicle.collide(self.TRACK_BORDER_MASK)
        # if result:
        #     self.delivery_vehicle.reset()
        return result

    def reset_car(self):
        self.delivery_vehicle.reset()

    def does_car_reach_target_delivery(self,delivery_vehicle):
        return delivery_vehicle.is_delivery_completed(self.DELIVERY_LOCATION_MASK,self.target_delivery) and delivery_vehicle.vel==0

    def get_state(self):
        delivery_destination=self.target_delivery[1].delivery_destination
        delivery_vehicle_distance_to_target= manhattan_distance(self.delivery_vehicle.x,self.delivery_vehicle.y,delivery_destination.x,delivery_destination.y)
        sensor_data= self.get_sensor_result(self.delivery_vehicle,self.grid)
        delivery_vehicle_velocity= self.delivery_vehicle.vel
        delivery_vehicle_location= (self.delivery_vehicle.x,self.delivery_vehicle.y)

        # is_there_border_collision= self.does_car_collide_with_border(self.delivery_vehicle)
        # if is_there_border_collision:
        #     self.delivery_vehicle.bounce()
        # is_there_obstacle_collision= self.does_car_collide_with_obstacle(self.delivery_vehicle,self.obstacles)

        # target_delivery_location= (self.target_delivery[0],self.target_delivery[1])
        # state = [delivery_vehicle_distance_to_target,sensor_data,delivery_vehicle_velocity,delivery_vehicle_location,(delivery_destination.x,delivery_destination.y)]
        bottom_right_screen_position=(self.WIDTH,self.HEIGHT)
        state= State(self.target_delivery[0],sensor_data,self.delivery_vehicle,delivery_destination,bottom_right_screen_position)

        return state
        # # state = [delivery_vehicle_distance_to_target,delivery_vehicle_velocity,delivery_vehicle_location[0],delivery_vehicle_location[1],delivery_destination.x,delivery_destination.y]
        #
        # return state

    def is_finished(self):
        return self.deliveries_pending==0

    def has_reached_destination(self):
        return self.delivery_vehicle.is_delivery_completed(self.DELIVERY_LOCATION_MASK, self.target_delivery)

    def mark_delivery_completed(self):
        self.target_delivery[1].delivery_state=DeliveryStates.COMPLETED
        self.deliveries_pending=self.deliveries_pending-1



    def reset(self):
        if self.are_all_deliveries_completed_flag:
            self.agent_checkpoint_position = [self.player_start_pos[0], self.player_start_pos[1],0]
            self.delivery_vehicle.update_vehicle_start_position(self.agent_checkpoint_position)
            self.delivery_vehicle.reset()
            self.obstacles, self.deliveries, self.start_time = self.env.reset(self.grid, self.player_start_pos,num_obstacles=0,num_deliveries=3)
            self.deliveries_pending=len(self.deliveries)
            self.next_delivery()
            self.are_all_deliveries_completed_flag=False
        return self.get_state()

    def draw(self):
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries)
        self.delivery_vehicle.draw_sensors(self.WIN)
        self.clock.tick(self.FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # run = False
                break

        pygame.display.update()



    # def does_car_reach_target_delivery_and_is_completed(self,delivery_vehicle):

    # def get_reward(self, previous_state: State,current_state):
    #     if previous_state is not None:
    #         efficiency_reward = self.calculate_efficiency_reward(previous_state, current_state)
    #         safety_reward = self.calculate_safety_reward(current_state)
    #         timestep_penalty = -0.1
    #
    #         reward= efficiency_reward + safety_reward + timestep_penalty
    #         return self.normalize_reward(reward)
    #     return 0.05

    def get_reward(self, previous_state: State, current_state: State) -> float:
        if previous_state is None:
            return 0.05

        efficiency_weight = 0.5
        safety_weight = 0.5

        efficiency_reward = self.calculate_efficiency_reward(previous_state, current_state)  # in [0, 1]
        safety_reward = self.calculate_safety_reward(current_state)  # needs normalization

        normalized_safety = self.normalize_safety(safety_reward)

        weighted_reward = (
                efficiency_weight * efficiency_reward +
                safety_weight * normalized_safety
        )

        weighted_reward =  max(0.0, min(1.0, weighted_reward))

        weighted_reward= self.transform_result_between_negative_one_and_positive_one(weighted_reward)
        time_step_punishment=0.01
        return weighted_reward-time_step_punishment

    def transform_result_between_negative_one_and_positive_one(self, value):
        return 2*value - 1



    def normalize_safety(self, safety_raw, min_reward=-3.0, max_reward=0):
        return (safety_raw - min_reward) / (max_reward - min_reward)

    def calculate_safety_reward(self,current_state: State) -> float:
        sensors = [
            current_state.get_sensor_one_data(),
            current_state.get_sensor_two_data(),
            current_state.get_sensor_three_data(),
        ]

        total_reward = 0
        for sensor in sensors:
            distance = sensor[0]
            if sensor[2] == 0:
                if distance >= Constant.MAX_SENSOR_DISTANCE :
                    total_reward += 0
                else:
                    total_reward -= (Constant.MAX_SENSOR_DISTANCE - distance) / Constant.MAX_SENSOR_DISTANCE

        return total_reward

    def calculate_efficiency_reward(self,previous_state: State, current_state: State) -> float:
        # previous_distance = previous_state.get_distance_to_target_delivery()
        delivery_vehicle_position_at_previous_state= previous_state.get_delivery_vehicle_location()


        # current_distance = current_state.get_distance_to_target_delivery()
        bottom_right_screen_position = (self.WIDTH, self.HEIGHT)
        top_right_screen_position=(0,0)
        map_max_x_axis=bottom_right_screen_position[0]
        map_max_y_axis=bottom_right_screen_position[1]
        # current_distance = current_distance
        delivery_destination=self.target_delivery[1].delivery_destination
        previous_distance=manhattan_distance(delivery_vehicle_position_at_previous_state[0],delivery_vehicle_position_at_previous_state[1],delivery_destination.x,delivery_destination.y)
        current_distance= (manhattan_distance(self.delivery_vehicle.x,self.delivery_vehicle.y,delivery_destination.x,delivery_destination.y))
        max_manhattan_distance=(manhattan_distance(self.agent_checkpoint_position[0],self.agent_checkpoint_position[1],delivery_destination.x,delivery_destination.y))
        reward= (previous_distance-current_distance)/max_manhattan_distance

        return  max(-1.0, min(1.0, reward))
        # if max_manhattan_distance == 0:
        #     return 1.0
        # reward = 1.0-(current_distance / max_manhattan_distance)
        # return max(0.0,min(1.0,reward))

    def normalize_reward(self,raw_reward, min_reward=-3.0, max_reward=2.4):
        return (raw_reward - min_reward) / (max_reward - min_reward)
