import pygame
import time
import numpy as np

from Constant import Constant
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
        self.TRACK = scale_image(pygame.image.load(basepath+"imgs/track2.png"), 1)
        self.TRACK_BORDER = scale_image(pygame.image.load(basepath+"imgs/track-border2.png"), 1)
        self.OBSTACLE = scale_image(pygame.image.load(basepath+"imgs/obstacle8.png"), 0.02)
        self.DELIVERY_LOCATION = scale_image(pygame.image.load(basepath+"imgs/delivery-locations-icon.png"), 0.05)
        self.RED_CAR = scale_image(pygame.image.load(basepath+"imgs/red-car.png"), 0.3)

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

        self.player_start_pos = (220, 370)
        self.delivery_vehicle = Car(self.RED_CAR, self.player_start_pos, 3, 8)

        self.obstacles = self.env.generate_obstacles(self.grid, self.player_start_pos, num_obstacles=10)
        self.deliveries = self.env.generate_deliveries(self.grid, self.player_start_pos, num_deliveries=10)

        self.env.init_delivery_queue(self.deliveries, self.delivery_vehicle, self.grid)

        #show all delivery for a specific number of seconds
        self.env.draw(self.WIN, self.images, self.delivery_vehicle, self.obstacles, self.deliveries)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # run = False
                break
        time.sleep(10)

        self.target_delivery = self.env.get_closest_delivery(self.delivery_vehicle)
        target_delivery_path=self.target_delivery[2]

        self.env.update_delivery_state_after_finding_tagret(self.deliveries, self.target_delivery)
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries, path=target_delivery_path)

        self.deliveries_pending = len(self.deliveries)
        self.start_time = time.time()
        self.are_all_deliveries_completed_flag = False
        self.next_delivery_flag=False
        self.clock = pygame.time.Clock()
        self.FPS = 60
        self.previous_vehicle_to_target_delivery_distance= Constant.MAX_INT_SIZE


    def next_delivery(self):
        self.env.init_delivery_queue(self.deliveries, self.delivery_vehicle, self.grid)
        self.target_delivery = self.env.get_closest_delivery(self.delivery_vehicle)
        target_delivery_path=self.target_delivery[2]
        self.env.update_delivery_state_after_finding_tagret(self.deliveries, self.target_delivery)
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries, path=target_delivery_path)

    def move(self,steering_action,acceleration_action):
        try:
            steering_action= int((steering_action/1)*5)
        except Exception as e:
            print(e)
            steering_action=0

        try:
            acceleration_action= (acceleration_action/1)*0.1
        except Exception as e:
            print(e)
            acceleration_action=0

        self.env.move_vehicle_rl(self.delivery_vehicle,self.grid,steering_action, acceleration_action)

    def get_sensor_result(self,delivery_vehicle,grid):
        sensor_targets = [(self.OBSTACLE_MASK, self.obstacles), (self.TRACK_BORDER_MASK, [grid.grid[0][0]])]
        sensor_result = delivery_vehicle.update_sensors(sensor_targets)
        return sensor_result

    def draw_sensors(self,delivery_vehicle):
        delivery_vehicle.draw_sensors(self.WIN)

    def does_car_collide_with_obstacle(self,delivery_vehicle,obstacles):
        result= delivery_vehicle.collide_with_obstacle(self.OBSTACLE_MASK,obstacles)
        # delivery_vehicle.reset()
        return result

    def does_car_collide_with_border(self,delivery_vehicle):
        result=  delivery_vehicle.collide(self.TRACK_BORDER_MASK)
        # delivery_vehicle.bounce()
        return result

    def does_car_reach_target_delivery(self,delivery_vehicle):
        return delivery_vehicle.is_delivery_completed(self.DELIVERY_LOCATION_MASK,self.target_delivery) and delivery_vehicle.vel==0

    def get_state(self):
        delivery_destination=self.target_delivery[1].delivery_destination
        delivery_vehicle_distance_to_target= manhattan_distance(self.delivery_vehicle.x,self.delivery_vehicle.y,delivery_destination.x,delivery_destination.y)
        sensor_data= self.get_sensor_result(self.delivery_vehicle,self.grid)
        delivery_vehicle_velocity= self.delivery_vehicle.vel
        delivery_vehicle_location= (self.delivery_vehicle.x,self.delivery_vehicle.y)

        is_there_border_collision= self.does_car_collide_with_border(self.delivery_vehicle)
        if is_there_border_collision:
            self.delivery_vehicle.bounce()
        is_there_obstacle_collision= self.does_car_collide_with_obstacle(self.delivery_vehicle,self.obstacles)

        # target_delivery_location= (self.target_delivery[0],self.target_delivery[1])
        # state = [delivery_vehicle_distance_to_target,sensor_data,delivery_vehicle_velocity,delivery_vehicle_location,(delivery_destination.x,delivery_destination.y)]
        state= State(self.target_delivery[0],sensor_data,self.delivery_vehicle,delivery_destination)
        bottom_right_screen_position=(self.WIDTH,self.HEIGHT)
        state_values= state.scale_state_values(bottom_right_screen_position,self.delivery_vehicle.max_vel)
        return state_values
        # # state = [delivery_vehicle_distance_to_target,delivery_vehicle_velocity,delivery_vehicle_location[0],delivery_vehicle_location[1],delivery_destination.x,delivery_destination.y]
        #
        # return state

    def is_finished(self):
        return self.deliveries_pending==0

    def reset(self):
        if self.are_all_deliveries_completed_flag:
            self.delivery_vehicle.reset()
            self.obstacles, self.deliveries, self.start_time = self.env.reset(self.grid, self.player_start_pos)
            self.deliveries_pending=len(self.deliveries)
            self.next_delivery()
            self.next_level_flag=False
        return self.get_state()

    def draw(self):
        self.env.draw(self.WIN, self.images,self.delivery_vehicle, self.obstacles, self.deliveries)
        self.delivery_vehicle.draw_sensors(self.WIN)
        self.clock.tick(self.FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # run = False
                break
        # pygame.display.update()
        # print("Pygame initialized:", pygame.get_init())
        # print("Display info:", pygame.display.Info())

    # def does_car_reach_target_delivery_and_is_completed(self,delivery_vehicle):


    # def get_reward(self):
    #
    #
    #     pass
    #
    # def calculate_safety_reward(self):
    #     pass
    #
    # def calculate_efficiency_reward(self):
    #     if
    #
    #     pass