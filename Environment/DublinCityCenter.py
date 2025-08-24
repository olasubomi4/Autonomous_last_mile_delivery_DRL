import copy
import os

import pygame

from Constant import Constant
from DeliveryStates import DeliveryStates
from Environment.Grid import Grid
from utils import blit_rotate_center, manhattan_distance, str_to_bool, scale_delivery_location_to_grid_cell_size
import random
import time
import heapq
import joblib
from dotenv import load_dotenv
from vehicles.Delivery import Delivery

load_dotenv()

class DublinCityCenter:
    def __init__(self, track,track_border,obstacle,delivery_location,vehicle,win,width,height):
        self.track = track
        self.track_clone = track.copy()
        self.track_border = track_border
        self.obstacle = obstacle
        self.delivery_location = delivery_location
        self.vehicle = vehicle
        self.win = win
        self.width = width
        self.height = height
        self.track_border_mask,self.obstacle_mask,self.delivery_location_mask = self.generate_masks()
        self.delivery_queue = []
        heapq.heapify(self.delivery_queue)
        self.delivery_sequence_cache=self.load_delivery_sequence_cache()
        self.reset_counter=0
        # self.test_delivery_sequence_cache=[]

    def generate_masks(self):
        TRACK_BORDER_MASK = pygame.mask.from_surface(self.track_border)
        OBSTACLE_MASK = pygame.mask.from_surface(self.obstacle)
        DELIVERY_LOCATION_MASK = pygame.mask.from_surface(self.delivery_location)

        return TRACK_BORDER_MASK,OBSTACLE_MASK,DELIVERY_LOCATION_MASK

    def draw(self, win, images, delivery_vehicle, obstacles=None, deliveries=None,path=None):
        # Calculate camera offset
        offset_x = delivery_vehicle.x - self.win.get_width() / 2
        offset_y = delivery_vehicle.y - self.win.get_height() / 2

        # Clamp the offset so we don’t scroll beyond the map
        max_x = self.track.get_width() - self.win.get_width()
        max_y = self.track.get_height() - self.win.get_height()

        offset_x = max(0, min(offset_x, max_x))
        offset_y = max(0, min(offset_y, max_y))

        # Draw each image with the offset applied
        if path is not None:
            self.track.blit(self.track_clone, (0, 0))

            # self.track.fill((0, 0, 0))
            for point in path:
                pygame.draw.rect(self.track, (0, 255, 0), (point[0], point[1], 1, 1))

        for img, pos in images:
            win.blit(img, (pos[0] - offset_x, pos[1] - offset_y))

        for obstacle in obstacles:
            win.blit(self.obstacle, (obstacle.x - offset_x, obstacle.y - offset_y))



        for delivery in deliveries:
            state = delivery.delivery_state
            if state==DeliveryStates.PREPARING or state==DeliveryStates.IN_PROGRESS:
                delivery_destination = delivery.delivery_destination
                win.blit(self.delivery_location, (delivery_destination.x - offset_x, delivery_destination.y - offset_y))

                # pygame.display.update()

        # Always draw the car at window center (or appropriate offset if clamped at edge)
        car_draw_x = delivery_vehicle.x - offset_x
        car_draw_y = delivery_vehicle.y - offset_y
        delivery_vehicle.car_center=blit_rotate_center(win, delivery_vehicle.img, (car_draw_x, car_draw_y), delivery_vehicle.angle)

        # pygame.display.update()



    def move_vehicle(self,delivery_vehicle,grid):
        keys = pygame.key.get_pressed()
        moved = False

        if keys[pygame.K_a]:
            delivery_vehicle.rotate(left=True)
        if keys[pygame.K_d]:
            delivery_vehicle.rotate(right=True)
        if keys[pygame.K_w]:
            moved = True
            grid_node = grid.grid[int(delivery_vehicle.x) // Grid.CELL_SIZE][int(delivery_vehicle.y) // Grid.CELL_SIZE]

            delivery_vehicle.move_forward(grid_node)
        if keys[pygame.K_s]:
            moved = True
            grid_node = grid.grid[int(delivery_vehicle.x) // Grid.CELL_SIZE][int(delivery_vehicle.y) // Grid.CELL_SIZE]
            delivery_vehicle.move_backward(grid_node)

        if not moved:
            delivery_vehicle.reduce_speed()

    def move_vehicle_rl(self,delivery_vehicle,grid,steering_action,acceleration_action):
        moved = False

        if steering_action!=0:
            try:
                steering_action = int((steering_action / 1) * 5)
            except Exception as e:
                print(e)
                steering_action = 0
            delivery_vehicle.rotate_rl(steering_action)

        if  acceleration_action!=0:
            moved = True
            is_move_forward = acceleration_action>0
            try:
                acceleration_action = (acceleration_action / 1) * 0.1
            except Exception as e:
                print(e)
                acceleration_action = 0
            grid_node = grid.grid[int(delivery_vehicle.x) // Grid.CELL_SIZE][int(delivery_vehicle.y) // Grid.CELL_SIZE]
            if is_move_forward:
                delivery_vehicle.move_forward_rl(acceleration_action,grid_node)
            else:
                delivery_vehicle.move_backward_rl(acceleration_action,grid_node)
        if not moved:
            delivery_vehicle.reduce_speed()

    def isObstacleOnTheWay(self,obstacle):
        offset = (obstacle[0], obstacle[1])
        poi = self.track_border_mask.overlap(self.obstacle_mask, offset)
        return poi

    def generate_obstacles(self,grid,delivery_vehicle_start_pos,num_obstacles: int = 10):
        obstacles = []
        i = 0
        while i < num_obstacles:
            delivery_vehicle_start_pos_x,delivery_vehicle_start_pos_y = delivery_vehicle_start_pos[0] // Grid.CELL_SIZE, delivery_vehicle_start_pos[1] // Grid.CELL_SIZE
            start_pos_grid_node = grid.grid[delivery_vehicle_start_pos_x][delivery_vehicle_start_pos_y]
            x = random.randint(0, grid.width-1)
            y = random.randint(0, grid.height-1)
            try :
                obstacle_grid_node = grid.grid[x][y]
                if start_pos_grid_node !=obstacle_grid_node and   (self.isObstacleOnTheWay((x*Grid.CELL_SIZE, y*Grid.CELL_SIZE)) is None) and obstacle_grid_node.is_road:
                    obstacle_grid_node.is_blocked = True
                    obstacles.append(obstacle_grid_node)
                    i = i + 1
            except Exception as e:
                print(f"error at {x} -{y}")
        return obstacles

    def generate_deliveries(self,grid,delivery_vehicle_start_pos,num_deliveries: int = 4, cache_sequence_deliveries_counter=0):
        deliveries = []
        i = 0
        use_cached_delivery_sequence=str_to_bool(os.environ.get("USE_CACHED_DELIVERY_SEQUENCE","False"))
        if use_cached_delivery_sequence:
            return self.generate_deliveries_from_cache(cache_sequence_deliveries_counter)
        # num_deliveries=0
        while i < num_deliveries:
            delivery_vehicle_start_pos_x,delivery_vehicle_start_pos_y = delivery_vehicle_start_pos[0] // Grid.CELL_SIZE, delivery_vehicle_start_pos[1] // Grid.CELL_SIZE
            start_pos_grid_node = grid.grid[delivery_vehicle_start_pos_x][delivery_vehicle_start_pos_y]
            x = random.randint(0, grid.width - 1)
            y = random.randint(0, grid.height - 1)
            delivery_grid_node = grid.grid[x][y]

            if start_pos_grid_node != delivery_grid_node and (not delivery_grid_node.is_blocked ) and delivery_grid_node.is_road and self.is_surrounding_area_clear(x,y,grid):
                deliveries.append(
                    Delivery(delivery_grid_node)
                )
                i = i + 1

        # deliveries.append((1800,600)) - trinity location
        # for x,y in Constant.EASY_DELIVERY_LOCATIONS:
        #     xd,yd= scale_delivery_location_to_grid_cell_size(x,y)
        #     deliveries.append(Delivery(grid.grid[xd][yd]))
        #     if len(deliveries)>num_deliveries:
        #         break
        # deliveries.append(Delivery(grid.grid[61][39]))
        # deliveries.append(Delivery(grid.grid[111][55]))
        # deliveries.append(Delivery(grid.grid[222][82]))
        # deliveries.append(Delivery(grid.grid[465][103]))
        # deliveries.append(self.generate_deliveries_from_cache(cache_sequence_deliveries_counter)[2])
        return deliveries

    def generate_deliveries_from_desired_location_list(self,grid,delivery_vehicle_start_pos,delivery_list):
        deliveries = []
        i = 0
        print(f"first delivery location {delivery_list[0]}")
        for x,y in delivery_list:
            delivery_vehicle_start_pos_x,delivery_vehicle_start_pos_y = delivery_vehicle_start_pos[0] // Grid.CELL_SIZE, delivery_vehicle_start_pos[1] // Grid.CELL_SIZE
            start_pos_grid_node = grid.grid[delivery_vehicle_start_pos_x][delivery_vehicle_start_pos_y]
            xd,yd= scale_delivery_location_to_grid_cell_size(x,y)
            delivery_grid_node = grid.grid[xd][yd]

            if start_pos_grid_node != delivery_grid_node and (not delivery_grid_node.is_blocked ) and delivery_grid_node.is_road and self.is_surrounding_area_clear(x,y,grid):
                deliveries.append(
                    Delivery(delivery_grid_node)
                )
                i = i + 1
            else:
                print(f"delivery location {x},{y} is blocked")
        return deliveries

    def generate_deliveries_from_cache(self,cache_sequence_deliveries_counter=0):
        cache_sequence_deliveries_counter= (cache_sequence_deliveries_counter%len(self.delivery_sequence_cache))
        deliveries=self.delivery_sequence_cache[cache_sequence_deliveries_counter]
        deliveries =copy.deepcopy(deliveries)
        return deliveries


    def load_delivery_sequence_cache(self):
        is_imitation_learning=str_to_bool(os.environ.get("IS_IMITATION_LEARNING_MODE","False"))

        if is_imitation_learning:
            imitation_learning_cache_file_path = os.environ.get("IMITATION_LEARNING_DELIVERY_SEQUENCE_CACHE_FILE_PATH", None)
            if imitation_learning_cache_file_path is None:
                return []
            else:
                return joblib.load(imitation_learning_cache_file_path)
        is_train=str_to_bool(os.environ.get("IS_TRAIN_MODE","True"))
        if is_train:
            train_cache_file_path=os.environ.get("TRAIN_DELIVERY_SEQUENCE_CACHE_FILE_PATH",None)
            if train_cache_file_path is None:
                return []
            return joblib.load(train_cache_file_path)
        else:
            difficulty_level = os.environ.get("DELIVERY_DIFFICULTY_LEVEL", "Easy")
            if difficulty_level.lower() == "medium":
                test_cache_file_path ="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences66.pkl"

            elif difficulty_level.lower() == "hard":
                test_cache_file_path ="/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences18.pkl"
            else:
                test_cache_file_path = "/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/rl/Delivery_sequences/test_delivery_sequences75.pkl"
            if test_cache_file_path is None:
                return []
            return joblib.load(test_cache_file_path)

    def is_surrounding_area_clear(self,x, y, grid):
        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        for dx, dy in directions:
            for step in range(1, 5):
                nx = x + (dx * step)
                ny = y + (dy * step)
                if 0 <= nx < grid.width and 0 <= ny < grid.height:
                    cell = grid.grid[nx][ny]
                    if cell.is_blocked and not cell.is_road:
                        return False
        return True


    def init_delivery_queue(self,deliveries,delivery_vehicle,grid):
        if len(self.delivery_queue)>0:
            self.delivery_queue.clear()
        for delivery in deliveries:
            if delivery.delivery_state == DeliveryStates.PENDING or delivery.delivery_state == DeliveryStates.PREPARING:
                cost_from_distance = self.get_rider_cost_distance_from_delivery(delivery, delivery_vehicle, grid)
                heapq.heappush(self.delivery_queue,(cost_from_distance[1],delivery,cost_from_distance[0]))

    def get_closest_delivery(self,delivery_vehicle):
        # print(heapq.nsmallest(len(self.delivery_queue),self.delivery_queue))
        heapq.nsmallest(len(self.delivery_queue),self.delivery_queue)

        closest_delivery=heapq.heappop(self.delivery_queue)
        return closest_delivery

    def update_delivery_state_after_finding_tagret(self,deliveries,target_delivery):
        target_delivery[1].delivery_state = DeliveryStates.IN_PROGRESS
        for delivery in deliveries:
            if delivery.delivery_state == DeliveryStates.PREPARING:
                delivery.delivery_state = DeliveryStates.PENDING

    def reset(self,grid,delivery_vehicle_start_pos,num_obstacles=10,num_deliveries=4):
        self.reset_counter=self.reset_counter+1
        obstacles = self.generate_obstacles(grid, delivery_vehicle_start_pos, num_obstacles)
        deliveries = self.generate_deliveries(grid, delivery_vehicle_start_pos, num_deliveries,self.reset_counter)
        start_time = time.time()
        return obstacles, deliveries, start_time


    def get_rider_cost_distance_from_delivery(self,delivery:Delivery,delivery_vehicle,grid:Grid):
        delivery_destination = delivery.delivery_destination
        result =grid.a_star_path_planning((int(delivery_vehicle.x), int(delivery_vehicle.y)), delivery_destination, delivery_vehicle.vehicle)
        if result is not None:
            return result

        #retry
        print("retrying path planning for delivery")
        directions=[ (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),]
        for dx, dy in directions:
            result = grid.a_star_path_planning((int(delivery_vehicle.x), int(delivery_vehicle.y)), delivery_destination,
                                               delivery_vehicle.vehicle,dx,dy)
            if result is not None:
                return result

        #fail-safe if retry doesnt work
        top_right_screen_position=(0,0)
        map_max_x_axis=self.width
        map_max_y_axis=self.height
        max_manhattan_distance=(manhattan_distance(top_right_screen_position[0],top_right_screen_position[1],map_max_x_axis,map_max_y_axis))
        max_int_value=  max_manhattan_distance
        return [],max_int_value





