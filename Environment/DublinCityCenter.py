import pygame

from DeliveryStates import DeliveryStates
import utils
from Environment.Grid import Grid
from Environment.GridNode import GridNode
from utils import scale_image, blit_rotate_center
import random
import time
import heapq

from vehicles.Delivery import Delivery


class DublinCityCenter:
    def __init__(self, track,track_border,obstacle,delivery_location,vehicle,win,width,height):
        self.track = track
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

    def generate_masks(self):
        TRACK_BORDER_MASK = pygame.mask.from_surface(self.track_border)
        OBSTACLE_MASK = pygame.mask.from_surface(self.obstacle)
        DELIVERY_LOCATION_MASK = pygame.mask.from_surface(self.delivery_location)

        return TRACK_BORDER_MASK,OBSTACLE_MASK,DELIVERY_LOCATION_MASK

    def draw(self, win, images, player_car, obstacles=None, deliveries=None,path=None):
        # Calculate camera offset
        offset_x = player_car.x - self.win.get_width() / 2
        offset_y = player_car.y - self.win.get_height() / 2

        # Clamp the offset so we don’t scroll beyond the map
        max_x = self.track.get_width() - self.win.get_width()
        max_y = self.track.get_height() - self.win.get_height()

        offset_x = max(0, min(offset_x, max_x))
        offset_y = max(0, min(offset_y, max_y))

        # Draw each image with the offset applied
        for img, pos in images:
            win.blit(img, (pos[0] - offset_x, pos[1] - offset_y))

        for obstacle in obstacles:
            win.blit(self.obstacle, (obstacle.x - offset_x, obstacle.y - offset_y))

        for delivery in deliveries:
            state = delivery.delivery_state
            if state==DeliveryStates.PREPARING or state==DeliveryStates.IN_PROGRESS:
                delivery_destination = delivery.delivery_destination
                win.blit(self.delivery_location, (delivery_destination.x - offset_x, delivery_destination.y - offset_y))
        if path is not None:
            for point in path:
                pygame.draw.rect(self.track, (0, 255, 0), (point[0], point[1], 1, 1))
                # pygame.display.update()

        # Always draw the car at window center (or appropriate offset if clamped at edge)
        car_draw_x = player_car.x - offset_x
        car_draw_y = player_car.y - offset_y
        blit_rotate_center(win, player_car.img, (car_draw_x, car_draw_y), player_car.angle)

        pygame.display.update()

    def move_player(self,player_car):
        keys = pygame.key.get_pressed()
        moved = False

        if keys[pygame.K_a]:
            player_car.rotate(left=True)
        if keys[pygame.K_d]:
            player_car.rotate(right=True)
        if keys[pygame.K_w]:
            moved = True
            player_car.move_forward()
        if keys[pygame.K_s]:
            moved = True
            player_car.move_backward()

        if not moved:
            player_car.reduce_speed()

    def isObstacleOnTheWay(self,obstacle):
        offset = (obstacle[0], obstacle[1])
        poi = self.track_border_mask.overlap(self.obstacle_mask, offset)
        return poi

    def generate_obstacles(self,grid,player_start_pos,num_obstacles: int = 10):
        obstacles = []
        i = 0
        while i < num_obstacles:
            player_start_pos_x,player_start_pos_y =player_start_pos[0]//Grid.CELL_SIZE,player_start_pos[1]//Grid.CELL_SIZE
            start_pos_grid_node = grid.grid[player_start_pos_x][player_start_pos_y]
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

    def generate_deliveries(self,grid,player_start_pos,num_deliveries: int = 4, obstacles=[]):
        deliveries = []
        i = 0
        while i < num_deliveries:
            player_start_pos_x,player_start_pos_y =player_start_pos[0]//Grid.CELL_SIZE,player_start_pos[1]//Grid.CELL_SIZE
            start_pos_grid_node = grid.grid[player_start_pos_x][player_start_pos_y]
            x = random.randint(0, grid.width - 1)
            y = random.randint(0, grid.height - 1)
            delivery_grid_node = grid.grid[x][y]

            if start_pos_grid_node != delivery_grid_node and (not delivery_grid_node.is_blocked ) and delivery_grid_node.is_road:
                deliveries.append(
                    Delivery(delivery_grid_node)
                )
                i = i + 1

        # deliveries.append((1800,600)) - trinity location
        return deliveries

    # def draw_closest_delivery_locations(self,win,deliveries,player_car,closest_delivery=None):
    #     # Calculate camera offset
    #     offset_x = player_car.x - self.win.get_width() / 2
    #     offset_y = player_car.y - self.win.get_height() / 2
    #
    #     # Clamp the offset so we don’t scroll beyond the map
    #     max_x = self.track.get_width() - self.win.get_width()
    #     max_y = self.track.get_height() - self.win.get_height()
    #
    #     offset_x = max(0, min(offset_x, max_x))
    #     offset_y = max(0, min(offset_y, max_y))
    #
    #     time.sleep(10)
    #
    #     win.blit(self.delivery_location, (closest_delivery[0] - offset_x, closest_delivery[1] - offset_y))

    def init_delivery_queue(self,deliveries,player_car):
        if len(self.delivery_queue)>0:
            self.delivery_queue.clear()
        for delivery in deliveries:
            if delivery.delivery_state == DeliveryStates.PENDING or delivery.delivery_state == DeliveryStates.PREPARING:
                heapq.heappush(self.delivery_queue,(self.get_rider_distance_from_delivery(delivery,player_car),delivery))

    def get_closest_delivery(self,player_car):
        print(heapq.nsmallest(len(self.delivery_queue),self.delivery_queue))
        # time.sleep(10)
        return heapq.heappop(self.delivery_queue)

    def update_delivery_state_after_finding_tagret(self,deliveries,target_delivery):
        target_delivery[1].delivery_state = DeliveryStates.IN_PROGRESS
        for delivery in deliveries:
            if delivery.delivery_state == DeliveryStates.PREPARING:
                delivery.delivery_state = DeliveryStates.PENDING

    def reset(self,grid):
        obstacles = self.generate_obstacles(grid)
        deliveries = self.generate_deliveries(grid)
        start_time = time.time()
        return obstacles, deliveries, start_time

    def get_rider_distance_from_delivery(self,delivery:Delivery,player_car):
        delivery_destination = delivery.delivery_destination
        return utils.manhattan_distance(player_car.x//Grid.CELL_SIZE,player_car.y//Grid.CELL_SIZE,delivery_destination.x,delivery_destination.y)




