import pygame
from utils import scale_image, blit_rotate_center
import random
import time
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

    def generate_masks(self):
        TRACK_BORDER_MASK = pygame.mask.from_surface(self.track_border)
        OBSTACLE_MASK = pygame.mask.from_surface(self.obstacle)
        DELIVERY_LOCATION_MASK = pygame.mask.from_surface(self.delivery_location)

        return TRACK_BORDER_MASK,OBSTACLE_MASK,DELIVERY_LOCATION_MASK

    def draw(self, win, images, player_car, obstacles=None, deliveries=None):
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
            win.blit(self.obstacle, (obstacle[0] - offset_x, obstacle[1] - offset_y))

        for delivery in deliveries:
            win.blit(self.delivery_location, (delivery[0] - offset_x, delivery[1] - offset_y))

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

    def generate_obstacles(self,num_obstacles: int = 10):
        obstacles = []
        i = 0

        while i < num_obstacles:
            x = random.randint(0, self.width)
            y = random.randint(0, self.height)
            if self.isObstacleOnTheWay((x, y)) == None:
                obstacles.append(
                    (x, y)
                )
                i = i + 1
        return obstacles

    def generate_deliveries(self,num_deliveries: int = 1, obstacles=[]):
        deliveries = []
        i = 0
        while i < num_deliveries:
            x = random.randint(0, self.width)
            y = random.randint(0, self.height)
            if self.isObstacleOnTheWay((x, y)) == None and (x, y) not in obstacles:
                deliveries.append(
                    (x, y)
                )
                i = i + 1

        # deliveries.append((1800,600)) - trinity location
        return deliveries

    def reset(self):
        obstacles = self.generate_obstacles()
        deliveries = self.generate_deliveries()
        start_time = time.time()
        return obstacles, deliveries, start_time
