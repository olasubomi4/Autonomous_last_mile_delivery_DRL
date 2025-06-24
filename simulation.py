import pygame
import time
import math
from utils import scale_image, blit_rotate_center
import random
from Environment.DublinCityCenter  import  DublinCityCenter

from vehicles.PlayerCar import PlayerCar

TRACK = scale_image(pygame.image.load("imgs/track.png"), 0.9)

TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 0.9)
TRACK_BORDER_MASK = pygame.mask.from_surface(TRACK_BORDER)

OBSTACLE = scale_image(pygame.image.load("imgs/obstacle8.png"), 0.05)
OBSTACLE_MASK = pygame.mask.from_surface(OBSTACLE)

DELIVERY_LOCATION = scale_image(pygame.image.load("imgs/delivery-locations-icon.png"), 0.05)
DELIVERY_LOCATION_MASK = pygame.mask.from_surface(DELIVERY_LOCATION)

RED_CAR = scale_image(pygame.image.load("imgs/red-car.png"), 0.4)

WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()
WIN = pygame.display.set_mode((1300, 800))

pygame.display.set_caption("Dublin city center")

FPS = 60

run = True
env = DublinCityCenter(TRACK,TRACK_BORDER,OBSTACLE,DELIVERY_LOCATION,RED_CAR,WIN,WIDTH,HEIGHT)
clock = pygame.time.Clock()
obstacles = env.generate_obstacles();
images = [(TRACK, (0, 0)), (TRACK_BORDER, (0, 0))]
player_car = PlayerCar(RED_CAR,(155, 370),3, 8)
deliveries = env.generate_deliveries()
next_level = False
start_time = time.time()
while run:

    if next_level:
        obstacles,deliveries,start_time=env.reset()
        next_level = False
    clock.tick(FPS)

    env.draw(WIN, images, player_car,obstacles,deliveries)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break

    env.move_player(player_car)

    if player_car.collide(TRACK_BORDER_MASK):
        # player_car.bounce()
        pass

    if player_car.collide_with_obstacle(OBSTACLE_MASK, obstacles):
        # player_car.reset()
        pass

    if player_car.collide_with_obstacle(DELIVERY_LOCATION_MASK, deliveries):
        player_car.reset()
        next_level = True
        print(f"Time taken: {time.time() - start_time} seconds")

pygame.quit()
