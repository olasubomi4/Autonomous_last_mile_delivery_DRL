import pygame
import time
import math
from utils import scale_image, blit_rotate_center
import random

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

def draw(win, images, player_car,obstacles=None,deliveries=None):
    # Calculate camera offset
    offset_x = player_car.x - WIN.get_width() / 2
    offset_y = player_car.y - WIN.get_height() / 2

    # Clamp the offset so we don’t scroll beyond the map
    max_x = TRACK.get_width() - WIN.get_width()
    max_y = TRACK.get_height() - WIN.get_height()

    offset_x = max(0, min(offset_x, max_x))
    offset_y = max(0, min(offset_y, max_y))

    # Draw each image with the offset applied
    for img, pos in images:
        win.blit(img, (pos[0] - offset_x, pos[1] - offset_y))

    for obstacle in obstacles:
        win.blit(OBSTACLE, (obstacle[0] - offset_x, obstacle[1] - offset_y))

    for delivery in deliveries:
        win.blit(DELIVERY_LOCATION, (delivery[0] - offset_x, delivery[1] - offset_y))

    # Always draw the car at window center (or appropriate offset if clamped at edge)
    car_draw_x = player_car.x - offset_x
    car_draw_y = player_car.y - offset_y
    blit_rotate_center(win, player_car.img, (car_draw_x, car_draw_y), player_car.angle)

    pygame.display.update()



def move_player(player_car):
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



def isObstacleOnTheWay(obstacle):
    offset = (obstacle[0], obstacle[1])
    poi = TRACK_BORDER_MASK.overlap(OBSTACLE_MASK, offset)
    return poi

def generate_obstacles(num_obstacles: int = 10):
    obstacles = []
    i = 0

    while i < num_obstacles:
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        if isObstacleOnTheWay ((x,y)) == None:
            obstacles.append(
                (x, y)
            )
            i = i + 1
    return obstacles

def generate_deliveries(num_deliveries: int = 1, Obstacles=None):
    deliveries = []
    i = 0
    while i < num_deliveries:
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        if isObstacleOnTheWay ((x,y)) == None and (x,y) not in obstacles:
            deliveries.append(
                (x, y)
            )
            i = i + 1

    # deliveries.append((1800,600))
    return deliveries


run = True
clock = pygame.time.Clock()
obstacles = generate_obstacles();
images = [(TRACK, (0, 0)), (TRACK_BORDER, (0, 0))]
player_car = PlayerCar(RED_CAR,(155, 370),3, 8)
deliveries = generate_deliveries()
next_level = False
start_time = time.time()
while run:

    if next_level:
        obstacles = generate_obstacles()
        deliveries = generate_deliveries()
        start_time = time.time()
        next_level = False
    clock.tick(FPS)

    draw(WIN, images, player_car,obstacles,deliveries)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break

    # time.sleep(10)
    #
    # WIN.blit(env.delivery_location, (delivery[0] - offset_x, delivery[1] - offset_y))

    move_player(player_car)



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
