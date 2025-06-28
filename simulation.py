import pygame
import time
import math
import pickle
import joblib

from DeliveryStates import DeliveryStates
from Environment.Grid import Grid
from utils import scale_image, blit_rotate_center
import random
from Environment.DublinCityCenter  import  DublinCityCenter
from vehicles.Delivery import Delivery

from vehicles.PlayerCar import PlayerCar
from vehicles.Vehicle import Vehicles

TRACK = scale_image(pygame.image.load("imgs/track.png"), 1)

TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 1)
TRACK_BORDER_MASK = pygame.mask.from_surface(TRACK_BORDER)

OBSTACLE = scale_image(pygame.image.load("imgs/obstacle8.png"), 0.02)
OBSTACLE_MASK = pygame.mask.from_surface(OBSTACLE)

DELIVERY_LOCATION = scale_image(pygame.image.load("imgs/delivery-locations-icon.png"), 0.05)
DELIVERY_LOCATION_MASK = pygame.mask.from_surface(DELIVERY_LOCATION)

RED_CAR = scale_image(pygame.image.load("imgs/red-car.png"), 0.3)

WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()
WIN = pygame.display.set_mode((1300, 800))

pygame.display.set_caption("Dublin city center")

FPS = 60

run = True
env = DublinCityCenter(TRACK,TRACK_BORDER,OBSTACLE,DELIVERY_LOCATION,RED_CAR,WIN,WIDTH,HEIGHT)
clock = pygame.time.Clock()
images = [(TRACK, (0, 0)), (TRACK_BORDER, (0, 0))]
player_start_pos = (220, 370)
player_car = PlayerCar(RED_CAR,player_start_pos,3, 8)
grid = Grid(WIDTH, HEIGHT, TRACK);
# f=grid.gen(WIDTH,HEIGHT)
generated_grid = grid.generate_grid(RED_CAR, TRACK_BORDER_MASK)
obstacles = env.generate_obstacles(grid,player_start_pos,num_obstacles=10)

number_of_deliveries = 10
deliveries_pending = 10
grid.get_grid_node(player_car.x, player_car.y, obstacles, RED_CAR)
deliveries:[Delivery] = env.generate_deliveries(grid,player_start_pos,number_of_deliveries)

next_level = False
start_time = time.time()
start=True
target_delivery =tuple([])
env.init_delivery_queue(deliveries,player_car,grid)
start2=True
while run:

    env.draw(WIN, images, player_car, obstacles, deliveries)
    if  deliveries_pending<=0:
        player_car.reset()
        obstacles, deliveries, start_time = env.reset(grid,player_start_pos)
        deliveries_pending=len(deliveries)
        print("You won")
        print("Resetting")

    if start:
        env.draw(WIN, images, player_car, obstacles, deliveries)
        for event in pygame.event.get():
            # time.sleep(5)
            break
        clock.tick(FPS)
        target_delivery = env.get_closest_delivery(player_car)
        target_delivery_path=target_delivery[2]
        env.update_delivery_state_after_finding_tagret(deliveries, target_delivery)
        # grid.draw_grid_lines(TRACK)
        env.draw(WIN, images, player_car, obstacles, deliveries,path=target_delivery_path)
        start=False

    clock.tick(FPS)

    if next_level:
        env.init_delivery_queue(deliveries, player_car,grid)
        target_delivery = env.get_closest_delivery(player_car)
        target_delivery_path=target_delivery[2]
        env.update_delivery_state_after_finding_tagret(deliveries, target_delivery)
        # env.draw(WIN, images, player_car, obstacles, deliveries)
        # path = grid.a_star_path_planning((int(player_car.x), int(player_car.y)), target_delivery[1].delivery_destination,
        #                                  Vehicles.CAR)
        env.draw(WIN, images, player_car, obstacles, deliveries, path=target_delivery_path)
        # env.draw(WIN, images, player_car, obstacles, deliveries)


        next_level = False


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break
    if start2:

        # joblib.dump(generated_grid,"grid.pkl")
        # print("grid dumped")
        # grid=joblib.load("grid.pkl")
        # grid.load_existing_grid()
        # path= grid.a_star_path_planning((player_car.x,player_car.y),target_delivery[1].delivery_destination,Vehicles.CAR)

        # if path is not None:
        #     joblib.dump(path,"path.pkl")
        #     clock.tick(FPS)
        #     env.draw(WIN, images, player_car, obstacles, deliveries,path=path)
        #     # for point in path:
        #     #     pygame.draw.rect(TRACK, (0, 255, 0), (point[0], point[1], 1, 1))
        #     #     pygame.display.update()
        #     print("path dumped")
        start2=False



    # color=TRACK.convert().get_at((70,70))
    #
    # print(f"color at {color}")
    env.move_player(player_car,grid)

    if player_car.collide(TRACK_BORDER_MASK):
        player_car.bounce()
        # pass

    if player_car.collide_with_obstacle(OBSTACLE_MASK, obstacles):
        player_car.reset()
        # pass
    if player_car.is_delivery_completed(DELIVERY_LOCATION_MASK, target_delivery) and player_car.vel==0:
        # player_car.reset()
        next_level = True
        print(f"Time taken: {time.time() - start_time} seconds")
        target_delivery[1].delivery_completed_time= time.time()
        target_delivery[1].delivery_state=DeliveryStates.COMPLETED

        # deliveries.remove(target_delivery)
        deliveries_pending=deliveries_pending-1

pygame.quit()
