import pygame
import time

from DeliveryStates import DeliveryStates
from Environment.Grid import Grid
from utils import scale_image
from Environment.DublinCityCenter  import  DublinCityCenter
from vehicles.Delivery import Delivery
from vehicles.Car import Car


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

delivery_vehicle = Car(RED_CAR, player_start_pos, 3, 8)


grid = Grid(WIDTH, HEIGHT, TRACK);

generated_grid = grid.generate_grid(RED_CAR, TRACK_BORDER_MASK)
obstacles = env.generate_obstacles(grid, player_start_pos, num_obstacles=10)

number_of_deliveries = 10
deliveries_pending = 10
grid.get_grid_node(delivery_vehicle.x, delivery_vehicle.y, obstacles, RED_CAR)
deliveries:[Delivery] = env.generate_deliveries(grid, player_start_pos, number_of_deliveries)

next_level = False
start_time = time.time()
start=True
target_delivery =tuple([])
env.init_delivery_queue(deliveries, delivery_vehicle, grid)
while run:
    clock.tick(FPS)
    env.draw(WIN, images, delivery_vehicle, obstacles, deliveries)
    #RESET/ done
    if  deliveries_pending<=0:
        delivery_vehicle.reset()
        obstacles, deliveries, start_time = env.reset(grid, player_start_pos)
        deliveries_pending=len(deliveries)
        print("You won")
        print("Resetting")

    if start:
        #init
        env.draw(WIN, images, delivery_vehicle, obstacles, deliveries)
        time.sleep(5)
        target_delivery = env.get_closest_delivery(delivery_vehicle)
        target_delivery_path=target_delivery[2]
        env.update_delivery_state_after_finding_tagret(deliveries, target_delivery)
        env.draw(WIN, images, delivery_vehicle, obstacles, deliveries, path=target_delivery_path)
        start=False



    #STEP ON A SPECIFIC CONDITION
    if next_level:
        env.init_delivery_queue(deliveries, delivery_vehicle, grid)
        target_delivery = env.get_closest_delivery(delivery_vehicle)
        target_delivery_path=target_delivery[2]
        env.update_delivery_state_after_finding_tagret(deliveries, target_delivery)
        env.draw(WIN, images, delivery_vehicle, obstacles, deliveries, path=target_delivery_path)
        next_level = False


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break

    env.move_vehicle(delivery_vehicle, grid)
    #NORMAL STEP
    sensor_targets= [(OBSTACLE_MASK,obstacles),(TRACK_BORDER_MASK,[grid.grid[0][0]])]
    sensor_result=delivery_vehicle.update_sensors(sensor_targets)
    # print(sensor_result)

    delivery_vehicle.draw_sensors(WIN)

    if delivery_vehicle.collide(TRACK_BORDER_MASK):
        delivery_vehicle.bounce()
        # pass

    if delivery_vehicle.collide_with_obstacle(OBSTACLE_MASK, obstacles):
        delivery_vehicle.reset()
        # pass

    if delivery_vehicle.is_delivery_completed(DELIVERY_LOCATION_MASK, target_delivery) and delivery_vehicle.vel==0:
        next_level = True
        print(f"Time taken: {time.time() - start_time} seconds")
        target_delivery[1].delivery_completed_time= time.time()
        target_delivery[1].delivery_state=DeliveryStates.COMPLETED
        deliveries_pending=deliveries_pending-1
    pygame.display.update()
pygame.quit()
