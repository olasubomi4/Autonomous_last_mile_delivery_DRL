import pygame
import time
import math
from utils import scale_image, blit_rotate_center

TRACK = scale_image(pygame.image.load("imgs/track.png"), 0.9)

TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 0.9)
TRACK_BORDER_MASK = pygame.mask.from_surface(TRACK_BORDER)


OBSTACLES = scale_image(pygame.image.load("imgs/obstacles.png"), 0.55)
RED_CAR = scale_image(pygame.image.load("imgs/red-car.png"), 0.55)

WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()
WIN = pygame.display.set_mode((1300, 800))


pygame.display.set_caption("Dublin city center")

FPS = 60


class AbstractCar:
    def __init__(self, max_vel, rotation_vel):
        self.img = self.IMG
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.angle = 0
        self.x, self.y = self.START_POS
        self.acceleration = 0.1

    def rotate(self, left=False, right=False):
        if left:
            self.angle += self.rotation_vel
        elif right:
            self.angle -= self.rotation_vel

    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

    def move_forward(self):
        self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()

    def move_backward(self):
        self.vel = max(self.vel - self.acceleration, -self.max_vel/2)
        self.move()

    def move(self):
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * self.vel
        horizontal = math.sin(radians) * self.vel

        self.y -= vertical
        self.x -= horizontal

    def  collide(self, mask, x=0, y=0):
        car_mask = pygame.mask.from_surface(self.img)
        offset = (int(self.x - x), int(self.y - y))
        poi = mask.overlap(car_mask, offset)
        return poi

    def reset(self):
        self.x, self.y = self.START_POS
        self.angle = 0
        self.vel = 0


class PlayerCar(AbstractCar):
    IMG = RED_CAR
    START_POS = (160, 280)

    def reduce_speed(self):
        self.vel = max(self.vel - self.acceleration / 2, 0)
        self.move()

    def bounce(self):
        self.vel = -self.vel
        self.move()



def draw(win, images, player_car):
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

    # Always draw the car at window center (or appropriate offset if clamped at edge)
    car_draw_x = player_car.x - offset_x
    car_draw_y = player_car.y - offset_y
    blit_rotate_center(win, player_car.img, (car_draw_x, car_draw_y), player_car.angle)

    pygame.display.update()


# def draw(win, images, player_car):
#     for imgs, pos in images:
#         win.blit(imgs, pos)
#
#     player_car.draw(win)
#     pygame.display.update()



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


run = True
clock = pygame.time.Clock()
images = [(TRACK, (0, 0)), (TRACK_BORDER, (0, 0))]
player_car = PlayerCar(3, 8)

while run:
    clock.tick(FPS)

    draw(WIN, images, player_car)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break

    move_player(player_car)

    if player_car.collide(TRACK_BORDER_MASK) == None:
        player_car.bounce()


pygame.quit()
