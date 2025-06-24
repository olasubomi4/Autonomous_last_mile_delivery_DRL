from utils import scale_image, blit_rotate_center
import pygame
import time
import math

class AbstractCar:
    def __init__(self, max_vel, rotation_vel,img,start_pos):
        self.img = img
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.angle = 0
        self.x, self.y = start_pos
        self.acceleration = 0.1
        self.original_image = img
        self.rect = self.img.get_rect(center=(400, 300))
        self.start_pos = start_pos

     #for rl the rotation_speed can be considered as the steering angle.
    def rotate(self, left=False, right=False, rotation_speed=-5):
        rotation_speed=abs(rotation_speed)
        if left:
            self.angle += min(rotation_speed, self.rotation_vel)
        elif right:
            self.angle -= min(rotation_speed, self.rotation_vel)

    # def rotate(self, left=False, right=False,rotation_speed=-1):
    #     rotation_speed=abs(rotation_speed)
    #     if left:
    #         self.angle += min(rotation_speed,self.rotation_vel)
    #     elif right:
    #         self.angle -= min(rotation_speed,self.rotation_vel)
    #
    #     self.image = pygame.transform.rotate(self.original_image, self.angle)
    #     self.rect = self.image.get_rect(center=self.rect.center)

    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

    def move_forward(self):
        self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()

    def move_forward_rl(self,acceleration):
        self.vel = min(self.vel + acceleration, self.max_vel)
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

    def collide_with_obstacle(self, obstacle_mask, obstacles):
        for obstacle in obstacles:
            if self.collide(obstacle_mask, obstacle[0], obstacle[1]) != None:
                return True
        return False


    def reset(self):
        self.x, self.y = self.start_pos
        self.angle = 0
        self.vel = 0
