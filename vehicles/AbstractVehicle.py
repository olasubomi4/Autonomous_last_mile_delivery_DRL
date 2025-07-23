from utils import scale_image, blit_rotate_center
import pygame
import time
import math
from Constant import Constant
from vehicles import Delivery


class AbstractVehicle:
    def __init__(self, max_vel, rotation_vel,img,start_pos,vehicle):
        self.img = img
        self.max_vel = max_vel
        self.max_speed=max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.start_angle=0
        self.angle = self.start_angle
        self.x, self.y = start_pos
        self.acceleration = 0.1
        self.original_image = img
        self.rect = self.img.get_rect(center=(400, 300))
        self.start_pos = start_pos
        self.vehicle=vehicle
        self.sensor_angles = [120,90,70]
        # self.sensor_angles = [120, 90, 60,0,180,270,300,240]
        self.sensor_max_len = Constant.MAX_SENSOR_DISTANCE                # pixels
        self.sensor_values = [self.sensor_max_len] * len(self.sensor_angles)
        self.car_center=self.rect.center

        #for rl the rotation_speed can be considered as the steering angle.
    def rotate(self, left=False, right=False, rotation_speed=-5):
        rotation_speed=abs(rotation_speed)
        if left:
            self.angle += min(rotation_speed, self.rotation_vel)
        elif right:
            self.angle -= min(rotation_speed, self.rotation_vel)

    def update_vehicle_start_position(self,new_start_pos):
        self.start_pos = [new_start_pos[0],new_start_pos[1]]
        self.start_angle = new_start_pos[2]



    def rotate_rl (self, rotation_speed=-5):
        isLeft=False
        if rotation_speed<0:
            isLeft=True
        rotation_speed=abs(rotation_speed)
        if isLeft:
            self.angle += min(rotation_speed, self.rotation_vel)
        else:
            self.angle -= min(rotation_speed, self.rotation_vel)

        self.angle = (self.angle) % Constant.MAX_STEERING_ANGLE

        # if self.angle is None or :
        #     print("angle is none")

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
        self.car_center=blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

    def move_forward(self,grid_node):

        if grid_node is not None:
            max_speed=grid_node.get_speed_limit_for_car(self.vehicle,self.max_vel)
            self.vel = min(self.vel + self.acceleration, max_speed)
            # print(f"current speed {self.vel} --- current max speed {max_speed}")
        else:
            self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()

    def move_forward_rl(self,acceleration,grid_node):
        if grid_node is not None:
            self.max_speed = grid_node.get_speed_limit_for_car(self.vehicle, self.max_vel)
            self.vel = min(self.vel + acceleration, self.max_speed)
            # print(f"current speed {self.vel} --- current max speed {max_speed}")
        else:
            self.vel = min(self.vel + acceleration, self.max_vel)
        self.move()

    def move_backward(self,grid_node):
        if grid_node is not None:
            self.max_speed=grid_node.get_speed_limit_for_car(self.vehicle,self.max_vel)
            self.vel = max(self.vel - self.acceleration,-self.max_speed/2)
        else:
            self.vel = max(self.vel - self.acceleration, -self.max_vel/2)
        self.move()

    def move_backward_rl(self, acceleration, grid_node):
        if grid_node is not None:
            max_speed = grid_node.get_speed_limit_for_car(self.vehicle, self.max_vel)
            self.vel = max(self.vel + acceleration, max_speed)
            # print(f"current speed {self.vel} --- current max speed {max_speed}")
        else:
            self.vel = max(self.vel + acceleration, self.max_vel)
        self.move()

    def move(self):
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * self.vel
        horizontal = math.sin(radians) * self.vel

        self.y -= vertical
        self.x -= horizontal

    def collide(self, mask, x=0, y=0):
        car_mask = pygame.mask.from_surface(self.img)
        try:
            offset = (int(self.x - x), int(self.y - y))
        except Exception as e:
            print(e)
        poi = mask.overlap(car_mask, offset)
        return poi

    def collide_with_obstacle(self, obstacle_mask, obstacles):
        for obstacle in obstacles:
            if self.collide(obstacle_mask, obstacle.x, obstacle.y) is not None:
                return True
        return False

    def is_delivery_completed(self, delivery_mask, target_delivery:Delivery):
        target_delivery_location= target_delivery[1].delivery_destination
        if self.collide(delivery_mask, target_delivery_location.x, target_delivery_location.y) != None:
            return True
        return False


    def reset(self):
        self.x, self.y = self.start_pos
        self.angle = self.start_angle
        self.vel = 0

    def _cast_single_sensor(self, angle, sensor_targets):
        #I need to positions of the things I am looking for
        rad = math.radians(self.angle + angle)
        hypotenuse_x, hypotenuse_y = math.cos(rad), math.sin(rad)

        x, y = self.x + self.img.get_width() // 2, self.y + self.img.get_height() // 2
        is_hit=0
        for d in range(0, self.sensor_max_len, 2):
            for sensor_target in sensor_targets:
                m, positions = sensor_target
                for a in positions:
                    try:
                    # I perform offset subtraction here
                        px, py = int(x + hypotenuse_x * d)-a.x , int(y - hypotenuse_y * d)-a.y
                        # for m in masks:
                        if 0 <= px < m.get_size()[0] and 0 <= py < m.get_size()[1]:
                            if m.get_at((px, py)):
                                is_hit=1
                                return d,angle,is_hit
                    except Exception as e:
                        print(f"error at {a}")
                        print(e)
                        continue
        return self.sensor_max_len,angle,is_hit

    def update_sensors(self, sensor_targets):

        result= [self._cast_single_sensor(a, sensor_targets)
                              for a in self.sensor_angles]
        self.sensor_values = [distance for distance, _, _ in result]
        return result

    def draw_sensors(self, win):
        x0 = self.car_center[0]
        y0 = self.car_center[1]

        for ang, dist in zip(self.sensor_angles, self.sensor_values):
            rad = math.radians(self.angle + ang)
            x1 = x0 + math.cos(rad) * dist
            y1 = y0 - math.sin(rad) * dist
            pygame.draw.line(win, (255, 255, 225), (x0, y0), (x1, y1), 2)
            pygame.draw.circle(win, (255, 50, 50), (int(x1), int(y1)), 3)
            pygame.display.update()
