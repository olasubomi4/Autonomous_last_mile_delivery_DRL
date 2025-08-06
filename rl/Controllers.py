import pygame
import numpy as np

# Control state

class Controllers:

    def __init__(self):
        self.steering = 0.0
        self.throttle = 0.0
        self.delta = 0.1

    def get_manual_input(self):
        keys = pygame.key.get_pressed()

        # Steering control
        if keys[pygame.K_a]:
            self.steering = max(self.steering - self.delta, -1.0)
        elif keys[pygame.K_d]:
            self.steering = min(self.steering + self.delta, 1.0)
        else:
            self.steering = 0.0

        # Throttle control
        if keys[pygame.K_w]:
            self.throttle = min(self.throttle + self.delta, 1.0)
        elif keys[pygame.K_s]:
            self.throttle = max(self.throttle - self.delta, -1.0)
        else:
            self.throttle = 0.0
        return np.array([self.steering, self.throttle])

    def get_supervisors_action(self):
        keys = pygame.key.get_pressed()
        no_steering_action=False
        no_throttle_action=False
        # Steering control
        if keys[pygame.K_a]:
            self.steering = max(self.steering - self.delta, -1.0)
        elif keys[pygame.K_d]:
            self.steering = min(self.steering + self.delta, 1.0)
        else:
            no_steering_action=True
            self.steering = 0.0

        # Throttle control
        if keys[pygame.K_w]:
            self.throttle = min(self.throttle + self.delta, 1.0)
        elif keys[pygame.K_s]:
            self.throttle = max(self.throttle - self.delta, -1.0)
        else:
            no_throttle_action=True
            self.throttle = 0.0

        if no_steering_action and no_throttle_action:
            return None
        return np.array([self.steering, self.throttle])