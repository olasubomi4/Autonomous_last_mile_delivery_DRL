from vehicles.AbstractCar import AbstractCar

class PlayerCar(AbstractCar):
    START_POS = (155, 370)

    def __init__(self, img, start_pos, max_vel, rotation_vel):
        super().__init__(max_vel, rotation_vel,img,start_pos)
        self.IMG = img
        self.x, self.y = start_pos

    def reduce_speed(self):
        self.vel = max(self.vel - self.acceleration / 2, 0)
        self.move()

    def bounce(self):
        self.vel = -self.vel
        self.move()