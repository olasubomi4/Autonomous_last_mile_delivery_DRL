import joblib

from Environment.Color import Color
from Environment.GridNode import GridNode
from vehicles.Car import Car
from utils import scale_image, blit_rotate_center
import pygame
import heapq
from utils import manhattan_distance

class Grid:
    OBSTACLE = scale_image(pygame.image.load("/Users/odekunleolasubomi/PycharmProjects/Autonomous_last_mile_delivery_DRL/imgs/obstacle8.png"), 0.02)
    OBSTACLE_MASK = pygame.mask.from_surface(OBSTACLE)
    CELL_SIZE=6
    def __init__(self, width, height,track):
        self.grid = [
            [GridNode(i, j, Color.RED.value) for j in range(0, height, Grid.CELL_SIZE)]
            for i in range(0, width, Grid.CELL_SIZE)
        ]
        self.track=track
        self.width = len(self.grid)
        self.height = len(self.grid[0])

    @staticmethod
    def convert_grid_position_to_map_view(position):
        return position*Grid.CELL_SIZE
    def generate_grid(self, RED_CAR, TRACK_BORDER_MASK):
        track_surface = self.track.convert()
        temp_car = Car(RED_CAR, (0, 0), max_vel=3, rotation_vel=8)
        car_mask = pygame.mask.from_surface(temp_car.IMG)
        car_width = temp_car.IMG.get_width()
        car_height = temp_car.IMG.get_height()

        for i in range(self.width):
            for j in range(self.height):
                x = i * Grid.CELL_SIZE
                y = j * Grid.CELL_SIZE
                current_grid_node = self.grid[i][j]
                current_grid_color = track_surface.get_at((x, y))

                if current_grid_color != Color.BLACK.value and current_grid_color != Color.BLUE.value :
                    continue

                # Position the car's top-left so that its center is at (x, y)
                car_x = x - car_width // 2
                car_y = y - car_height // 2


                offset = (int(car_x - 0), int(car_y - 0))
                collision = TRACK_BORDER_MASK.overlap(car_mask, offset)

                if not collision:
                    current_grid_node.is_road = True
                    current_grid_node.is_blocked = False
                    current_grid_node.color = current_grid_color

        return self.grid

    def get_grid_node(self, x, y, obstacles, RED_CAR):
        temp_car = Car(RED_CAR, (x, y), max_vel=3, rotation_vel=8)
        car_mask = pygame.mask.from_surface(temp_car.IMG)
        car_width = temp_car.IMG.get_width()
        car_height = temp_car.IMG.get_height()

        for i in range(self.width):
            for j in range(self.height):
                x_pixel = i * Grid.CELL_SIZE
                y_pixel = j * Grid.CELL_SIZE

                if not self.grid[i][j].is_road:
                    self.grid[i][j].is_blocked = True
                    continue

                # Place car's topleft to center the image at (x_pixel, y_pixel)
                car_x = x_pixel - car_width // 2
                car_y = y_pixel - car_height // 2
                blocked = False

                # Check for collision with each obstacle
                for obs in obstacles:
                    offset_x = int(obs.x - car_x)
                    offset_y = int(obs.y - car_y)
                    if car_mask.overlap(Grid.OBSTACLE_MASK, (offset_x, offset_y)):
                        blocked = True
                        break

                self.grid[i][j].is_blocked = blocked

    def load_existing_grid(self):
        self.grid=joblib.load("grid.pkl")
        return self.grid



    def a_star_path_planning(self,start,goal,vehicle,directionx=0,directiony=0):
        open_set = []

        start = ((start[0] // Grid.CELL_SIZE)+directionx, (start[1] // Grid.CELL_SIZE)+directiony)
        goal = (goal.x // Grid.CELL_SIZE, goal.y// Grid.CELL_SIZE)

        heapq.heappush(open_set, (0+ manhattan_distance(start[0],start[1], goal[0],goal[1]), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                pixel_path = [(x * Grid.CELL_SIZE, y * Grid.CELL_SIZE) for (x, y) in path[::-1]]
                return pixel_path,current_cost


            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    neighbor_node = self.grid[neighbor[0]][neighbor[1]]
                    if neighbor_node.is_blocked==False and neighbor_node.is_road==True:
                        move_cost = self.grid[neighbor[0]][neighbor[1]].get_node_weight(vehicle)
                        tentative_g_score = g_score[current] + move_cost
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            g_score[neighbor] = tentative_g_score
                            f_score = tentative_g_score + manhattan_distance(neighbor[0], neighbor[1], goal[0], goal[1])
                            heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
                            came_from[neighbor] = current
        return None
