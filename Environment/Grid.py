import joblib

from Environment.Color import Color
from Environment.GridNode import GridNode
from vehicles.PlayerCar import PlayerCar
from utils import scale_image, blit_rotate_center
import pygame
import heapq
from utils import manhattan_distance

class Grid:
    OBSTACLE = scale_image(pygame.image.load("imgs/obstacle8.png"), 0.05)
    OBSTACLE_MASK = pygame.mask.from_surface(OBSTACLE)
    def __init__(self, width, height,track):
        self.width = width
        self.height = height
        self.grid= [[GridNode(i,j,Color.RED.value) for i in range(height)] for j in range(width)]
        self.track=track


    def generate_grid(self,RED_CAR,TRACK_BORDER_MASK):
        car = PlayerCar(RED_CAR, (0, 0), 3, 8)
        for i in range(self.width):
            for j in range(self.height):
                current_grid_node = self.grid[i][j]
                current_grid_color= self.track.convert().get_at((i,j))
                car.x, car.y = i, j
                if current_grid_color is not None and current_grid_color == Color.BLACK.value and car.collide(TRACK_BORDER_MASK):
                    current_grid_node.is_road=True
                    current_grid_node.color=Color.BLACK.value
                    self.grid[i][j]=current_grid_node
        return self.grid

    def get_grid_node(self,x,y,obstacles,RED_CAR):
        car=PlayerCar(RED_CAR, (x, y), 3, 8)
        for i in range(self.width):
            for j in range(self.height):
                car.x,car.y=i,j
                if self.grid[i][j].is_road and car.collide_with_obstacle(Grid.OBSTACLE_MASK, obstacles):
                    self.grid[i][j].is_blocked=True
                else:
                    self.grid[i][j].is_blocked=False

    def load_existing_grid(self):
        self.grid=joblib.load("grid.pkl")
        return self.grid

    def a_star_path_planning(self,start,goal,vehicle):
        open_set = []
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
                return path[::-1]

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4 directions only
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    neighbor_node = self.grid[neighbor[0]][neighbor[1]]
                    if not neighbor_node.is_blocked:
                        move_cost = self.grid[neighbor[0]][neighbor[1]].get_node_weight(vehicle)
                        tentative_g_score = g_score[current] + move_cost
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            g_score[neighbor] = tentative_g_score
                            f_score = tentative_g_score + manhattan_distance(neighbor[0], neighbor[1], goal[0], goal[1])
                            heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
                            came_from[neighbor] = current

        return None
