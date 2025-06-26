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
    CELL_SIZE=15
    def __init__(self, width, height,track):
        # self.width = width
        # self.height = height
        self.grid = [
            [GridNode(i, j, Color.RED.value) for j in range(0, height, Grid.CELL_SIZE)]
            for i in range(0, width, Grid.CELL_SIZE)
        ]
        self.track=track
        self.width = len(self.grid)
        self.height = len(self.grid[0])
        # self.visualize_grid()


    # def generate_grid(self,RED_CAR,TRACK_BORDER_MASK):
    #     car = PlayerCar(RED_CAR, (0, 0), 3, 8)
    #     for i in range(self.width):
    #         for j in range(self.height):
    #             current_grid_node = self.grid[i][j]
    #             current_grid_color= self.track.convert().get_at((i,j))
    #             car.x, car.y = i, j
    #             if current_grid_color is not None and current_grid_color == Color.BLACK.value and car.collide(TRACK_BORDER_MASK):
    #                 current_grid_node.is_road=True
    #                 current_grid_node.color=Color.BLACK.value
    #                 self.grid[i][j]=current_grid_node
    #     return self.grid
    def generate_grid(self, RED_CAR, TRACK_BORDER_MASK):
        track_surface = self.track.convert()
        car = PlayerCar(RED_CAR, (0, 0), 3, 8)
        for i in range(self.width):
            for j in range(self.height):
                x = i * Grid.CELL_SIZE
                y = j * Grid.CELL_SIZE
                current_grid_node = self.grid[i][j]
                current_grid_color = track_surface.get_at((x, y))

                car.x, car.y = x, y
                if current_grid_color is not None and current_grid_color == Color.BLACK.value and not car.collide(
                        TRACK_BORDER_MASK):
                    current_grid_node.is_road = True
                    current_grid_node.color = Color.BLACK.value
        return self.grid

    # def get_grid_node(self,x,y,obstacles,RED_CAR):
    #     car=PlayerCar(RED_CAR, (x, y), 3, 8)
    #     for i in range(self.width):
    #         for j in range(self.height):
    #             car.x,car.y=i,j
    #             if self.grid[i][j].is_road and car.collide_with_obstacle(Grid.OBSTACLE_MASK, obstacles):
    #                 self.grid[i][j].is_blocked=True
    #             else:
    #                 self.grid[i][j].is_blocked=False
    def get_grid_node(self,x,y,obstacles,RED_CAR):
        car=PlayerCar(RED_CAR, (x, y), 3, 8)
        for i in range(self.width):
            for j in range(self.height):
                x = i * Grid.CELL_SIZE
                y = j * Grid.CELL_SIZE
                car.x, car.y = x, y
                if self.grid[i][j].is_road:
                    self.grid[i][j].is_blocked = car.collide_with_obstacle(Grid.OBSTACLE_MASK, obstacles)
                else:
                    self.grid[i][j].is_blocked = True

    def load_existing_grid(self):
        self.grid=joblib.load("grid.pkl")
        return self.grid

    def visualize_grid(self):
        for i in range(self.width):
            for j in range(self.height):
                pygame.draw.rect(self.track, self.grid[i][j].color, (i * Grid.CELL_SIZE, j * Grid.CELL_SIZE, Grid.CELL_SIZE, Grid.CELL_SIZE))
        pygame.display.update()

    def a_star_path_planning(self,start,goal,vehicle):
        open_set = []

        start = (start[0] // Grid.CELL_SIZE, start[1] // Grid.CELL_SIZE)
        goal = (goal[0] // Grid.CELL_SIZE, goal[1] // Grid.CELL_SIZE)

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
                return pixel_path

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4 directions only
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    neighbor_node = self.grid[neighbor[0]][neighbor[1]]
                    # if neighbor_node.is_road and neighbor_node.is_blocked==False:
                    if not neighbor_node.is_blocked:
                        move_cost = self.grid[neighbor[0]][neighbor[1]].get_node_weight(vehicle)
                        tentative_g_score = g_score[current] + move_cost
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            g_score[neighbor] = tentative_g_score
                            f_score = tentative_g_score + manhattan_distance(neighbor[0], neighbor[1], goal[0], goal[1])
                            heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
                            came_from[neighbor] = current

        return None
