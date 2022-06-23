from cmath import sqrt
from pickle import FALSE
from tkinter import Widget
from turtle import width
import queue
import pygame
import math
from queue import PriorityQueue
from array import *

WIDTH = 600
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding aStar")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):  # 4D
        self.neighbors = []
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        # RIGHT
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # return abs(x1 - x2) + abs(y1 - y2)  # mahanttan distance
    return math.dist(p1, p2)  # euclidean distance
    # return max(abs(x1 - x2),abs(y1 - y2)) #chebyshev distance


def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def bfs(draw, grid, start, end):  # function for BFS
    count = 0
    open_set = PriorityQueue()
    open_set.put((count, start))
    came_from = {}
    visited = []
    open_set_hash = {start}

    while not open_set.empty():          # Creating loop to visit each node
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[1]

        visited.append(current)
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbour in current.neighbors:
            if (neighbour not in visited) and (neighbour not in open_set_hash):
                came_from[neighbour] = current
                # print( neighbour.get_pos())
                count += 1
                open_set_hash.add(neighbour)
                open_set.put((count, neighbour))
                neighbour.make_open()

        print(current.get_pos())

        draw()
        if current != start:
            current.make_closed()
    print("EMpty set")
    return False


def greedy_bfs(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()  # Candidates for next node consideration
    open_set.put((0, start))
    came_from = {}
    visited = [[False for x in range(50)]
               for y in range(50)]  # Visited nodes (Each node only gets visited once)

    # Potential cost for each node using a heuristic function
    f_score = {spot: float("inf") for row in grid for spot in row}
    # Potential cost for from starting node to destination node
    f_score[start] = h(start.get_pos(), end.get_pos())

    row, col = start.get_pos()
    visited[row][col] = True

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Get node with least potential cost in PriorityQueue
        current = open_set.get()[1]

        if current == end:  # If destination is reached
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            row, col = neighbor.get_pos()

            if not visited[row][col]:
                came_from[neighbor] = current
                # Calculate potential cost of current nodes' neighbours
                f_score[neighbor] = h(neighbor.get_pos(), end.get_pos())
                visited[row][col] = True
                open_set.put((f_score[neighbor], neighbor))
                neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False


def aStar(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + \
                    h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False


def make_grid(rows, width):  # draw object spot
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):  # column
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows, width):  # draw line of grid
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap),
                         (width, i * gap))  # horizontal line
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0),
                             (j * gap, width))  # vertical line


def draw(win, grid, rows, width):  # main draw function
    win.fill(WHITE)

    for row in grid:  # draw spot first
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()


def main(win, width):
    ROWS = 20
    grid = make_grid(ROWS, WIDTH)

    start = None
    end = None

    run = True
    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            start = grid[0][0]  # make start node at row 0 column 0
            start.make_start()

            # make end node on the bottom rigth corner
            end = grid[ROWS-1][ROWS-1]
            end.make_end()

            ### make obstacle ###
            gap = WIDTH // ROWS
            for i in [1, 5, 7, 11, 16]:
                for j in [1, 2, 3, 4, 5, 9, 10, 11, 14, 17, 19]:
                    grid[i][j].make_barrier()

            for k in [11, 16]:
                grid[k][0].make_barrier()
            for k in [10, 11, 12, 13, 14]:
                for t in [3, 5, 8, 14, 15, 18]:
                    grid[k][t].make_barrier()
                    ###########################

                    # press SPACE to start and update neighbors
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    # aStar(lambda: draw(win, grid, ROWS, width),grid,start,end)

                    # bfs(lambda: draw(win, grid, ROWS, width), grid, start, end)

                    greedy_bfs(lambda: draw(win, grid, ROWS, width),
                               grid, start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()


main(WIN, WIDTH)
