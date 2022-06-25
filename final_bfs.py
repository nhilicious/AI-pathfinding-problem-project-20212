from cmath import sqrt
from pickle import FALSE
from tkinter import Widget
from turtle import width
import queue
import pygame
import math
from queue import PriorityQueue, Queue
from array import *

WIDTH = 600 # size of pygame window
WIN = pygame.display.set_mode((WIDTH, WIDTH)) # create a pygame window, which is a square of side 'WIDTH' pixels

# set title for pygame window
pygame.display.set_caption("BFS Path Finding Algorithm")
# pygame.display.set_caption("Greedy BFS Path Finding Algorithm")
# pygame.display.set_caption("A* Path Finding Algorithm")

# colors in RGB 
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
		# draw a rectangle in win
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		self.neighbors = []  # a list storing neighbor of a spot
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


def h(p1, p2): # heuristic function
	x1, y1 = p1
	x2, y2 = p2
	# return abs(x1 - x2) + abs(y1 - y2) # mahanttan distance
	return math.dist(p1, p2)  # euclidean distance
	# return max(abs(x1 - x2),abs(y1 - y2)) # chebyshev distance


def reconstruct_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()


def bfs(draw, grid, start, end):  # function for BFS
    count = 0
    open_set = Queue()  # Candidates for next node consideration
    open_set.put((count, start))
    came_from = {}

    # Visited nodes (Each node only gets visited once)
    # Nodes is marked "Visited" if it is put in the PriorityQueue
    visited = []
    open_set_hash = {start}  # Nodes already considered

    while not open_set.empty():          # Creating loop to visit each node
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        # This would be chosen as the next node to travel through
        current = open_set.get()[1]

        visited.append(current)
        if current == end: # If destination is reached
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbour in current.neighbors: # Consider each neighbour of current node
            if (neighbour not in visited) and (neighbour not in open_set_hash):
                came_from[neighbour] = current
                # print( neighbour.get_pos())
                count += 1

                # Add neighbour node to already considered list
                open_set_hash.add(neighbour)

                # Add neighbour node to Queue
                open_set.put((count, neighbour))

                # Set the next to be considered node as open
                neighbour.make_open()
        # Draw xD
        draw()

        # Keep the color of the start node (not change to red)
        if current != start:
            current.make_closed()
    return False


def make_grid(rows, width):
	grid = [] # grid is 2D array whose elements are object of class Spot
	gap = width // rows

	for i in range(rows):
		grid.append([])
		for j in range(rows):  # column
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, width):  # draw line of grid on 'win'
	gap = width // rows # khoang cach 1 o vuong
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap),
		                 (width, i * gap))  # horizontal line
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))  # vertical line


def draw(win, grid, rows, width):  # main draw function
	win.fill(WHITE)  # fill the screen with WHITE

	for row in grid:  
		for spot in row:
			spot.draw(win) # draw spots on pygame window 'win'

	draw_grid(win, rows, width)  # draw grid line
	pygame.display.update()  # update the drawing on 'win'


def get_clicked_pos(pos, rows, width):  # get clicked position
	gap = width // rows  # width and height of a spot
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col # return the row and column of clicked position


def main(win, width):
	ROWS = 20
	grid = make_grid(ROWS, width)  # generate the gird

	start = None
	end = None

	run = True
	while run:
		draw(win, grid, ROWS, width)
		for event in pygame.event.get():  # loop through the events happened

			if event.type == pygame.QUIT:  # press the X button at the top righthand corner of the screen
				run = False

			# add fixed start and goal
			start = grid[4][4]
			start.make_start()
			end = grid[11][11]
			end.make_end()

			# # add fixed obstacle
			grid[12][4].make_barrier()
			grid[11][4].make_barrier()
			grid[10][4].make_barrier()
			grid[7][11].make_barrier()
			grid[8][11].make_barrier()
			grid[9][9].make_barrier()
			grid[10][9].make_barrier()
			grid[11][9].make_barrier()



			if pygame.mouse.get_pressed()[0]:  # '0' means the LEFT MOUSE button
				pos = pygame.mouse.get_pos()  # get the mouse cursor position
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				if not start and spot != end:  # if the start point has not been placed yet
					start = spot
					start.make_start()

				elif not end and spot != start:  # if the end point has not been placed yet
					end = spot
					end.make_end()

				elif spot != end and spot != start:  # make spot the barrier
					spot.make_barrier()

			# right click the spot to reset its state
			elif pygame.mouse.get_pressed()[2]:  # '2' means the RIGHT MOUSE button
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:  # when a key button is pressed and released
				# press the SPACE button and both start and end point have already been initialized
				if event.key == pygame.K_SPACE and start and end:
					# reset state of spot that is not the start/end/barrier
					for row in grid:
						for spot in row:
							if spot.is_start() == 0 and spot.is_end() == 0 and spot.is_barrier() == 0:
								spot.reset()


					#update neighbor list of each spot
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)

					#algorithms
					# aStar(lambda: draw(win, grid, ROWS, width), grid, start, end)
					bfs(lambda: draw(win, grid, ROWS, width), grid, start, end)
					# greedy_bfs(lambda: draw(win, grid, ROWS, width), grid, start, end)

				if event.key == pygame.K_c:  # press C button to restart
					start = None
					end = None
					grid = make_grid(ROWS, width)
					# can replaced by resetting all spot ?

	pygame.quit()

main(WIN, WIDTH)
