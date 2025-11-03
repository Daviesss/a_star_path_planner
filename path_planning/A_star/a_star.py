#!/usr/bin/env python3
from matplotlib import pyplot as plt
import numpy as np
import heapq
import math


# Geting the total cost to end/final destination
# f(n) = g(n) + h(n)
# f(n) = total cost to reach the goal
# g(n) = cost from start to current node
# h(n) = heuristic cost estimate from current node to goal (Euclidean distance)
# h(n) = sqrt((x2 - x1)^2 + (y2 - y1)^2)

grid = [
    # columns
    [0, 1, 0, 0, 0],  # rows
    [0, 1, 0, 1, 0], 
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

start = (0, 0)  # Starting point
goal = (3, 4)   # Goal point 

# class to visualize A* pathfinding algorithm
class AStarVisualizer:
    def __init__(self,start, goal, grid):
        # getting the grid_size, start , goal and obstacles
        self.grid = grid
        self.grid_size = (len(grid), len(grid[0])) # grid size
        self.start = start # starting point
        self.goal = goal # final destination
        # self.grid = np.zeros(grid_size) # initializing the grid
        self.path = []
        self.open_set = set() # set of nodes to be evaluated
        self.closed_set = set() # set of nodes already evaluated
     
     
    
    # Visualizing the grid with obstacles, start and goal   
    def plot_grid(self):
        """Visualize the grid with obstacles, start, and goal"""
        grid_array = np.array(self.grid)

        plt.imshow(grid_array, cmap='Greys', origin='upper')

        # Plot start (green) and goal (red)
        plt.scatter(self.start[1], self.start[0], marker='o', color='green', s=200, label='Start')
        plt.scatter(self.goal[1], self.goal[0], marker='x', color='red', s=200, label='Goal')
        
        
                
        # Plot the path if it exists
        if self.path:
            for (r, c) in self.path:
                plt.scatter(c, r, color='blue', s=100)

        
        plt.title('A* Grid Visualization')
        plt.legend(loc='upper right')
        plt.grid(True, color='black', linewidth=0.5)
        plt.show()
       
       
    
    # heuristic function to calculate Euclidean distance
    def heuristic(self,node):
        """Calculate the Euclidean distance heuristic"""
        x1,y1 = node
        x2,y2 = self.goal 
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
      
      
    def get_neighbors(self,node):
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        neighbors = []
        rows , cols = self.grid_size
        for dx , dy in moves:
            x, y = node[0] + dx , node[1] + dy
            if 0 <=x < rows and 0 <=y < cols and self.grid[x][y] == 0:
                neighbors.append((x,y))
        return neighbors
    
    
    # A* search algorithm implementation
    def a_star_search(self):
        open_heap = []
        heapq.heappush(open_heap,(0 + self.heuristic(self.start), self.start))
        
        came_from = {}
        g_score = {self.start: 0}
        
        
        while open_heap:
            current_f , current = heapq.heappop(open_heap)
            
            if current == self.goal:
                self.path = self.reconstruct_path(came_from, current)
                return self.path
                
            self.closed_set.add(current)
            
            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue 
                
                tentative_g  = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self . heuristic(neighbor)
                    heapq.heappush(open_heap, (f_score, neighbor))  
                    came_from[neighbor] = current
                    self.open_set.add(neighbor)     
        return None
    
    
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
            

a_star = AStarVisualizer(start, goal, grid)
path = a_star.a_star_search()
print("Shortest path:", path)
a_star.plot_grid()

    
  