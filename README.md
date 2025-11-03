# a_star_path_planner
A lightweight Python implementation of the A* pathfinding algorithm with matplotlib visualization.

## Dependecies 
pip3 install matplotlib

## How it works 
- The planner computes the optimal path from the current position to the goal using a grid-based search.
It evaluates cost based on:
f(n)=g(n)+h(n)

where:

- g(n) → cost from start to node

- h(n) → Euclidean or Manhattan distance to goal