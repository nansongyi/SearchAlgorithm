from utils import *

def a_star_search(graph, start, goal):

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()            
        if current == goal:
            print("break")
            break            
        for next_term in graph.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next_term not in cost_so_far or new_cost < cost_so_far[next_term]:
                cost_so_far[next_term] = new_cost
                priority = new_cost + graph.compute_heuristic(next_term)
                frontier.put(next_term, priority)
                came_from[next_term] = current

    return came_from, cost_so_far

