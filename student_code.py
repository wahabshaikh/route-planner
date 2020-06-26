'''
Initial state ---> s0
Actions (s) ---> {a1, a2, a3, ...}
Result (s,a) ---> s'
GoalTest(s) ---> True | False
Path Cost (s(i) -a(j)-> s(i+1) -a(j+1)-> s(i+2) ---> cost value (n)
                                                     where i = 0, 1, ...
                                                           j = 1, 2, ...
Step Cost(s, a, s') ---> n

f = g + h
g (path) = path cost
h (path) = h(s) = estimated distance to goal
'''                                                     

from math import hypot
from heapq import heapify, heappush, heappop


def get_estimated_distance(state, end):
    return hypot(end[0] - state[0], end[1] - state[1])


def shortest_path(M,start,goal):
    result = []
    
    explored = {}
    
    path_cost = {start: 0}
    
    frontier = []
    heapify(frontier)
    heappush(frontier, (0, start))
    
    while len(frontier) > 0:
        state = heappop(frontier)[1]
        
        if state == goal:
            break
            
        for neighbor in M.roads[state]:
            g = path_cost[state]
            h = get_estimated_distance(M.intersections[state], M.intersections[neighbor])
            
            f = g + h
            
            if neighbor not in path_cost or f < path_cost[neighbor]:
                explored[neighbor] = state
                path_cost[neighbor] = f
                heappush(frontier, (f, neighbor))
   
                         
    state = goal
    if state not in explored:
        return [goal]

    while state != start:
        result.append(state)
        state = explored[state]
    result.append(start)
    
    return list(reversed(result))