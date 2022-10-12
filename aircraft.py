"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""
import time as timer
import math
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location
from cbs import detect_collision, detect_collisions

def calc_distance(path1, path2, time):          #change this to a heuristics based function?
    loc1 = get_location(path1, time)
    loc2 = get_location(path2, time)
    distance = math.sqrt((loc1[0]-loc2[0])**2 + (loc1[1]-loc2[1])**2)
    return distance

class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = heuristics
        
        
    def find_individual_solution(self, constraints):
        return a_star(self.my_map, self.start, self.goal, self.heuristics, self.id, constraints)
    

        
    def radar(self, radar_loc, radar_range):
        #for t in range(len(radar_loc[self.id])):
        for j in range(len(radar_loc)):
            if self.id != j:
                if calc_distance(radar_loc[self.id], radar_loc[j], 0) > radar_range:
                    continue
                else:
                    coll = detect_collision(radar_loc[self.id], radar_loc[j])
                    
                    while coll[0] != None:
                    
                    
                        print("agent1: ",self.id,"agent2: ", j, "timesteps in the future: ", coll[-1] ' future collision should be resolved')
                        #resolve conflict here
                        constraint = {'agent': self.id, 'loc': coll[0], 'timestep': coll[-1]}
                        constraints.append(constraint)
                        
                        
                        
                        coll = detect_collision(radar_loc[self.id], radar_loc[j])
                        
                        
                    #change main paths here