"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""
import time as timer
import math
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location
from cbs import detect_collision, detect_collisions

radar = 9
timeradar = radar


def calc_distance(path1, path2, time):  # change this to a heuristics based function?
    loc1 = get_location(path1, time)
    loc2 = get_location(path2, time)
    distance = math.sqrt((loc1[0]-loc2[0])**2 + (loc1[1]-loc2[1])**2)
    return distance


def returnradar(agent, paths, time, timeradar=timeradar):
    radar_loc = []

    for j in range(len(paths)):
        agent_loc = []

        for k in range(time, time + timeradar):
            agent_loc.append(get_location(paths[j], k))

        radar_loc.append(agent_loc)
    return radar_loc


def detect_stalemate(path1, path2, t):

    if path1[t-1] == path1[t] and path2[t-1] == path2[t]:
        return True
    else:
        return False


class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id, path, constraints):
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
        self.path = path
        self.constraints = constraints
        self.it = 0

    def find_individual_solution(self, constraints):
        self.path = a_star(self.my_map, self.start, self.goal, self.heuristics, self.id, constraints)
        return self.path

    def radar(self, radar_loc, radar_range, agent_objects, t):

        for constraint in self.constraints:
            
            if t == constraint["timestep"]-1 or t == constraint["timestep"]-2:
                third_agent = agent_objects[constraint["constrained_by"]]

                temp_constraints = self.constraints.copy()
                temp_constraints.remove(constraint)

                if t < len(self.path):
                    future = a_star(self.my_map, self.path[t], self.goal, self.heuristics,
                                    self.id, temp_constraints, timestep=t)
                    if future != None:
                        new_path = self.path[:t] + future
                    else:
                        new_path = self.path
                else:
                    new_path = self.path

                if detect_collision(new_path, third_agent.path)[0] != None:
                    continue
                else:

                    self.constraints = temp_constraints
                self.path = new_path

        for j in range(len(radar_loc)):
            second_agent = agent_objects[j]

            if self.id != j:
                if calc_distance(radar_loc[self.id], radar_loc[j], 0) > radar_range:
                    continue
                else:

                    coll = detect_collision(radar_loc[self.id], radar_loc[j])

                    while coll[0] != None and self.it < 200:
                        self.it += 1

 
                        
                        if self.it < 10 or self.it % 100 == 0:
                            print(self.it, "iterations")
                            print("agent1: ", self.id, "agent2: ", j, "timesteps in the future: ", coll[-1])
                   

                        constraint1 = [{'agent': self.id, 'constrained_by': j, 'loc': coll[0], 'timestep': coll[-1]+t}]

                        loc2 = []

                        if len(coll[0]) == 1:
                            loc2 = coll[0]

                        if len(coll[0]) == 2:
                            loc2.append(coll[0][1])
                            loc2.append(coll[0][0])

                        constraint2 = [{'agent': j, 'constrained_by': self.id, 'loc': loc2, 'timestep': coll[-1]+t}]

                        if t < len(self.path):
                            future = a_star(self.my_map, self.path[t], self.goal, self.heuristics,
                                            self.id, self.constraints + constraint1, timestep=t)
                            if future != None:
                                # take path up to current point and add the future path
                                new_path = self.path[:t] + future
                            else:
                                new_path = self.path
                                constraint1 = []

                        # when the agent has reached its goal:
                        elif t >= len(self.path):
                            future = a_star(self.my_map, self.path[-1], self.goal, self.heuristics,
                                            self.id, self.constraints + constraint1, timestep=t)
                            if future != None:
                                new_path = self.path[:t] + future
                            else:
                                new_path = self.path
                                constraint1 = []

                        else:
                            new_path = self.path
                            constraint1 = []
                            print("did not work as planned")

                        if t < len(second_agent.path):
                            second_future = a_star(
                                second_agent.my_map, second_agent.path[t], second_agent.goal, second_agent.heuristics, second_agent.id, second_agent.constraints + constraint2, timestep=t)
                            if second_future != None:
                                new_path_second = second_agent.path[:t] + second_future
                            else:
                                new_path_second = second_agent.path
                                constraint2 = []

                        elif t >= len(second_agent.path):
                            second_future = a_star(
                                second_agent.my_map, second_agent.path[-1], second_agent.goal, second_agent.heuristics, second_agent.id, second_agent.constraints + constraint2, timestep=t)

                            if second_future != None:
                                new_path_second = second_agent.path[:t] + second_future
                            else:
                                new_path_second = second_agent.path
                                constraint2 = []

                        else:
                            new_path_second = second_agent.path
                            constraint2 = []
                            print("did not work as planned")

                        temp_radar = returnradar(self.id, [new_path, new_path_second], t)
                        temp_coll = detect_collision(temp_radar[0], temp_radar[1])

                        if temp_coll[0] == None and len(new_path+new_path_second) <= len(second_agent.path+new_path) and len(new_path+new_path_second) <= len(self.path+new_path_second) and not detect_stalemate(temp_radar[0], temp_radar[1], coll[-1]):
                            self.path = new_path
                            second_agent.path = new_path_second
                            self.constraints = self.constraints + constraint1
                            #print('both constraints')
                            second_agent.constraints = second_agent.constraints + constraint2
                        # and t<len(self.path):
                        elif len(new_path + second_agent.path) <= len(self.path + new_path_second)  and len(constraint1) > 0 and not detect_stalemate(temp_radar[0], radar_loc[j], coll[-1]):
                            self.path = new_path
                            self.constraints = self.constraints + constraint1
                            #print('first agents constraint')

                        # and t < len(second_agent.path):
                        elif len(constraint2) > 0  and not detect_stalemate(radar_loc[self.id], temp_radar[1], coll[-1]):
                            second_agent.path = new_path_second
                            second_agent.constraints = second_agent.constraints + constraint2

                            #print('else')
                            
                        elif len(constraint1) > 0:   #!!! this was 1 but changed it to 0     # and t < len(self.path)

                            self.path = new_path
                            self.constraints = self.constraints + constraint1
                            #print('first agents constraint')

                        else:
                            print('no solution')
                            break
                        new_radar = returnradar(self.id, [self.path, second_agent.path], t)
                        radar_loc[self.id] = new_radar[0]
                        radar_loc[j] = new_radar[1]
                        con1 = self.constraints
                        con2 = second_agent.constraints

                        coll = detect_collision(new_radar[0], new_radar[1])  

