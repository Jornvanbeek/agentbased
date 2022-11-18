"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location
from aircraft import AircraftDistributed, returnradar, radar, timeradar
from cbs import detect_collision, detect_collisions


class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 

        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants
        start_time = timer.time()
        result = []
        agent_objects = []
        self.CPU_time = timer.time() - start_time

        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, [], [])
            path = newAgent.find_individual_solution([])
            #newAgent.path = path
            result.append(path)
            agent_objects.append(newAgent)
        list_len = [len(i) for i in result]
        t_max = max(list_len)
        
        #radar_loc = returnradar(agent, result, t, timeradar)
        t = 0
        
        
        while t < t_max:
            it = 0
            remaining_coll = detect_collisions(result)
            while it <50 and len(remaining_coll)>0:
                for agent in range(self.num_of_agents):
                    
    
                    radar_loc = returnradar(agent, result, t, timeradar)
    
                    agent_objects[agent].radar(radar_loc, radar, agent_objects,t)
                    result[agent] = agent_objects[agent].path
                
                list_len = [len(i) for i in result]
                t_max = max(list_len)               #update t max as total time could be increased!Alpha_[0]'
                remaining_coll = detect_collisions(result)
                it += 1
            # if len(detect_collisions(result)) > 0:
            #     t = t - 1
            else:    
                t += 1
            
            
            # if len(remaining_coll)>0:     # stay at the current timestep if there still are collisions that can be detected and run the iterations all over again
            #     print(remaining_coll)  
            #     for coll in remaining_coll:
            #         if coll['timestep']  <1:
            #             t = t - 1
            #             break
            #         if len(remaining_coll) == 1 and coll['timestep'] > 0:
            #             t += 1
                        
            # else:
            #     t +=1
            
            # if t < 0:
            #     t = 0

            

        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        # Hint: think about how cost is defined in your implementation
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        #print(result)

        # Hint: this should be the final result of the distributed planning (visualization is done after planning)
        return result, len(detect_collisions(result))
