import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    for t in range(min(len(path1), len(path2))):
        
        if get_location(path1,t) == get_location(path2,t):
            print('vertex collision at t = ', t)
            return [get_location(path1,t)],t
        
        elif t > 0:
            if get_location(path1,t-1) == get_location(path2, t) and get_location(path1,t) == get_location(path2,t-1):
                print('edge collision at t = ', t)
                return [get_location(path1,t-1),get_location(path1,t)], t
        
        else:
            continue
    return None, None           #should this return none or something else??


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)):
        for j in range(len(paths)):
            if i !=j:
                collision = {}
                coll, t = detect_collision(paths[i],paths[j])
                collision['agent1'] = i
                collision['agent2'] = j

                if coll != None:
                    collision['loc'],collision['timestep'] = coll,t
                
                    collisions.append(collision)
                

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if len(collision) == 4:
        loc = collision['loc']
        
        loc2 = []
        if len(loc) == 2:
            loc2.append(loc[1])
            loc2.append(loc[0])

        
        if loc2 == None:
            print('loc2 = None')
        return [{'agent': collision['agent1'], 'loc': loc, 'timestep': collision['timestep']}, 
                {'agent': collision['agent2'], 'loc': loc2, 'timestep': collision['timestep']}]



def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print("root collisions: ", root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print("standard splitting collision: ", standard_splitting(collision))
        
        
        
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        new_node = {}
        
        while len(self.open_list)>0:
            for i in range(500):
                curr_node = self.pop_node()
                
                if len(new_node) !=0:
                    if sum(curr_node['cost'])> sum(new_node['cost']):
                        print('---------------non minimum cost!!')
                        
                if len(curr_node['collisions']) == 0:
                    return curr_node['paths'] #moet dit een return zijn?
                else:
                    for j in range(len(curr_node['collisions'])):
                        constraints = standard_splitting(curr_node['collisions'][j])
                        for constraint in constraints:
                            new_node = {}
                            new_node['constraints'] = curr_node['constraints']
                            if len(constraint['loc']) > 0:
                                new_node['constraints'].append(constraint)
                            new_node['paths'] = curr_node['paths']
                            agent = constraint['agent']
                            #for agent in range(len(curr_node['paths'])):
                            path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, new_node['constraints'])
                        
                            if path is None:
                                raise BaseException('No solutions')
                                
                            if len(path)>0:
                                new_node['paths'][agent] = path
                                new_node['collisions'] = detect_collisions(new_node['paths'])
                                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                                
                                self.push_node(new_node)
                        
                    
            return curr_node['paths']
                    
        # self.print_results(root)
        return "No solution"

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
