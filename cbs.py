import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):

    # normal constraints
    for t in range(min(len(path1), len(path2))):

        if get_location(path1, t) == get_location(path2, t):
            return [get_location(path2, t)], t

        elif t > 0:
            if get_location(path1, t-1) == get_location(path2, t) and get_location(path1, t) == get_location(path2, t-1):
                return [get_location(path1, t-1), get_location(path2, t-1)], t

        else:
            continue

    # goal constraints:
    goal_timestep = min(len(path1), len(path2))-1

    for k in range(goal_timestep, max(len(path1), len(path2))):

        if len(path1) >= len(path2):
            if get_location(path1, k) == get_location(path2, goal_timestep):
                return [get_location(path1, k)], k

        elif len(path1) < len(path2):
            if get_location(path1, goal_timestep) == get_location(path2, k):

                return [get_location(path2, k)], k

    return None, None


def detect_collisions(paths):

    collisions = []
    for i in range(len(paths)):
        for j in range(i, len(paths)):

            if i != j:
                collision = {}
                coll, t = detect_collision(paths[i], paths[j])
                collision['agent1'] = i
                collision['agent2'] = j

                if coll != None:
                    collision['loc'], collision['timestep'] = coll, t
                    collisions.append(collision)

    return collisions


def standard_splitting(collision):
    if len(collision) == 4:
        loc = collision['loc']
        loc2 = []

        if len(loc) == 1:
            loc2 = loc

        if len(loc) == 2:
            loc2.append(loc[1])
            loc2.append(loc[0])

        return [{'agent': collision['agent1'], 'loc': loc, 'timestep': collision['timestep']},
                {'agent': collision['agent2'], 'loc': loc2, 'timestep': collision['timestep']}]


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
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

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

        root['cost'] = sum(get_sum_of_cost(root['paths']))
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        new_node = {}
        time = timer.time()
        while len(self.open_list) > 0:                                          # open list should have entries
            for i in range(400000):                                             # keep iterations within bounds

                curr_node = self.pop_node()                                     # pop node with lowest cost (and later least collisions)
                if i % 1000 == 0 and i != 0:
                    print(i, ' timestep: ', timer.time()-time)
                    print('cost: ', curr_node['cost'], ' amount of collisions left: ', len(curr_node['collisions']))
                    time = timer.time()

                if len(curr_node['collisions']) == 0:             # collision free? there's your answer
                    print("no more collisions! ", i, " iterations")
                    print('cost: ', curr_node['cost'])
                    return curr_node['paths']
                else:
                    # detect_collisions(curr_node['paths'])[0])      # get two new constraints
                    constraints = standard_splitting(curr_node['collisions'][0])
                    for constraint in constraints:                              # loop over the two constraints and create new nodes

                        new_node = {}
                        new_node['constraints'] = curr_node['constraints'].copy()
                        agent = constraint['agent']
                        new_node['paths'] = curr_node['paths'].copy()           # copy parent

                        if len(curr_node['paths'][agent]) == 1 and self.goals[agent] == constraint['loc'][0] and constraint['agent'] == agent:
                            continue  # if an agent starts at its goal, and the new constraint is for its goal: dont add the constraint
                        new_node['constraints'].append(constraint)          # add the new constraint

                        path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                                      self.heuristics[agent], agent, new_node['constraints'])  # retreive new paths for agents with the new constraint

                        if path is None:
                            raise BaseException('No solutions')  # no solutions found

                        new_node['paths'][agent] = path  # replace old path by new
                        new_node['collisions'] = detect_collisions(new_node['paths'])  # what are the collisions?
                        new_node['cost'] = sum(get_sum_of_cost(new_node['paths']))  # costs?
                        self.push_node(new_node)  # add the new node to the open list

            return curr_node['paths']  # return if too much iterations have taken place: for debugging

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
