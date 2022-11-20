import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################

            # -------------------- begin of changed code-----------------------------------------------------------------------

            for k in range(1, len(path)):            # loop over path to add path to constraints
                for l in range(self.num_of_agents):  # loop over all the agents to add constraints
                    if l != i:  # dont add the path of the current agent to its own constraints
                        constraints.append({'agent': l, 'loc': [path[k]], 'timestep': k})  # add vertices
                        if k > 1:
                            constraints.append({'agent': l, 'loc': [path[k], path[k-1]], 'timestep': k})  # add edges

            # add the goal to the constraints[0], for using when the goal is reached to block the goal
            for l in range(self.num_of_agents):
                if l != i:
                    constraints.append({'agent': l, 'loc': [path[-1]], 'timestep': 0, 'goaltime': len(path)-1})

            # -------------------- end of changed code-----------------------------------------------------------------------
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)[0]))
        print("Waiting time:    {}".format(get_sum_of_cost(result)[1]))
        # print(result)
        return result
