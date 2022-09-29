import heapq


def move(loc, dir):
    directions = [(0, 0),(0, -1), (1, 0), (0, 1), (-1, 0)]  # added 0,0 for 0 velocity, but increase timestep
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    wait = 0
    for path in paths:

        for i in range(1, len(path)):
            if path[i] == path[i-1]:
                wait += 1
                continue
            else:
                rst += 1
        # rst += len(path) - 1
    return [rst, wait]


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.



    constraint_table = []
    j = 1
    for i in range(len(constraints)):
        if constraints[i]["agent"] == agent: #filter out constraints not belonging to this agent
            j = constraints[i]["timestep"]   #save the timestep of the current constraint as integer
            while len(constraint_table) < j+1: 
                constraint_table.append([]) #make sure constraint table is as long as the largest timestep to index by timestep

            constraint_table[j].append(constraints[i])  #add constraints to the constraints table, index is timestep

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################     length of the constraint table must be larger than next time (len is max(i) + 1)
    if constraint_table != None and len(constraint_table)>0:   #check if there are entries in the constraint table for this timestep
          #retreive constraints for the next timestep
        
        
        
        goal_constraints = constraint_table[0]
        if len(goal_constraints)>0:
            for j in range(len(goal_constraints)):
                goalcon = goal_constraints[j]
                
                
                if len(goalcon) == 4:
                    if next_time >= goalcon['goaltime']:
                        if goalcon['loc'][0] == next_loc:
                            return True
                        else:
                            continue
                        # goalcon['timestep'] = next_time
                        # constraints.append(goalcon)
        
        if len(constraint_table)>next_time:
            constraints = constraint_table[next_time]
        else:
            constraints = []
        
        
        if len(constraints) > 0:              #any constraint for this timestep?
            for i in range(len(constraints)):   #loop over constraints
                con = constraints[i]
                
                    
                if len(con['loc']) == 1:            #if vertex constraint
                    if con['loc'][0] == next_loc:   # if location is constrained
                        return True
                    else:
                        continue
                if len(con['loc']) == 2:          #if edge constraint
                    if con['loc'][0] == curr_loc and con['loc'][1] == next_loc: #if move from curr_loc to next_loc forbidden
                        return True
                    else:
                        continue
                else:
                    raise RuntimeError("length of given constraint is not 1 or 2! constraint: ", con)
            else:
                return False            
        else:
            return False
        
        
        
    
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.




def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    #print("constraints: ", constraints, ", agent: ", agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'timestep': 0, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        
        
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        timestep = curr['timestep']
        if curr['loc'] == goal_loc:
            if timestep > len(constraint_table): #no more constraints after this time so easy shortcut
                return get_path(curr)
                
            else:
                con = 0
                for i in range(timestep,len(constraint_table)):
                    if is_constrained(goal_loc, goal_loc, i, constraint_table):
                        con += 1
                if con ==0:  
                    return get_path(curr)
                else:
                    continue
        
            

            
        
        # move and check constraints
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            
  
            
            #is there a constraint?
            if is_constrained(curr['loc'], child_loc, curr['timestep']+1, constraint_table):  
                continue
            
            #check if the next location is on a available vertex: not the edge or blocked vertex
            #first check is for the case that the edges are not given as @@@@@@@@ (such as in the test_4 file)
            if child_loc[0] >= 0 and child_loc[1]>=0 and child_loc[0] < len(my_map):
                if child_loc[1] < len(my_map[child_loc[0]]):
                    
                    #check if vertex is not blocked
                    if my_map[child_loc[0]][child_loc[1]]:
                        continue
                else:
                    continue
            else:
                continue
            
            #make child
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'timestep': curr['timestep'] + 1,
                     'parent': curr}
            
            # if location visited before:
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                # if visited before and path plus heuristic path shorter, then old node replaced by child
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
                    
            # not visited before just add the child
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
