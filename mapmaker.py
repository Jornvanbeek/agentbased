#layoutgenerator3000
import numpy as np
import random as rd
from single_agent_planner import compute_heuristics
from pathlib import Path


# n_agents = 30
# grid_y = 10
# grid_x = 22
# n_obstacles = 40




"""this file was created to speed up the process of developing maps. It is not part of the three planners!"""

def import_mapf_instance(filename):

    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def random_coordinates(n, obstacles, starts, goals, grid_x , grid_y ):
    lst = []
    i = 0


    while i < n:
        x = rd.randint(0, grid_x-1)
        y = rd.randint(0, grid_y-1)
        loc = [y,x]

        if loc not in lst and loc not in obstacles and loc not in starts and loc not in goals:
            lst.append([y,x])
            i = i + 1
    return lst


def randommapmaker(n_agents, grid_y, grid_x, n_obstacles):
    # n_agents = 30
    # grid_y = 10
    # grid_x = 22
    # n_obstacles = 40
    
    my_map = []
    for i in range(grid_y):
        my_map.append([])
        for j in range(grid_x):
            my_map[i].append(False)
    
    obstacles = []
    starts = []
    goals = []
    i = 0
    
    std_loc = True
    std_map = False
    std_map = 3
    
    if std_map != False:
        filename = 'instances/layout'+str(std_map)+'.txt'
        my_map, generic_starts, generic_goals = import_mapf_instance(filename)
    
    std_starts = []
    std_goals= []
    for i in range(1,len(my_map)-1):
        std_starts.append([i,1])
        std_goals.append([len(my_map)-i-1, 22])
        std_starts.append([i,22])
        std_goals.append([len(my_map)-i-1, 1])
    

    

    
    
    if std_map == False:
        obstacles = random_coordinates(n_obstacles, obstacles, starts, goals, grid_x , grid_y )
    else:
        obstacles = []
        for i in range(len(my_map)):
            for j in range(len(my_map[i])):
                if my_map[i][j]==True:
                    obstacles.append([i,j])
    
    if std_loc == False:
        starts = random_coordinates(n_agents, obstacles, starts, goals, grid_x , grid_y )
        goals = random_coordinates(n_agents, obstacles, starts, goals, grid_x , grid_y )
    else:
        starts = std_starts[0:n_agents]
        goals = std_goals[0:n_agents]
    
    
    if std_map == False:
        for obstacle in obstacles:
            my_map[obstacle[0]][obstacle[1]] = True
    
    
    for goal in goals:

        try:
            h_values = compute_heuristics(my_map, (goal[0],goal[1]))
        except:
            print("goal is wrong")
        for start in starts:
            try:
                h_values[start[0],start[1]]
            except:
                ind = starts.index(start)
                
                it = 0
                connect = False
                while connect != True and it <100:
                    new_start = random_coordinates(1, obstacles, starts, goals,grid_x , grid_y )[0]
                    new_goal = random_coordinates(1, obstacles, starts, goals,grid_x , grid_y )[0]
                    new_h_values = compute_heuristics(my_map, (new_goal[0],new_goal[1]))
                    
                    try:
                        h_values[new_start[0],new_start[1]]
                    except:
                        try:
                            new_h_values[start[0],start[1]]
                        except:
                            connect = False
                        else:
                            connect = True
                            goals[ind] = new_goal
                            
                        
                    else:
                        connect = True
                        
                        
                        
                if connect != True and it >98:
                     print('unable to connect starts and goals, advice is to reduce number of obstacles')
    
    for i in range(len(starts)):
        starts[i] = tuple(starts[i])
    for i in range(len(goals)):
        goals[i] = tuple(goals[i])
    return my_map,starts,goals 

def mapwriter():
    for n_agents in range(1,18):
        for i in range(3,4):
            #n_agents = 8
            grid_y = 11
            grid_x = 24
            n_obstacles = 50
            my_map,starts,goals = randommapmaker(n_agents, grid_y, grid_x, n_obstacles)
            #print(my_map)
            
            with open('standard_maps/new_random_' + str(n_agents)+ "_" + str(i) + '.txt', 'w') as f:
                f.write(str(grid_y) + ' '+ str(grid_x)+'\n')
                for i in range(len(my_map)):
                    for j in range(len(my_map[i])):
                        if my_map[i][j]== True:
                            f.write("@ ")
                        elif my_map[i][j]== False:
                            f.write(". ")
                    f.write("\n")
                f.write(str(n_agents)+ "\n")
                for k in range(n_agents):
                    f.write(str(starts[k][0]) + " " + str(starts[k][1]) + " " + str(goals[k][0]) + " " + str(goals[k][1]) + "\n")

#mapwriter()
