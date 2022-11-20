

#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver  # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import time as timer
from mapmaker import randommapmaker
import matplotlib.pyplot as plt
import numpy as np
import gpboost as gpb

# instance = open('instances/exp0.txt','r').read()

# new
batch = False
batch = True

# "CBS",
solverlist = [ "Distributed"]
defaultinstance = 'instances/random.txt'
defaultinstance ='standard_maps/new_random_10_1.txt'

#defaultinstance = 'instances/random.txt'

if batch == True:
    defaultinstance = 'instances/test_*.txt'
    defaultinstance ='new_instances/new_random_*.txt'
    defaultinstance ='standard_maps/new_random_*_3.txt'
    solverlist = ["Prioritized", "Distributed", "CBS"]


n_agents = 16
grid_y = 10
grid_x = 20
n_obstacles = 30


mincost = open("instances/min-sum-of-cost.csv", "r").read()
mincost = mincost.split(",")

starttime = timer.time()
batch_cost = 0


starttime = timer.time()
# batch_cost = 0

prio_coll = ' '


def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and 
    a number for the start location of each agent.

    Example:
        @ @ @ @ @ @ @ 
        @ 0 1 . . . @ 
        @ @ @ . @ @ @ 
        @ @ @ @ @ @ @ 
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename, n_agents=n_agents, grid_y=grid_y, grid_x=grid_x, n_obstacles=n_obstacles):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)

    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    if filename == 'instances/random.txt':
        return randommapmaker(n_agents, grid_y, grid_x, n_obstacles)

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


allcosts = [ [], [], [] ]
alltimes = [ [], [], [] ]
for k in range(len(solverlist)):
    SOLVER = solverlist[k]
    filetime = []
    solvertime = timer.time()
    if __name__ == '__main__':
        parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
        parser.add_argument('--instance', type=str, default=defaultinstance,
                            help='The name of the instance file(s)')
        parser.add_argument('--batch', action='store_true', default=False,
                            help='Use batch output instead of animation')
        parser.add_argument('--disjoint', action='store_true', default=False,
                            help='Use the disjoint splitting')
        parser.add_argument('--solver', type=str, default=SOLVER,
                            help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

        args = parser.parse_args()
        # Hint: Command line options can be added in Spyder by pressing CTRL + F6 > Command line options.
        # In PyCharm, they can be added as parameters in the configuration.
        name = "results"+ SOLVER + ".csv"
        result_file = open(name, "w", buffering=1)
        batch_cost = 0
        
        for file in sorted(glob.glob(args.instance)):
            filetime = timer.time()
            print("***Import an instance***")
            print("filename: ----   ", file)
            my_map, starts, goals = import_mapf_instance(file)
            #print_mapf_instance(my_map, starts, goals)

            if args.solver == "CBS":
                print("***Run CBS*** file: ", file)
                cbs = CBSSolver(my_map, starts, goals)
                paths = cbs.find_solution(args.disjoint)
            elif args.solver == "Independent":
                print("***Run Independent*** file: ", file)
                solver = IndependentSolver(my_map, starts, goals)
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print("***Run Prioritized*** file: ", file)
                solver = PrioritizedPlanningSolver(my_map, starts, goals)
                paths = solver.find_solution()

            elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
                print("***Run Distributed Planning*** file: ", file)
                # !!!TODO: add your own distributed planning implementation here.
                solver = DistributedPlanningSolver(my_map, starts, goals)  # ,....
                paths = solver.find_solution()
                # method to write amount of collisions to results.csv:
                prio_coll = paths[-1]
                paths = paths[0]
            else:
                raise RuntimeError("Unknown solver!")

            # generate two methods of writing to results.csv, if the file is a test_* file, a comparison with the optimal can be made
            cost = get_sum_of_cost(paths)
            allcosts[k].append(sum(cost))
            alltimes[k].append(timer.time()-filetime)
            if file[10] == 't':
                mincost_instance = int(mincost[mincost.index('\n'+file.replace("\\", "/"))+1])
                result_file.write("{},{},{},{}\n".format(file, cost, prio_coll, sum(cost)-mincost_instance))
            else:
                result_file.write("{},{},{}\n".format(file, cost, sum(cost)))
                print("working on:---- ", file)
            batch_cost += sum(cost)

            if not batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                animation.show(solver=SOLVER, filename=file)
        
        result_file.write(str(batch_cost)+"\n")
        result_file.write(str(timer.time()-starttime))
        result_file.close()

        
    
    # print the cpu clock
    print("cpu time overall:", timer.time()-starttime)

    # print difference between optimal and current batch cost
    if batch:
        print("BATCH total difference between solutions and optimal: ", batch_cost-1850)

    # print difference between optimal and current individual test cost
    elif len(defaultinstance) > 10:
        if defaultinstance[10] == 't':
            mincost_instance = int(mincost[mincost.index('\n'+defaultinstance)+1])
            print("INDIVIDUAL difference between current solution and optimal: ",  sum(cost)-mincost_instance)


# plt.plot([np.average(allcosts[0]), np.average(allcosts[1]), np.average(allcosts[2])])
# plt.show()
# plt.figure()
# plt.plot([np.average(alltimes[0]), np.average(alltimes[1]), np.average(alltimes[2])])
# plt.show()
# plt.figure()
# plt.plot([np.sum(alltimes[0]), np.sum(alltimes[1]), np.sum(alltimes[2])])


if batch == True:
    if len(solverlist)>2:
        fig, axs = plt.subplots(2,2)
        fig.suptitle('Comparison between different planning methods on a 8x8 map, n= 200, 8 agents, 6 obstacles')
        
        # basic plot
        axs[0,0].boxplot(allcosts, 0, '')
        
        axs[0, 1].boxplot(allcosts)
        
        
        # don't show outlier points
        axs[1,0].boxplot(alltimes, 0, '')
        
        
        # don't show outlier points
        axs[1,1].boxplot(alltimes)
        
        
        axs[0,0].set(
            axisbelow=True,  # Hide the grid behind plot objects
        
            xlabel='Distribution per planning method',
            ylabel='Total costs per file without outliers',
        )
        axs[0,0].yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                       alpha=0.5)
        
        
        axs[0,1].set(
            axisbelow=True,  # Hide the grid behind plot objects
        
            xlabel='Distribution per planning method',
            ylabel= 'Total costs per file with outliers',
        )
        axs[0,1].yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                       alpha=0.5)
        
        
        axs[1,0].set(
            axisbelow=True,  # Hide the grid behind plot objects
        
            xlabel='Distribution per planning method',
            ylabel='T compute without outliers (s)',
        )
        axs[1,0].yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                       alpha=0.5)
        
        
        axs[1,1].set(
            axisbelow=True,  # Hide the grid behind plot objects
        
            xlabel='Distribution per planning method',
            ylabel='T compute with outliers',
        )
        axs[1,1].yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                       alpha=0.5)
        
        
        for i in range(2):
            axs[i,0].set_xticklabels(solverlist,
                                  rotation=45)
            axs[i,1].set_xticklabels(solverlist,
                                  rotation=45)
        
    # print([np.average(allcosts[0]), np.average(allcosts[1]), np.average(allcosts[2])])
    # print([np.sum(alltimes[0]), np.sum(alltimes[1]), np.sum(alltimes[2])])
    # if len(solverlist)>2:
    #     print([np.average(alltimes[0]), np.average(alltimes[1]), np.average(alltimes[2])])
    
    plt.figure()
    
    file_compensator = len(allcosts[0]) - 9
    
    allcosts[0] = allcosts[0][file_compensator:]+allcosts[0][:file_compensator]
    allcosts[1] = allcosts[1][file_compensator:]+allcosts[1][:file_compensator]
    if len(solverlist)>2:
        allcosts[2] = allcosts[2][file_compensator:]+allcosts[2][:file_compensator]
    alltimes[0] = alltimes[0][file_compensator:]+alltimes[0][:file_compensator]
    alltimes[1] = alltimes[1][file_compensator:]+alltimes[1][:file_compensator]
    if len(solverlist)>2:
        alltimes[2] = alltimes[2][file_compensator:]+alltimes[2][:file_compensator]
    
    
    
    
    plt.plot(np.arange(1,len(allcosts[0])+1),allcosts[0])
    plt.plot(np.arange(1,len(allcosts[0])+1),allcosts[1])
    if len(solverlist)>2:
        plt.plot(np.arange(1,len(allcosts[0])+1),allcosts[2])
    plt.title('Cost for increasing number of agents, layout 3')
    plt.ylabel("Total cost")
    plt.xlabel("Amount of agents")
    plt.legend(solverlist)
    plt.show()
    
    if len(solverlist)>2:
        plt.figure()
        plt.plot(np.arange(1,len(allcosts[0])+1),np.array(allcosts[0])-np.array(allcosts[2]))
        plt.plot(np.arange(1,len(allcosts[0])+1),np.array(allcosts[1])-np.array(allcosts[2]))
        plt.title('Additional cost for Prioritized and Distributed layout 3')
        plt.ylabel("Additional cost")
        plt.xlabel("Amount of agents")
        plt.legend(solverlist[:2])
    
        plt.show()
    
    
    

    
    # plt.figure()
    # plt.plot([np.average(alltimes[0]), np.average(alltimes[1]), np.average(alltimes[2])])
    # plt.title('Cost for increasing number of agents, layout 1')
    # plt.ylabel("Average ")
    # plt.xlabel("Solver")
    # plt.legend(solverlist)
    # plt.show()
    
    
    plt.figure()
    plt.plot([np.sum(alltimes[0]), np.sum(alltimes[1]), np.sum(alltimes[2])])
    plt.show()

    plt.figure()
    plt.plot(np.arange(1,len(alltimes[0])+1),alltimes[0])
    plt.plot(np.arange(1,len(alltimes[0])+1),alltimes[1])
    if len(solverlist)>2:
        plt.plot(np.arange(1,len(alltimes[0])+1),alltimes[2])
    plt.title('Computation time for increasing number of agents, layout 3')
    plt.ylabel("Computation time [s]")
    plt.xlabel("Amount of agents")
    plt.legend(solverlist)
    plt.show()
    
    plt.figure()
    plt.plot(np.arange(1,len(alltimes[0])+1),alltimes[0])
    plt.plot(np.arange(1,len(alltimes[0])+1),alltimes[1])
    if len(solverlist)>2:
        plt.plot(np.arange(1,len(alltimes[0])),alltimes[2][:-1])
    plt.title('Computation time for increasing number of agents, layout 3')
    plt.ylabel("Computation time [s]")
    plt.xlabel("Amount of agents")
    plt.legend(solverlist)
    plt.show()
    
    # layout 1
    # [183.08333333333334, 175.5, 170.08333333333334]
    # [0.3116722106933594, 6.539522409439087, 1747.9386405944824]
    # [0.025972684224446613, 0.5449602007865906, 145.66155338287354]
    
    """
    layout 2
[144.0, 141.66666666666666, 138.55555555555554]
[0.08776497840881348, 1.4700851440429688, 147.9874472618103]
[0.009751664267645942, 0.1633427937825521, 16.4430496957567]
    
    """
