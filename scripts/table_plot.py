import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
import argparse
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)
def main(): 
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help="yaml file for the table")
    args = parser.parse_args()


    with open(args.input) as stats_file:
        statsdict = yaml.load(stats_file, Loader=yaml.FullLoader)
  

    files = list(statsdict.keys())
    startnumofRobots   = list(statsdict[files[0]].keys())[0]
    finalnumberofRobots   = list(statsdict[files[0]].keys())[-1]
    numofRobots = finalnumberofRobots - startnumofRobots + 2
    print(r"%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print(r"% Generated table, do not edit!")
    print(r"\begin{tabular}{l||c||c||c||c||c||c||c||c}")
    print("Environment & Metrics &"+ r" & ".join([str(cfnum)  + " robots" for cfnum in range(startnumofRobots, finalnumberofRobots+1)]) + r" & \\")

    for file in files:
        stats = statsdict[file]
        cfnums = list(stats.keys())
        filename = file.replace('.sh','').capitalize()
        print(r"\hline")

        print(" & Cost & "+ " & ".join(["{:.3f} {{\\color{{gray}} \\tiny {:.3f} }}".format(np.mean(statsdict[file][cfnum]['initcosts']),np.std(statsdict[file][cfnum]['initcosts']))
            if statsdict[file][cfnum].get('initcosts') else " - " for cfnum in cfnums])  + r" & \\")
        print(filename + " & Time [min] & "+ " & ".join(["{:.1f} {{\\color{{gray}}\\tiny {:.1f} }}".format(np.mean(statsdict[file][cfnum]['inittimes'])/60, np.std(statsdict[file][cfnum]['inittimes'])/60) 
                if statsdict[file][cfnum].get('initcosts') else " - " for cfnum in cfnums])  + r" & \\")
        print(" & Success Rate [\%] & "+ " & ".join(["{:.1f}".format((10*statsdict[file][cfnum]['successRuns'])) 
        if statsdict[file][cfnum].get('initcosts') else " - " for cfnum in cfnums]) + r" & \\")

    
    print(r"\end{tabular}")
    print(r"%%%%%%%%%%%%%%%%%%%%%%%%%%%")


    # ################### Plot ###########################
    # ## Maze for 3 robots
    # print('\n\n\n\n')
    # maze_stat = statsdict['maze.sh'][3]
    # maze_stat.pop('initcosts')
    # maze_stat.pop('inittimes')
    # maze_stat.pop('successRuns')

    # totalcost = []
    # totaltime = []
    # shortest_sol_size = 100
      
    # for run in maze_stat.keys():
    #     if maze_stat[run].get('stats') !=  None:
    #         stats = maze_stat[run]
    #         if shortest_sol_size > len(list(stats.values())[0]):
    #             shortest_sol_size = len(list(stats.values())[0])

    # totalcost = np.zeros((10, shortest_sol_size))
    # totaltime = np.zeros((10, shortest_sol_size))
    # for run in maze_stat.keys():
    #     costcounter = 0
    #     if maze_stat[run].get('stats') !=  None:
    #         stats = maze_stat[run]['stats']
    #         for stat in  stats:
    #             if costcounter < shortest_sol_size:
    #                 totalcost[run, costcounter] = stat['cost']
    #                 totaltime[run, costcounter] = stat['t']
    #             costcounter+=1

    # meancost = np.mean(totalcost, axis=0)
    # meancost = meancost[meancost != 0]
    
    # stdcost = np.std(totalcost, axis=0)
    # stdcost = stdcost[stdcost != 0]
    
    # meantime = np.mean(totaltime, axis=0)
    # meantime = meantime[meantime != 0]
    # print(meancost)
    # print(meantime)
    # plt.plot(meantime, meancost, color='blue', label='mean')
    # plt.fill_between(meantime, meancost-stdcost, meancost+stdcost, color='lightblue', alpha=0.5, label='std')
    # plt.legend()
    # plt.xlabel('Time')
    # plt.ylabel('Value')
    # plt.title('Mean and Std over Time')
    # plt.show()
    # # print(np.mean(totaltime, axis=1))

if __name__ == "__main__": 
    main()