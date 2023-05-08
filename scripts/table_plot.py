import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
import argparse
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)
from matplotlib.backends.backend_pdf import PdfPages


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
    print(r"\begin{tabular}{l||l||l||l||l||l||l||l||c}")
    print("Environment & Metrics &"+ r" & ".join([str(cfnum)  + " robots" for cfnum in range(startnumofRobots, finalnumberofRobots+1)]) + r" & \\")

    for file in files:
        stats = statsdict[file]
        cfnums = list(stats.keys())
        filename = file.replace('.sh','').capitalize()
        print(r"\hline")

        print(" & Cost & "+ " & ".join(["{:.2f} {{\\color{{gray}} \\tiny {:.2f} }}".format(np.mean(statsdict[file][cfnum]['initcosts']),np.std(statsdict[file][cfnum]['initcosts']))
            if statsdict[file][cfnum].get('initcosts') else "\\hspace{0.4cm}- " for cfnum in cfnums])  + r" & \\")
        print(filename + " & Time [sec] & "+ " & ".join(["{:.2f} {{\\color{{gray}}\\tiny {:.2f} }}".format(np.mean(statsdict[file][cfnum]['inittimes']), np.std(statsdict[file][cfnum]['inittimes'])) 
                if statsdict[file][cfnum].get('initcosts') else "\\hspace{0.4cm}- " for cfnum in cfnums])  + r" & \\")
        print(" & Success Rate [\%] & "+ " & ".join(["{:.2f}".format((10*statsdict[file][cfnum]['successRuns'])) 
        if statsdict[file][cfnum].get('initcosts') else "\\hspace{0.4cm}- " for cfnum in cfnums]) + r" & \\")

    
    print(r"\end{tabular}")
    print(r"%%%%%%%%%%%%%%%%%%%%%%%%%%%")


    # ################### Plot ###########################

    numRobots = [3, 5]
    envs = ['forest.sh', 'maze.sh']
    labels = [ 'Forest: 4 robots', 'Maze: 5 robots']
    meancolors = ['r', 'g']
    stdcolors = ['r','g']
    with PdfPages('result_logscale.pdf') as pdf:   
        fig, ax = plt.subplots()
        ax.grid('True', which='both', axis='x', linestyle='dashed')
        ax.grid(which='major', axis='y', linestyle='dashed')
        for env, robot, label, color, stdcolor in zip(envs, numRobots, labels, meancolors, stdcolors):
            allstat = statsdict[env][robot]
            if allstat.get('initcosts'):
                allstat.pop('initcosts')
                allstat.pop('inittimes')
                allstat.pop('successRuns')
            T = 750 
            dt = 0.1
            costs = []
            for run in allstat.keys():
                costsperrun = np.zeros(int(T / dt)) * np.nan
                if allstat[run].get('stats') !=  None:    # ## Maze for 3 robots
                    for d in allstat[run]['stats']:
                        idx =  int(d["t"] / dt)
                        costsperrun[idx:] = d["cost"]
                    costs.append(costsperrun)
            times = np.arange(0, T, dt)
            
            costs = np.array(costs)
            rs = costs.shape[0]
            cs = costs.shape[1]
            indices = []
            for c in range(cs):
                nanNums = 0
                for r in range(rs):
                    if np.isnan(costs[r, c]): 
                        nanNums += 1
                if nanNums > 5:
                    indices.append(c)

            costs = np.delete(costs, indices, axis=1)
            times = np.delete(times, indices, axis=0)
            
            mean = np.nanmean(costs, axis=0)
            std = np.nanstd(costs, axis=0)

            ax.plot(times, mean, label=label, color=color, lw=2.5)
            ax.set_xscale('log')
            ax.fill_between(times, mean+std, mean-std, color=stdcolor, alpha=0.1)
            ax.legend()
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Cost [s]")
        plt.show()
        pdf.savefig(fig)
        plt.close()

if __name__ == "__main__": 
    main()