import numpy as np
import subprocess
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
import argparse

def main():

    
    with open("compute_stats.yaml") as compute_stats_file:
        compute_stats = yaml.load(compute_stats_file, Loader=yaml.FullLoader)
    files = [compute_stats[filename]['path'] for filename in compute_stats.keys() if compute_stats[filename]['enable'] == True]
    yamlDirs = [compute_stats[filename]['yamldir'] for filename in compute_stats.keys() if compute_stats[filename]['enable'] == True]
    runs = [compute_stats[filename]['runs'] for filename in compute_stats.keys() if compute_stats[filename]['enable'] == True]
    
    statsdict = dict()

    ## This part is responsible to set some values in the config files of each number of robots based on compute_stats.yaml
    ## For forest problem: it  is responsible to generate the forest environment with random obstacles
            # it makes sure that the obstacle is not centered at start or goal
            # it also makes sure that there is a tolerance between the obs and start and goal
    ## For maze and empty it sets the timelimits
    problems = [problem for problem in compute_stats.keys() if compute_stats[problem]['enable'] == True]
    for problem in problems:
        robots = compute_stats[problem]["rangeofRobots"]
        yamlnums = list(range(robots[0],robots[-1]+1))
        timelimits = compute_stats[problem]["timelimits"]
        environment_min = compute_stats[problem]["environment"]["min"]
        environment_max = compute_stats[problem]["environment"]["max"]
        if problem == "forest":
            numOfObs = compute_stats[problem]["numofObs"]
            for yamlnum, timelimit in zip(yamlnums, timelimits):
                with open("../examples/forest/"+str(yamlnum)+"cfs.yaml") as forestyaml:
                    forestcfg = yaml.load(forestyaml, Loader=yaml.FullLoader)
                obstacle = []
                for Obs in range(numOfObs):
                    min_env = compute_stats["forest"]["min_env"]
                    max_env = compute_stats["forest"]["max_env"]
                    tol     = compute_stats["forest"]["tol"]
                    center = [np.random.uniform(low=min_env, high=max_env), np.random.uniform(low=min_env, high=max_env), np.random.uniform(low=-0.01, high=0.01)]
                    feasible = False
                    while not feasible:
                        start_tol = np.linalg.norm(np.asarray(center) - np.asarray(forestcfg["payload"]["start"][0:3]))
                        goal_tol  = np.linalg.norm(np.asarray(center) - np.asarray(forestcfg["payload"]["goal"][0:3]))
                        if start_tol <= tol or goal_tol <= tol:
                            center = [np.random.uniform(low=min_env, high=max_env), np.random.uniform(low=min_env, high=max_env), np.random.uniform(low=-0.01, high=0.01)]
                        else:
                            feasible = True
                    obstacle.append({'type' : "cylinder",  
                                    'center' : center, 
                                    'radius' : 0.1,
                                    'height' : 3.0})
                forestcfg['environment']['obstacles'] = obstacle
                forestcfg['timelimit'] = timelimit
                forestcfg['environment']['min'] = environment_min
                forestcfg['environment']['max'] = environment_max
                forestfile=open("../examples/forest/"+str(yamlnum)+"cfs.yaml","w")
                yaml.dump(forestcfg,forestfile, default_flow_style=True)
                forestfile.close()
        
        else:
            for yamlnum, timelimit in zip(yamlnums, timelimits):
                with open("../examples/"+problem+"/"+str(yamlnum)+"cfs.yaml") as problemyaml:
                    problemcfg = yaml.load(problemyaml, Loader=yaml.FullLoader)
                
                problemcfg['timelimit'] = timelimit
                problemcfg['environment']['min'] = environment_min
                problemcfg['environment']['max'] = environment_max
                problemfile=open("../examples/"+problem+"/"+str(yamlnum)+"cfs.yaml","w")
                yaml.dump(problemcfg,problemfile, default_flow_style=True)
                problemfile.close()

    problem_num = 0
    for file, yamlDir in zip(files, yamlDirs):
        statsdict.update({file: {}})

        for yamlnum in yamlnums:
            statsdict[file].update({yamlnum : {}})
            avcost = 0
            avtime = 0
            successRuns = 0
            initcosts = []
            inittimes = []
            for run in range(runs[problem_num]):    
                print(run) 
                statsdict[file][yamlnum].update({run: {}})
                print('Solving: '  + str(yamlnum) + " " + file)
                subprocess.run(['../tests/./' + file + " " +str(yamlnum) + " " + str(yamlnum) + " " + str(run)], shell=True)
                print('Completed '  + str(yamlnum) + " " + file)
                print('add stats in dict')
                with open("../examples/"+yamlDir+"/ompl_stats"+str(yamlnum)+'.yaml') as stats_file:
                    stats_ = yaml.load(stats_file, Loader=yaml.FullLoader)
                if stats_['stats'] != None:
                    statsdict[file][yamlnum][run].update({'stats' : stats_['stats']})
                    successRuns+=1
                    initcosts.append(statsdict[file][yamlnum][run]['stats'][0]['cost'])
                    inittimes.append(statsdict[file][yamlnum][run]['stats'][0]['t'])
            if successRuns > 0:
                statsdict[file][yamlnum].update({'initcosts' : initcosts})
                statsdict[file][yamlnum].update({'inittimes' : inittimes})
                statsdict[file][yamlnum].update({'successRuns' : successRuns})
        problem_num+=1

        statdictFile=open("statdict.yaml","w")
        yaml.dump(statsdict,statdictFile)
        statdictFile.close()
        print("YAML file saved.")

if __name__ == "__main__": 
    main()