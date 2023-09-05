import numpy as np
import yaml
import argparse
import rowan as rn
import math as mt
FREQUENCY = 100 #Hz
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def unitvecs(cable, num_of_cables): 
    qcs = []
    for cableNum in range(num_of_cables):
        qcs.extend(polartovector(cable[2*cableNum: 2*cableNum+2]))
    return qcs

def polartovector(cablestate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = [-mt.cos(az)*mt.cos(el),
                -mt.sin(az)*mt.cos(el),
                -mt.sin(el)]
    return unitvec

planner_envs = [
    "empty_2robots.yaml",
    "empty_3robots.yaml",
    "empty_4robots.yaml",
    "empty_5robots.yaml",
    "empty_6robots.yaml",
    "forest_2robots.yaml",
    "forest_3robots.yaml",
    "forest_4robots.yaml",
    "forest_5robots.yaml",
    "forest_6robots.yaml",
    "maze_2robots.yaml",
    "maze_3robots.yaml",
    "maze_4robots.yaml",
    "maze_5robots.yaml",
    "maze_6robots.yaml",
]

planner_path = "../examples/benchmark/"
opt_env_path = "../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/"

for env in planner_envs:
    with open(planner_path + env, "r") as pl_file: 
        plan_env = yaml.safe_load(pl_file)
    obstacles = plan_env["environment"]["obstacles"]
    start = plan_env["payload"]["start"]
    goal = plan_env["payload"]["goal"]
    num_robots = plan_env["numofcables"]
    pType = plan_env["payload"]["shape"]["type"]
   
    with open(opt_env_path + env, "r") as opt_file:
        opt_env = yaml.safe_load(opt_file)
    if pType == "sphere":
        start_st = np.zeros(6 + 6*num_robots + 7 * num_robots)
        goal_st  = np.zeros(6 + 6*num_robots + 7 * num_robots)
    
        start_st[0:3] = start[0:3]
        goal_st[0:3] = goal[0:3]

        cable_St_start = start[7:7+6*num_robots]
        cable_St_goal  =  goal[7:7+6*num_robots]
        qcs_start = unitvecs(cable_St_start, num_robots)
        qcs_goal  = unitvecs(cable_St_goal , num_robots)
        wcs_start = np.zeros_like(qcs_start)
        wcs_goal  = np.zeros_like(qcs_goal)

        for i in range(num_robots):
            start_st[6+6*i: 6+6*i+3]  = qcs_start[3*i:3*i+3]          
            start_st[6+6*i+3: 6+6*i+6] = wcs_start[3*i:3*i+3]
            start_st[6+6*num_robots+7*i +3] = 1
            goal_st[6+6*i: 6+6*i+3]  = qcs_goal[3*i:3*i+3]          
            goal_st[6+6*i+3: 6+6*i+6] = wcs_goal[3*i:3*i+3]
            goal_st[6+6*num_robots+7*i+ 3] = 1

        opt_env["robots"][0]["start"] = start_st.tolist()
        opt_env["robots"][0]["goal"] = goal_st.tolist()
        opt_env["environment"]["obstacles"] = obstacles
        with open(opt_env_path + env, "w") as opt_file:
            yaml.safe_dump(opt_env, opt_file, default_flow_style=None)    
    elif pType == "triangle" or pType == "rod":
        start_st = np.zeros(13 + 6*num_robots + 7*num_robots)
        goal_st  = np.zeros(13 + 6*num_robots + 7*num_robots)   
    else: 
        print("NOT IMPLEMENTED")
        exit()