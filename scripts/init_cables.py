import numpy as np
import yaml
import argparse
import rowan as rn
import math as mt


def unitvecs(cables): 
    qcs = []
    num_of_cables = int(len(cables)/2)
    for cable in cables:
        tmp = []
        for cableNum in range(num_of_cables):
            tmp.append(polartovector(cable[2*cableNum: 2*cableNum+2]))
        qcs.append(tmp) 
    return qc

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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", type=str, help="init_guess for 0 robots loaded")
    parser.add_argument("--out", default=None,  type=str, help="output file with cable")
    parser.add_argument("--envName", default=None,  type=str, help="env to copy the init cables")
    args = parser.parse_args()
    # laod init guess
    with open(args.inp, "r") as f:
        plan = yaml.safe_load(f) 
    states = plan["result"]["states"]

    with open(args.envName, "r") as f:
        env = yaml.load(f, Loader=yaml.FullLoader) 
    cableSt = env["payload"]["start"][7::]
    num_robots = int(len(cableSt)/2)
    cableandUAVNewStates = []
    
    quatwStates_i = list([0,0,0,1,0,0,0])
    quatwStates = []
    for i in range(num_robots):
        quatwStates.extend(quatwStates_i)
    for i in range(num_robots):
        cbSt = cableSt[2*i:2*i+2]
        cableandUAVNewStates.extend([*polartovector(cbSt), *[0,0,0]])
        # cableandUAVNewStates.extend([0,0,0])
    cableandUAVNewStates.extend(list(quatwStates))
    state_tmp = []
    
    actions = []
    action_i = []
    act = [1.,1.,1.,1.]
    for i in range(num_robots):
        action_i.extend(list(act))
    for state in states: 
        # action_i = list(act)  # Create a new list as a copy of 'act' for each robot
        actions.append(list(action_i))
        state.extend(cableandUAVNewStates)
        # state = np.array(state).flatten().tolist()
        state_tmp.append(state)

    plan["result"]["states"] = state_tmp
    plan["result"]["actions"] = []
    plan["result"]["actions"] = actions
    with open(args.inp, "w") as f:
        yaml.safe_dump(plan, f, default_flow_style=None)

    
if __name__ =="__main__":
    main()