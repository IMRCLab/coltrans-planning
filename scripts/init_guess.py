import numpy as np
import yaml
import argparse
import rowan as rn
import math as mt
FREQUENCY = 100 #Hz

def derivative(vec, dt):
    dvec  =[[0,0,0]]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp.tolist())
    return np.asarray(dvec)

def cable2uavPos(payloadPos, qs, l, num_of_cables):
    #pos: positions of payload
    #qs: directional vectors of cable
    #l: length of cable
    uavPos = []
    for cableNum in range(num_of_cables):
        for i in range(len(qs)):
            q = qs[i+cableNum]
            po = payloadPos[i]
            uavPos.append((np.asarray(po) + l*np.asarray(q)).tolist())

    return uavPos

def cable2uavVel(payloadVel, qds, l, num_of_cables):
    # vos: velocity vectors of payload 
    # qds: derivative of directional vectors
    # length of cable 
    uavVel = []
    for cableNum in range(num_of_cables):
        for i in range(len(qds)):
            qd = qds[i+cableNum]
            vo = payloadVel[i]
            uavVel.append((np.asarray(vo) + l*np.asarray(qd)).tolist())

    return uavVel

def cable2uavAcc(payloadAcc, qdds, l, num_of_cables):
    # vos: velocity vectors of payload 
    # qds: derivative of directional vectors
    # length of cable 
    uavAcc = []
    for cableNum in range(num_of_cables):
        for i in range(len(qdds)):
            qdd = qdds[i+cableNum]
            ao = payloadAcc[i]
            uavAcc.append((np.asarray(ao) + l*np.asarray(qdd)).tolist())

    return uavAcc




## from : https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L904
def qinv(q):
    return rn.conjugate(q)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L896
def qqmul(q, p):
    return rn.multiply(q, p)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L989
def quat2omega(quat,dt):
    omega = [[0,0,0]]
    for i in range(len(quat)-1):
        q0 = quat[i]
        q1 = quat[i+1]
        q_dot =[
            (q1[0] - q0[0]) / dt,
            (q1[1] - q0[1]) / dt,
            (q1[2] - q0[2]) / dt,
            (q1[3] - q0[3]) / dt
            ]
        
        q_inv = qinv(q0);
        r = qqmul(q_dot, q_inv)
        w = 2*r[1::]
        omega.append(w.tolist())
    return np.asarray(omega)

def cableOmega(cables, dt):
    num_of_cables = int(len(cables[0])/2)

    omega = [0]
    for i in range(len(el)-1):
        e0 = el[i]
        e1 = el[i+1]
        omega.append((e1-e0)/dt)
    return omega


def unitvecs(cables): 
    qcs = []
    num_of_cables = int(len(cables[0])/2)
    for cable in cables:
        tmp = []
        for cableNum in range(num_of_cables):
            tmp.append(polartovector(cable[2*cableNum: 2*cableNum+2]))
        qcs.append(tmp) 
    return qcs

def qdots(qcs, dt, num_of_cables):
    qcsd = [[np.zeros(3,).tolist() for cableNum in range(num_of_cables)]]
    for idx, qc in enumerate(qcs[0:-1]): 
        tmp = []
        for cableNum in range(num_of_cables):
            qc0 = np.array(qc[cableNum])
            qc1 = np.array(qcs[idx+1][cableNum])
            tmp.append(((qc1 - qc0)/dt).tolist())
        qcsd.append(tmp)
    return qcsd
            

def omegaCable(qcs, qcsd, num_of_cables):
    omega = [[np.zeros(3,).tolist() for cabeNum in range(num_of_cables)]] # initial state
    for qc, qcd in zip(qcs[0:-1], qcsd[0:-1]):
        tmp = []
        for cableNum in range(num_of_cables):
            qci = np.array(qc[cableNum])
            qcdi = np.array(qcd[cableNum])
            tmp.append(np.cross(qci, qcdi).tolist())
        omega.append(tmp)
    return omega


def polartovector(cablestate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = [mt.cos(az)*mt.cos(el),
                mt.sin(az)*mt.cos(el),
                -mt.sin(el)]
    return unitvec

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", type=str, help="yaml file for the table")
    parser.add_argument("--out", default=None,  type=str, help="yaml file for the table")
    parser.add_argument("-w", "--write", action="store_true")  # on/off flag: write yaml file

    args = parser.parse_args()
    write = args.write

    dt = 1/FREQUENCY    
    inp = args.inp
    out = args.out

    with open(args.inp, "r") as f:
        plan = yaml.load(f, Loader=yaml.FullLoader) 

    states = plan["result"][0]["states"]
    l = plan["result"][1]["cablelengths"]
    attPts = plan["result"][2]["cablepoints"]
    payloadType = plan["result"][4]["payload"][0]["type"]

    # payload position
    p_load = np.asarray([state[0:3] for state in states])   
    # payload velocity 
    v_load = derivative(p_load, dt)

    # cables states
    cableSt = np.empty(3,)
    if payloadType == "sphere":
        cableSt  = np.asarray([state[7::] for state in states])
    elif payloadType == "rigid":
        cableSt  = np.asarray([state[13::] for state in states])
    else:
        print("payload type is wrong please check it: sphere or rigid")
        exit()
    
    num_of_cables = int(len(cableSt[0])/2)

    # cable directional vectors
    qcs = unitvecs(cableSt)
    # cable directional derivative
    qcsd = qdots(qcs, dt, num_of_cables)
    cable_omegas = omegaCable(qcs, qcsd, num_of_cables)
   
    finalStates = []
    actions = []
    oldStateRep = False
    for i, (pos, vel, qc, wc) in enumerate(zip(p_load, v_load, qcs, cable_omegas)):
        if i < len(v_load)-1:
            actions.append(np.ones(4*num_of_cables,).tolist())
        quat = [0,0,0,1]
        w = [0,0,0]
        if oldStateRep: 
            finalState = [*pos.tolist(), *qc[0], *vel.tolist(), *wc[0], *quat, 0,0,0]
            finalStates.append(finalState)
        else:
            tmp = []
            for qc_, wc_ in zip(qc, wc): 
                tmp.extend(qc_)
                tmp.extend(wc_)
            for i in range(num_of_cables):
                tmp.extend(quat)
                tmp.extend(w)
            finalState = [*pos.tolist(), *vel.tolist(), *tmp]
            
            finalStates.append(finalState)
    output = {}
    output["feasible"] = 0
    output["cost"] = 10
    output["result"] = {}
    output["result"]["states"] = finalStates
    output["result"]["actions"] = actions 
    output["result"]["dt"] = dt

    if write:
        with open(args.out, 'w') as file:
            yaml.safe_dump(output, file, default_flow_style=None)

if __name__ == "__main__":
    main()