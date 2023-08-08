import numpy as np
import yaml
import argparse
import rowan as rn
import math as mt
FREQUENCY = 10 #Hz

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
    unitvec = []
    num_of_cables = int(len(cables[0])/2)
    for cableNum in range(num_of_cables): 
        for cab in cables:
            unitvec.append(polartovector(cab[2*cableNum : 2*cableNum+2])) 
    return unitvec

def qdots(unitvectors, dt, num_of_cables):
    qds = [[0,0,0]]

    for cableNum in range(num_of_cables):
        for i in range(len(unitvectors)-1):
            qi0 = np.asarray(unitvectors[i])
            qi1 = np.asarray(unitvectors[i+num_of_cables])
            qds.append(((qi1 - qi0)/dt).tolist())
    return qds
            
def qddots(qdots, dt, num_of_cables):
    qdds = []
    for cableNum in range(num_of_cables):
        for i in range(len(qdots)-1):
            qdi0 = np.asarray(qdots[i])
            qdi1 = np.asarray(qdots[i+num_of_cables])
            qdds.append(((qdi1 - qdi0)/dt).tolist())
    return qdds
            

def omegaCable(qdts, unitvectors):
    omega = []

    for qd, qi in zip(qdts, unitvectors):
        omega.append(np.cross(qi,qd).tolist())
    return omega


def polartovector(cablestate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = [mt.cos(az)*mt.sin(el),
                mt.sin(az)*mt.cos(el),
                mt.cos(el)]
    return unitvec


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", type=str, help="yaml file for the table")
    parser.add_argument("--out", default=None,  type=str, help="yaml file for the table")
    args = parser.parse_args()

    dt = 1/FREQUENCY    
    l  = 0.25
    inp = args.inp
    out = args.out

    with open(args.inp, "r") as f:
        plan = yaml.load(f, Loader=yaml.FullLoader) 

    states = plan["result"][0]["states"]
    
    # payload position
    position = np.asarray([state[0:3] for state in states])
   
    # payload velocity 
    velocity = derivative(position, dt)
    
    acceleration = derivative(position, dt)
    # cables states
    cables  = np.asarray([state[7::] for state in states])
    num_of_cables = int(len(cables[0])/2)

    # cable directional vectors
    unitvectors = unitvecs(cables)
    
    # cable directional derivative
    qdts = qdots(unitvectors, dt, num_of_cables)
    qddts = qddots(qdts, dt, num_of_cables)
    cable_omegas = omegaCable(qdts, unitvectors)
    # uav position  
    uavPos = cable2uavPos(position, unitvectors, l, num_of_cables)
    uavVel = cable2uavVel(velocity, qdts, l, num_of_cables)
    uavAcc = cable2uavAcc(acceleration, qddts, l, num_of_cables)

    finalStates = []
    actions = []
    count = 0
    for pos, vel, acc, uvec, om in zip(uavPos, uavVel, uavAcc, unitvectors, cable_omegas):
        th = 0
        yaw = mt.atan2(uvec[0], uvec[2])
        d = 1
        M = np.array([[1, 1], [d, -d]])
        if count != 0:
            # m = 2+0.25
            # act_vec = np.asarray([acc[0],vel[2]+ m*9.81])
            # act = act_vec/np.linalg.norm(act_vec)
            # th = mt.atan2(act_vec[0],act_vec[1])
            # f = np.linalg.norm(act)
            # f1f2 = np.linalg.inv(M)@np.array([f, th])
            # actions.append([float(f1f2[0]),float(f1f2[1])])
            actions.append([1,1])
        finalStates.append([pos[0],pos[2], 0, 0, 0, 0, 0, 0])
        count+=1
    output = {}
    output["feasible"] = 0
    output["cost"] = 10
    output["result"] = {}
    output["result"]["states"] = finalStates
    output["result"]["actions"] = actions 

    with open(args.out, 'w') as file:
        yaml.safe_dump(output, file, default_flow_style=None)
if __name__ == "__main__":
    main()