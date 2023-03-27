import numpy as np
import time
import rowan as rn
import yaml
import argparse
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


FREQUENCY = 100 #Hz

def derivative(vec, dt):
    dvec  =[]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp)
    return np.asarray(dvec)


## from : https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L904
def qinv(q):
    return rn.conjugate(q)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L896
def qqmul(q, p):
    return rn.multiply(q, p)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L989
def quat2omega(quat,dt):
    omega = []
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
    return np.asarray(omega);

def polartovector(cablestate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = [np.cos(az)*np.cos(el),
                np.sin(az)*np.cos(el),
                np.sin(el)]
    return unitvec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("motions", type=str, help="output file containing solution")
    args = parser.parse_args()

    with open(args.motions) as motions_file:
        motions = yaml.load(motions_file, Loader=yaml.FullLoader)

    dt = 1/FREQUENCY
    states = motions["result"][0]["states"]
    
    # payload postion
    position = np.asarray([state[0:3] for state in states])
    
    # velocity postion
    velocity = derivative(position, dt)
    
    # acceleration postion
    acceleration = derivative(velocity, dt)
    
    #time array
    time = [0]
    for i in range(len(position)-1):
        time.append(time[i]+dt)

    # payload orientation
    quat     = [state[3:7] for state in states]
    euler    = rn.to_euler(rn.normalize(quat))
   
   # payload omega
    omega    = quat2omega(quat, dt)
    #cables unit vec
    cables   = np.asarray([state[7::] for state in states])
   
    with PdfPages('result.pdf') as pdf:
        # payload position
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("px [m]")
        axs[1].set_ylabel("py [m]")
        axs[2].set_ylabel("pz [m]")
        axs[-1].set_xlabel("Time [s]")
        axs[0].plot(time,  np.asarray([pos[0] for pos in position]), label='pos')
        axs[1].plot(time,  np.asarray([pos[1] for pos in position]), label='pos')
        axs[2].plot(time,  np.asarray([pos[2] for pos in position]), label='pos')
        axs[0].legend()
        pdf.savefig(fig)
        plt.close()
        # payload velocity
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("vx [m/s]")
        axs[1].set_ylabel("vy [m/s]")
        axs[2].set_ylabel("vz [m/s]")
        axs[-1].set_xlabel("Time [s]")

        axs[0].plot(time[0:-1],  np.asarray([vel[0] for vel in velocity]), label='vel')
        axs[1].plot(time[0:-1],  np.asarray([vel[1] for vel in velocity]), label='vel')
        axs[2].plot(time[0:-1],  np.asarray([vel[2] for vel in velocity]), label='vel')
        axs[0].legend()
        pdf.savefig(fig)
        plt.close()
        # payload acceleration
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("ax [m/s^2]")
        axs[1].set_ylabel("ay [m/s^2]")
        axs[2].set_ylabel("az [m/s^2]")
        axs[-1].set_xlabel("Time [s]")
        axs[0].plot(time[0:-2],  np.asarray([acc[0] for acc in acceleration]), label='acc')
        axs[1].plot(time[0:-2],  np.asarray([acc[1] for acc in acceleration]), label='acc')
        axs[2].plot(time[0:-2],  np.asarray([acc[2] for acc in acceleration]), label='acc')
        axs[0].legend()
        pdf.savefig(fig)
        plt.close()
        #payload euler angles
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("r [rad]")
        axs[1].set_ylabel("p [rad]")
        axs[2].set_ylabel("y [rad]")
        axs[-1].set_xlabel("Time [s]")
        axs[0].plot(time,  np.asarray([angle[0] for angle in euler]), label='rpy')
        axs[1].plot(time,  np.asarray([angle[1] for angle in euler]), label='rpy')
        axs[2].plot(time,  np.asarray([angle[2] for angle in euler]), label='rpy')
        axs[0].legend()
        pdf.savefig(fig)
        plt.close()

        # payload omega
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("wx [rad/s]")
        axs[1].set_ylabel("wy [rad/s]")
        axs[2].set_ylabel("wz [rad/s]")
        axs[-1].set_xlabel("Time [s]")
        axs[0].plot(time[0:-1],  np.asarray([w[0] for w in omega]), label='w')
        axs[1].plot(time[0:-1],  np.asarray([w[1] for w in omega]), label='w')
        axs[2].plot(time[0:-1],  np.asarray([w[2] for w in omega]), label='w')
        axs[0].legend()
        pdf.savefig(fig)
        plt.close()
        # cables qi (unit vec)
        num_of_cables = int(len(cables[0])/2)
        for cableNum in range(num_of_cables):
            cable_i = []
            cable_i.append([cable[0+2*cableNum: 2+2*cableNum].tolist() for cable in cables])
            unitvec = []
            for az_el in cable_i[0]:
                unitvec.append(polartovector(az_el))
            fig, axs = plt.subplots(3, 1, sharex=True)
            axs[0].set_ylabel("qx")
            axs[1].set_ylabel("qy")
            axs[2].set_ylabel("qz")
            axs[-1].set_xlabel("Time [s]")
            axs[0].plot(time,  np.asarray([q[0] for q in unitvec]), label='q'+str(cableNum))
            axs[1].plot(time,  np.asarray([q[1] for q in unitvec]), label='q'+str(cableNum))
            axs[2].plot(time,  np.asarray([q[2] for q in unitvec]), label='q'+str(cableNum))
            axs[0].legend()
            pdf.savefig(fig)
            plt.close()

if __name__ == "__main__": 
    main()