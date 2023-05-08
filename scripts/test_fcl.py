import numpy as np
import time
import rowan as rn
import yaml
import argparse

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf



def unitvec(az, el):
    # returns the points to be visualized for the cable 
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = np.array([np.cos(az)*np.cos(el),
                        np.sin(az)*np.cos(el),
                        np.sin(el)])     
    return unitvec


def main(): 

    parser = argparse.ArgumentParser()
    parser.add_argument("env", type=str, help="output file containing environment")
    args = parser.parse_args()

    vis = meshcat.Visualizer()
  
    vis.open()
    vis["/Cameras/default"].set_transform(
    tf.translation_matrix([0.5, 0, 0]).dot(
        tf.euler_matrix(np.radians(-25), np.radians(0), np.radians(-35))))

    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([-1, 0, 2]))
    
    with open(args.env) as env_file:
        env = yaml.load(env_file, Loader=yaml.FullLoader)
    
    # print(env)
    # exit()
    obstacles = env["obstacles"]

    payload = env["payload"]
    
    while True:
        obsNum = 0
        for obstacle in obstacles:
            center = obstacle["center"]
            radius = obstacle["radius"]
            if obstacle["type"] == "cylinder":
                height = obstacle["height"]
                vis["obstacle"+str(obsNum)].set_object(g.Mesh(g.Cylinder(height, radius=radius)))
                rotation = obstacle["rotation"]
                unitvector = unitvec(rotation[0], rotation[1])
                basevec = [0,1,0]
                quat = rn.vector_vector_rotation(basevec, unitvector)
                vis["obstacle"+str(obsNum)].set_transform(tf.translation_matrix(center).dot(tf.quaternion_matrix(quat)))
            elif obstacle["type"] == "sphere":
                vis['obstacle'+str(obsNum)].set_object(g.Mesh(g.Sphere(radius=radius)))
                vis["obstacle"+str(obsNum)].set_transform(tf.translation_matrix(center))
            obsNum+=1
            if payload['type'] == 'sphere':
                vis['payload'].set_object(g.Mesh(g.Sphere(payload["radius"])))
                vis["payload"].set_transform(tf.translation_matrix(payload["center"]))
            elif payload['type'] == 'box':
                vis['payload'].set_object(g.Mesh(g.Box([0.08, 0.08, 0.005])))
                vis["payload"].set_transform(tf.translation_matrix(payload["center"]))

if __name__ == "__main__": 
    main()