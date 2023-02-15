import numpy as np
import time
import rowan as rn
import yaml
import argparse

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf



if __name__ == "__main__": 
    # loads the abstract meshcat yaml file.
    # the meshcat.yaml has all rigid body data
    # UAV, payload, cables: 
    # 1.  timestep 
    # 2. payloads and their configs
    # 3. uavs and configs
    with open("../examples/visualize/meshcat.yaml") as mc: 
        mcconfigs = yaml.safe_load(mc) 

    parser = argparse.ArgumentParser()
    parser.add_argument("motions", type=str, help="output file containing solution")
    parser.add_argument("payload", type=str, help="type of payload")
    args = parser.parse_args()

    vis = meshcat.Visualizer()
  
    if mcconfigs['meshcat']['openWindow'] == True:
        vis.open()
        vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(np.radians(-40), np.radians(0), np.radians(-35))))

        vis["/Cameras/default/rotated/<object>"].set_transform(
            tf.translation_matrix([-1, 0, 2]))
  
    # Extract shape of each rigid body
    payloadType = args.payload
    try:
        ploadConfig = mcconfigs['payload'][payloadType] 
    except:
        print('wrong shape: {}'.format(payloadType))
        raise

    if payloadType == 'triangle':
        vis["payload"].set_object(g.StlMeshGeometry.from_file(ploadConfig['path']), 
                                            g.MeshLambertMaterial(color=ploadConfig['color']))
    elif payloadType == 'rod':
        vis["payload"].set_object(g.Mesh(g.Cylinder(ploadConfig['height'], 
                                        radius=ploadConfig['radius']),
                                        g.MeshLambertMaterial(color=ploadConfig['color'])))
    elif payloadType == 'cuboid':
            vis["payload"].set_object(g.Mesh(g.Box(ploadConfig['size']),
                                         g.MeshLambertMaterial(color=ploadConfig['color'])))    
    elif payloadType == 'pointmass':
        vis["payload"].set_object(g.Mesh(g.Sphere(ploadConfig['radius']), 
                                            g.MeshLambertMaterial(color=ploadConfig['color'])))  

    
    with open(args.motions) as motions_file:
        motions = yaml.load(motions_file, Loader=yaml.FullLoader)
    # print(motions.values())
    # motions consist of: 
        # - payload states: [x y z qw qx qy qz]
    while True:
        for motion in motions.values(): 
            for state in motion[0]["states"]:
                vis["payload"].set_transform(
                            tf.translation_matrix(state[0:3]).dot(
                    tf.quaternion_matrix([1, 0, 0, 0])))
            # for state in motion["states"]:
            #     print(state)
                time.sleep(mcconfigs['meshcat']["timestep"])

        
        
        pass