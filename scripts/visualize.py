import numpy as np
import time
import rowan as rn
import yaml
import argparse

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

def normalize(v):
    return v/np.linalg.norm(v)

    
def polartovector(cablestate, attpoint, length, plstate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    plpos = plstate[0:3]
    plquat = plstate[3:7]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = np.array([np.cos(az)*np.cos(el),
                        np.sin(az)*np.cos(el),
                        np.sin(el)]) 
    
    attpInfixedFr = plpos + rn.rotate(plquat, attpoint) # attachment point in fixed frame
    uavpos = attpInfixedFr + length*unitvec 
    return uavpos, np.linspace(attpInfixedFr, uavpos, num=2).T

def main(): 
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
        tf.translation_matrix([0.5, 0, 0]).dot(
            tf.euler_matrix(np.radians(-25), np.radians(0), np.radians(-35))))

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
    # motions consist of: 
        # - payload states: [x y z qw qx qy qz az elv]
    cablelengths = motions['result'][1]['cablelengths'][0]
    numofCables = 2*len(cablelengths)
    numofuavs   = len(cablelengths)
    attpoints = motions['result'][2]['cablepoints'][0]
    numofAtts = len(attpoints)
    ## draw obstacles
    if (motions['result'][3]['obstacles']):
        obstacles = motions['result'][3]['obstacles']
        obsNum = 0
        for obstacle in obstacles:
            center = obstacle["center"]
            radius = obstacle["radius"]
            if obstacle["type"] == "cylinder":
                height = obstacle["height"]
                vis["obstacle"+str(obsNum)].set_object(g.Mesh(g.Cylinder(height, radius=radius)))
                ai = np.pi/2
                aj = 0 
                ak = 0
                vis["obstacle"+str(obsNum)].set_transform(tf.translation_matrix(center).dot(tf.euler_matrix(ai, aj, ak)))
            elif obstacle["type"] == "sphere":
                vis["obstacle"+str(obsNum)].set_object(g.Mesh(g.Sphere(radius)))
                vis["obstacle"+str(obsNum)].set_transform(tf.translation_matrix(center))
            obsNum+=1
    ## set objects for the uavs in a list
    uavsphere = []
    path  = mcconfigs['uav']['path']
    shape = mcconfigs['uav']['sphere']         
    for i in range(numofuavs):
        uavshape = vis["uav"+str(i)]
        uavconst = vis["sphere"+str(i)]
        uavshape.set_object(g.StlMeshGeometry.from_file(path))
        uavconst.set_object(g.Mesh(g.Sphere(shape["radius"]), g.MeshLambertMaterial(color=shape["color"], opacity=shape["opacity"])))
        uavsphere.append((uavshape, uavconst))

    while True:
        for motion in motions.values():
            for plstate in motion[0]["states"]:
                vis["payload"].set_transform(
                            tf.translation_matrix(plstate[0:3]).dot(
                    tf.quaternion_matrix(plstate[3:7])))                
                # visualization of uavs, constraint spheres, and cables
                cablestates = plstate[7::]
                for cablecounter, attcounter, length, uavcounter in zip(range(0,numofCables,2), range(0,numofAtts,3), cablelengths, range(numofuavs)):
                    cablestate = cablestates[cablecounter:cablecounter+2]
                    attpoint = attpoints[attcounter:attcounter+3]
                    uavpos, cablegeometry = polartovector(cablestate, attpoint, length, plstate)
                    # visualize cables
                    vis["cable"+str(cablecounter)].set_object(g.Line(g.PointsGeometry(cablegeometry)))
                    # visualize uavs
                    uav, sphere = uavsphere[uavcounter]
                    uav.set_transform(tf.translation_matrix(uavpos))
                    sphere.set_transform(tf.translation_matrix(uavpos))
                time.sleep(mcconfigs['meshcat']["timestep"])
        time.sleep(mcconfigs['meshcat']["finalstep"])

    
if __name__ == "__main__": 
    main()