
import numpy as np
import yaml
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf


def main():
    obstacles = []
    numOfObs = 16
    for Obs in range(numOfObs):
        min_env = [0,  -1.5]
        max_env = [1.5, 1.5]
        tol_stg = 0.5
        tol     = 0.3
        center = [np.random.uniform(low=min_env[0], high=max_env[0]), np.random.uniform(low=min_env[1], high=max_env[1]), np.random.uniform(low=-0.0001, high=0.0001)]
        feasible = False
        while not feasible:
            start_tol = np.linalg.norm(np.asarray(center) - np.asarray([-0.5, 0, 0]))
            goal_tol  = np.linalg.norm(np.asarray(center) - np.asarray([2.0, 0, 0]))    
            if Obs > 0:
                currentObs = [obs_['center'] for obs_ in obstacles] 
                isdisBetweenObs = False
                while not isdisBetweenObs:
                    for obst in currentObs:
                        if np.linalg.norm((np.asarray(center))- (np.asarray(obst))) < tol or start_tol < tol_stg or goal_tol < tol_stg:
                            print('not correct')
                            center = [np.random.uniform(low=min_env[0], high=max_env[0]), np.random.uniform(low=min_env[1], high=max_env[1]), np.random.uniform(low=-0.0001, high=0.0001)]
                            isdisBetweenObs = False
                        else:
                            isdisBetweenObs = True
                feasible = True
            else:
                feasible = True
        obstacles.append({'type' : "cylinder",  
                        'center' : center, 
                        'radius' : 0.05,
                        'height' : 3.0})
    
    
    obstaclesdict = {'obstacles': obstacles}
    obstacleYaml=open("obstacles.yaml","w")
    yaml.dump(obstaclesdict, obstacleYaml, default_flow_style=True)
    obstacleYaml.close()
    print("YAML file saved.")

    vis = meshcat.Visualizer()
    vis.open()
    while True:   
        obsNum = 0
        for obstacle in obstacles:
            center = obstacle["center"]
            radius = obstacle["radius"]
            height = obstacle["height"]
            vis["obstacle"+str(obsNum)].set_object(g.Mesh(g.Cylinder(height, radius=radius)))
            ai = np.pi/2
            aj = 0 
            ak = 0
            vis["obstacle"+str(obsNum)].set_transform(tf.translation_matrix(center).dot(tf.euler_matrix(ai, aj, ak)))
            obsNum+=1

if __name__ == "__main__":
    main()