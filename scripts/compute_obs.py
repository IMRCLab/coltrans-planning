
import numpy as np
import yaml
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf


def main():
    # Define boundaries
    boundaries_min = np.array([0, -1.2, 0])
    boundaries_max = np.array([3, 1.2, 0])

    # Define tolerance for minimum distance between points
    tolerance = 0.8

    # Initialize points array
    points = np.empty((0, 3))

    # Initialize obstacles list
    obstacles = []

    # Generate points and corresponding obstacles
    for i in range(10000):
        # Generate a new point
        new_point = np.concatenate((np.random.uniform(low=boundaries_min[:2], high=boundaries_max[:2]), [0]))

        # Check distance to all existing points
        distances = np.linalg.norm(points[:, :2] - new_point[:2], axis=1)

        # Add new point to list if distance is greater than the tolerance
        if np.all(distances > tolerance):
            # Append new obstacle to the list
            obstacles.append({'type': 'cylinder',
                            'center': new_point.tolist(),
                            'radius': 0.03,
                            'height': 3.0})
            
            # Append new point to the list of points
            points = np.vstack((points, new_point))
    print(len(obstacles))
    # Convert list of points to NumPy array
    points = np.array(points)
    
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