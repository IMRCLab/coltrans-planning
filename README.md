# Colaborative Transportation Planning

## Building

Tested on Ubuntu 22.04.

```bash
mkdir build
cd build
cmake ..
make
```

## Running

```bash
cd examples/2cfs_pointmass
mkdir stats/
touch ompl_stats.yaml
cd ../../build
./nUavsPayloadPlanner --input ../examples/2cfs_pointmass/2cfs_pointmass.yaml --output output.yaml --stats ../examples/2cfs_pointmass/stats/ompl_stats.yaml
```
## Scenarios
- you can run it using the commands above or for maze, forest and empty:
```bash 
   cd tests/
   ./maze 0 3
   ./forest 3 4
   ./empty 5 8
```
- The first argument is the experiment number (you can have multiple experiments to compare).
- The second argument is the number of uavs. Now we support from 3 uavs to 8.
- If you want to run any of the other tests you do not need any arguments (i.e., it is only needed for the scenarios): 
  ```bash
  cd tests/
  ./2cfs_payload
  ./3cfs_payload
  ./3cfs_pointmass
  ```
## Visualize

```bash
python3 ../scripts/visualize.py output.yaml pointmass
```

- For different payloads: currently we support: rod, cuboid, triangle and pointmass.

## Shell scripts to run examples and visualize with one command

```bash
cd tests/
./2cfs_pointmass.sh
```

## Plot

```bash
 python3 ../scripts/plot.py output.yaml
```

## Compute Statistics

```bash
cd scripts/
python3 compute_stats.py
```
- To change configuration, check `scripts/compute_stats.yaml`. Three scenarios (the configs are in examples folder) are provided: empty, maze and forest. All of them have the same config parameters (i.e., runs, rangeofRobots: which number of robots to include) except forest, has some extra ones like the number of Obstacles, [min_env, max_env]: the minimum and maximum space where the random obstacles can be.
- The stats for the latest run is found inside `exampls/forest/stats`
- The output of each run to visualize can be found inside examples/forest/stats/run3
- The script generates a dict output: `stadtdict.yaml`.
- For the forest example, we provide a random generation obstacles script `compute_obs.py, that generates and visualizes obstacles.yaml.
- In case the obstacles.yaml is a good example rename the file to obstacles_final.yaml.
 ``` bash
cp -a obstacles.yaml obstacles_final.yaml
```
- otherwise, the compute_stats.py will complain about not finding the `obstacles_final.yaml`
- To visualize any result, choose scenario and which run and which number of robots (i.e., output6.yaml means for 6 robots, output3.yaml, 3 robots and so on):
```bash
cd scripts
python3 visualize.py ../examples/maze/run4/output6.yaml triangle
```