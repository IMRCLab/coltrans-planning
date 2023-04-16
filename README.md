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
