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
cd build
./nUavsPayloadPlanner --input ../examples/2cfs_pointmass/2cfs_pointmass.yaml --output output.yaml
```

## Visualize

```bash
python3 ../scripts/visualize.py output.yaml pointmass
```

- For different payloads: currently we support: rod, cuboid, triangle and pointmass.

## Plot

```bash
 python3 ../scripts/plot.py output.yaml
```
