# coltrans-planning

## Building

Tested on Ubuntu 22.04.

```
mkdir build
cd build
cmake ..
make
```

## Running

```
cd build
./payloadPlanner --input ../examples/payload/payload.yaml --output ../examples/payload/output.yaml
```

## Visualize
```
cd scripts
 python3 visualize.py ../examples/payload/output.yaml rod
```
- For different payloads: currently we support: rod, cuboid, triangle and pointmass. 