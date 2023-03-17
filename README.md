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
./payloadPlanner --input ../examples/payload/payload.yaml --output ../examples/payload/output.yaml
```

## Visualize

```bash
cd scripts
 python3 visualize.py ../examples/payload/output.yaml rod
```

- For different payloads: currently we support: rod, cuboid, triangle and pointmass.

## Plot

```bash
cd scripts
 python3 plot.py ../examples/payload/output.yaml
```
