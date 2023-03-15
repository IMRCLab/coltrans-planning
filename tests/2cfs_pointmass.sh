cd ../build
./nUavsPayloadPlanner --input ../examples/2cfs_pointmass/2cfs_pointmass.yaml --output ../examples/2cfs_pointmass/output.yaml
cd ../scripts
python3 visualize.py ../examples/2cfs_pointmass/output.yaml pointmass