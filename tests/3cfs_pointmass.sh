cd ../build
./nUavsPayloadPlanner --input ../examples/3cfs_pointmass/3cfs_pointmass.yaml --output ../examples/3cfs_pointmass/output.yaml
cd ../scripts
python3 visualize.py ../examples/3cfs_pointmass/output.yaml pointmass