cd ../build
./nUavsPayloadPlanner --input ../examples/2cfs_payload/2cfs_payload.yaml --output ../examples/2cfs_payload/output.yaml
cd ../scripts
python3 visualize.py ../examples/2cfs_payload/output.yaml rod