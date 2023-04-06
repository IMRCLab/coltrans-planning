cd ../build
./nUavsPayloadPlanner --input ../examples/2cfs_payload/takeoff.yaml --output ../examples/2cfs_payload/takeoff_2cfs_payload_output.yaml
cd ../scripts
python3 visualize.py ../examples/2cfs_payload/takeoff_2cfs_payload_output.yaml rod