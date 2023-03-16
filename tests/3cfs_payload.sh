cd ../build
./nUavsPayloadPlanner --input ../examples/3cfs_payload/3cfs_payload.yaml --output ../examples/3cfs_payload/output.yaml
cd ../scripts
python3 visualize.py ../examples/3cfs_payload/output.yaml triangle