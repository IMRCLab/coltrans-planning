cd ../build
./test_fcl --input ../examples/test-fcl/test-fcl.yaml --output ../examples/test-fcl/output.yaml
cd ../scripts
python3 test_fcl.py ../examples/test-fcl/output.yaml