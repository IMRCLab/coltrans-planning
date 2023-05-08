cd ../examples/2cfs_payload
FOLDER=stats

if [ ! -d $FOLDER ] 
then
    mkdir $FOLDER
fi

cd $FOLDER
FILE=ompl_stats.yaml 
if [ ! -d $FILE ] 
then
    touch $FILE
fi

cd ../../../build
./nUavsPayloadPlanner --input ../examples/2cfs_payload/2cfs_payload.yaml --output ../examples/2cfs_payload/output.yaml --stats ../examples/2cfs_payload/stats/$FILE
cd ../scripts
python3 visualize.py ../examples/2cfs_payload/output.yaml rod