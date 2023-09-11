cd ../examples/3cfs_payload
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
./nUavsPayloadPlanner --input ../examples/3cfs_payload/3cfs_payload.yaml --output ../examples/3cfs_payload/output.yaml --stats ../examples/3cfs_payload/stats/$FILE --timelimit 40
cd ../scripts
python3 visualize.py ../examples/3cfs_payload/output.yaml triangle