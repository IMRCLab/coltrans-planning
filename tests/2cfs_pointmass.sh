cd ../examples/2cfs_pointmass
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
./nUavsPayloadPlanner --input ../examples/2cfs_pointmass/2cfs_pointmass.yaml --output ../examples/2cfs_pointmass/output.yaml --stats ../examples/2cfs_pointmass/stats/$FILE --timelimit 30
cd ../scripts
python3 visualize.py ../examples/2cfs_pointmass/output.yaml pointmass