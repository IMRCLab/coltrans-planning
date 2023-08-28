cd ../examples/1cf_pointmass
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
./nUavsPayloadPlanner --input ../examples/1cfs_pointmass/1cfs_pointmass.yaml --output ../examples/1cfs_pointmass/output.yaml --stats ../examples/1cfs_pointmass/stats/$FILE
cd ../scripts
python3 visualize.py ../examples/1cfs_pointmass/output.yaml pointmass