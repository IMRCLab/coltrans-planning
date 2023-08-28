cd ../examples/1cf_pointmass2d
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
./nUavsPayloadPlanner --input ../examples/1cfs_pointmass2d/1cfs_pointmass2d.yaml --output ../examples/1cfs_pointmass2d/output.yaml --stats ../examples/1cfs_pointmass2d/stats/$FILE
cd ../scripts
python3 visualize.py ../examples/1cfs_pointmass2d/output.yaml pointmass