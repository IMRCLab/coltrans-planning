cd ../examples/takeoff
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
./nUavsPayloadPlanner --input ../examples/takeoff/takeoff_2robots.yaml --output ../examples/takeoff/output.yaml --stats ../examples/takeoff/stats/$FILE --timelimit 100
cd ../scripts
python3 visualize.py ../examples/takeoff/output.yaml pointmass