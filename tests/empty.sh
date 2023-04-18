cd ../examples/empty
FOLDER1=stats

if [ ! -d $FOLDER1 ] 
then
    mkdir $FOLDER1
fi

cd $FOLDER1
FILE="ompl_stats$1.yaml"

if [ ! -d $FILE ] 
then
    touch $FILE
fi

FOLDER2=run$3

if [ ! -d $FOLDER2 ] 
then
    mkdir $FOLDER2
fi

cd ../../../build
./nUavsPayloadPlanner --input ../examples/empty/$2cfs.yaml --output ../examples/empty/$FOLDER1/$FOLDER2/output$1.yaml --stats ../examples/empty/$FOLDER1/$FOLDER2/ompl_stats$1.yaml