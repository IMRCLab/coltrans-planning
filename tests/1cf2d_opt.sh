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
./nUavsPayloadPlanner --input ../examples/1cf_pointmass2d/1cf_pointmass2d.yaml --output ../examples/1cf_pointmass2d/output.yaml --stats ../examples/1cf_pointmass2d/stats/$FILE
cd ../scripts
# python3 visualize.py ../examples/1cf_pointmass2d/output.yaml pointmass
python3 init_guess.py --inp ../examples/1cf_pointmass2d/output.yaml --out ../deps/dynoplan/dynobench/envs/quad2dpole_v0/trajectories/quad2dpole_init_guess.yaml

cd ../build

deps/dynoplan/main_optimization --env_file ../deps/dynoplan/dynobench/envs/quad2dpole_v0/quad2dpole_v0_obs/quad2dpole_obs.yaml --models_base_path ../deps/dynoplan/dynobench/models/ --init_file ../deps/dynoplan/dynobench/envs/quad2dpole_v0/trajectories/quad2dpole_init_guess.yaml --results_file quad2pole

python3 ../deps/dynoplan/dynobench/utils/viewer/viewer_cli.py --robot quad2dpole --env ../deps/dynoplan/dynobench/envs/quad2dpole_v0/quad2dpole_v0_obs/quad2dpole_obs.yaml --result quad2pole.trajopt.yaml -i