mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH="/opt/openrobots/"
make -j

## run an example of the geomtric planner
./nUavsPayloadPlanner --input ../examples/2cfs_pointmass/2cfs_pointmass.yaml --output ../examples/2cfs_pointmass/output.yaml --stats ../examples/2cfs_pointmass/stats/stats.yaml

## run simulator in dynobench
## first create a build folder inside dynobench
## Currently the simulator has its own reference trajectory, but ideally it will use the output from the geometric planner 
cd ../deps/dynoplan/dynobench/
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH="/opt/openrobots/"
make -j 

## first export the pythonpath of the crazyflie firmware inside the build for the python bindings while inside the build of dynobench
## export PYTHONPATH=$PWD/../crazyflie-firmware 
## to run the simulator and save the "initial guess of the controller here"
python3 ../example/test_quad3dpayload_n.py -cff -w --out ../envs/quad3d_payload/trajectories/initial_guess.yaml
## to view the output of the contorller/ or the initial guess
cd ../util/viewer
python3 viewer_cli.py --robot point --env ../../envs/quad3d_payload/quad3d_payload_one_obs/quad3d_payload_one_obs_0_2_pm_hard.yaml --result ../../envs/quad3d_payload/trajectories/quad3d_payload_2_pm_hover_initial_guess.yaml -i

## To run the optimizer to use this initial guess (or any initial guess inside the trajectories folder)
cd ../../../../../build
./deps/dynoplan/main_optimization --init_file ../deps/dynoplan/dynobench/envs/quad3d_payload/trajectories/quad3d_payload_2_pm_hard_init_guess.yaml --env_file ../deps/dynoplan/dynobench/envs/quad3d_payload/quad3d_payload_one_obs/quad3d_payload_one_obs_0_2_pm_hard.yaml --models_base_path ../deps/dynoplan/dynobench/models/ --results_file quad3d_payload_hard
# to view the output of the optimizer
# You have to run the viewer from its directory because the cf2_assembly.stl is currently downloaded there and it is imported in the coded relative to the script
cd ../deps/dynoplan/dynobench/utils/viewer/
python3 viewer_cli.py --robot point --env ../../envs/quad3d_payload/quad3d_payload_one_obs/quad3d_payload_one_obs_0_2_pm_hard.yaml --result ../../../../../build/quad3d_payload_hard.trajopt.yaml -i