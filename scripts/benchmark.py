import yaml
from pathlib import Path
import shutil
import subprocess
from dataclasses import dataclass
import multiprocessing as mp
import tqdm
import psutil
from compute_errors import compute_errors

@dataclass
class ExecutionTask:
	"""Class for keeping track of an item in inventory."""
	# env: Path
	# cfg: Path
	# result_folder: Path
	instance: str
	env: str
	num_robot: int
	alg: str
	trial: int
	timelimit_geom: float
	timelimit_opt: float

def run_geom(filename_env, folder, timelimit):
	folder = Path(folder)
	try:
		with open(folder / "log.txt", 'w') as f:
			subprocess.run(["./nUavsPayloadPlanner",
						"--input", filename_env,
						"--output", folder / "output.yaml",
						"--stats" , folder / "stats.yaml",
						"--timelimit", str(timelimit)],
						stdout=f, stderr=f)
	except Exception as e:
		print(e)

def gen_ref_init_guess(folder, flag, envName=None):
	folder = Path(folder)
	traj = "init_guess.yaml"
	if envName is not None:
		subprocess.run(["python3",
			"../scripts/init_guess.py",
			"--inp", folder / "output.yaml",
			"--out", folder / traj,
			"--envName", envName,
			"-w"])
	else: 
		subprocess.run(["python3",
			"../scripts/init_guess.py",
			"--inp", folder / "output.yaml",
			"--out", folder / traj,
			"-w"])

def run_controller(folder, reftrajectory, output, num_robot, computeAcc=False):
	folder = Path(folder)
	if computeAcc:
		subprocess.run(["python3",
			"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
				"-cff", "-w", "-a",
				"--inp", folder / reftrajectory,
				"--out", folder / output,
				"--num_robots", str(num_robot),
			], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"})
	else: 
		subprocess.run(["python3",
			"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
				"-cff", "-w",
				"--inp", folder / reftrajectory,
				"--out", folder / output,
				"--num_robots", str(num_robot),
			], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"})
	

def run_visualizer(filename_env, reference_traj, filename_result, filename_output):
	subprocess.run(["python3",
		 "../deps/dynoplan/dynobench/utils/viewer/viewer_cli.py",
		 	"--robot", "point",
			"--ref", str(reference_traj),
			"--env", str(filename_env),
			"--result", str(filename_result),
			"--output", str(filename_output)
		 ])

def run_opt(filename_init, filename_env, folder, timelimit):
	folder = Path(folder)
	try:
		with open(folder / "log.txt", 'w') as f:
			subprocess.run(["./deps/dynoplan/main_optimization",
				"--init_file", filename_init,
				"--env_file", filename_env,
				"--models_base_path", "../deps/dynoplan/dynobench/models/",
				"--results_file", folder / "output"],
				stdout=f, stderr=f, timeout=timelimit)
	except Exception as e:
		print(e)

def execute_task(task: ExecutionTask):
	results_path = Path("../results")
	# tuning_path = Path("../tuning")
	env_path = Path().resolve() / "../examples/benchmark"
	env = (env_path / task.instance).with_suffix(".yaml")
	assert(env.is_file())

	result_folder = results_path / task.instance / task.alg / "{:03d}".format(task.trial)
	if result_folder.exists():
			print("Warning! {} exists already. Deleting...".format(result_folder))
			shutil.rmtree(result_folder)
	result_folder.mkdir(parents=True, exist_ok=False)

	if task.alg == "geom":
		# run_geom -> input:env output: output.yaml
		run_geom(str(env), str(result_folder), task.timelimit_geom)
		# gen_ref_init_guess -> inp: output.yaml + "-r" , output: reference trajectory geom_ref_traj.yaml
		gen_ref_init_guess(str(result_folder), "")
		#run_controller -> input: reference trajecetory to be tracked (geom_init_guess.yaml), output: controller output (trajectory_geom.yaml)
		run_controller(result_folder, "init_guess.yaml", "trajectory_geom.yaml", task.num_robot)
		# visualize: reference trajectory from the geometric planner, output of controller tracking the ref traj
		run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env ,result_folder / "init_guess.yaml",  result_folder / "trajectory_geom.yaml", result_folder / "trajectory_geom.html")
	
	if task.alg == "opt":
		run_geom(str(env), str(result_folder), task.timelimit_geom)
		# gen_ref_init_guess -> inp: output.yaml, output: initial guess for optimizer
		gen_ref_init_guess(str(result_folder), "", envName="../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env)
		# filename_init, filename_env, folder, timelimit
		run_opt(result_folder / "init_guess.yaml", "../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env, str(result_folder), task.timelimit_opt)
		# run_controller -> input: reference trajecetory to be tracked (output.trajopt.yaml), output: controller output (trajectory_opt.yaml)
		run_controller(result_folder, "output.trajopt.yaml", "trajectory_opt.yaml", task.num_robot, computeAcc=True)
		# filename_env, reference_traj, filename_result, filename_output
		run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env, result_folder / "output.trajopt.yaml", result_folder / "trajectory_opt.yaml", result_folder / "trajectory_opt.html")

def main():
	parallel = True
	instances = [
	
	"empty_2robots",
    "empty_3robots",
    "empty_4robots",
    "empty_5robots",
    "empty_6robots",
    "forest_2robots",
    "forest_3robots",
    "forest_4robots",
    "forest_5robots",
    "forest_6robots",
    "maze_2robots",
    "maze_3robots",
    "maze_4robots",
    "maze_5robots",
    "maze_6robots",	
	
	]
	envs = [
	"empty_2robots.yaml",
    "empty_3robots.yaml",
    "empty_4robots.yaml",
    "empty_5robots.yaml",
    "empty_6robots.yaml",
    "forest_2robots.yaml",
    "forest_3robots.yaml",
    "forest_4robots.yaml",
    "forest_5robots.yaml",
    "forest_6robots.yaml",
    "maze_2robots.yaml",
    "maze_3robots.yaml",
    "maze_4robots.yaml",
    "maze_5robots.yaml",
    "maze_6robots.yaml",
	]
	num_robots = [
		2,
		3,
		4,
		5,
		6,
		2,
		3,
		4,
		5,
		6,
		2,
		3,
		4,
		5,
		6
	]
	algs = [
		"geom",
		"opt",
	]
	trials = 1
	timelimit_geom = 300
	timelimit_opt = 15*60
	max_cpus = 8 # limit the number of CPUs due to high memory usage

	tasks = []
	for instance, env, num_robot in zip(instances, envs, num_robots):
		for alg in algs:
			for trial in range(trials):
				tasks.append(ExecutionTask(instance, env, num_robot, alg, trial, timelimit_geom, timelimit_opt))

	if parallel and len(tasks) > 1:
		use_cpus = max(max_cpus, psutil.cpu_count(logical=False)-1)
		print("Using {} CPUs".format(use_cpus))
		with mp.Pool(use_cpus) as p:
			for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
				pass
	else:
		for task in tasks:
			execute_task(task)
	trials_ = ["00"+str(i) for i in range(trials)]
	compute_errors(instances, algs, trials_)
if __name__ == '__main__':
	main()
