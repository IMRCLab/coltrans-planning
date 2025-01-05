import yaml
from pathlib import Path
import shutil
import subprocess
from dataclasses import dataclass
import multiprocessing as mp
import tqdm
import psutil
from compute_errors import compute_errors
import traceback
import shutil
import paper_tables

@dataclass
class ExecutionTask:
	"""Class for keeping track of an item in inventory."""
	# env: Path
	# cfg: Path
	# result_folder: Path
	instance: str
	env: str
	model_path: str
	alg: str
	trial: int
	timelimit_geom: float
	timelimit_opt: float

def run_geom(filename_env, folder, timelimit, robot_radius):
	folder = Path(folder)
	try:
		with open(folder / "log.txt", 'w') as f:
			subprocess.run(["./nUavsPayloadPlanner",
						"--input", filename_env,
						"--output", folder / "output.yaml",
						"--stats" , folder / "stats.yaml",
						"--timelimit", str(timelimit),
						"--robot_radius", str(robot_radius),],
						stdout=f, stderr=f, check=True, timeout=timelimit+60)
	except Exception as e:
		print(e)

def add_init_cable_states(folder, envName=None):
	folder = Path(folder)
	traj = "init_guess.yaml"
	subprocess.run(["python3",
			"../scripts/init_cables.py",
			"--inp", folder / traj,
			"--out", folder / traj,
			"--envName", envName], check=True)

def gen_unicycle_init_guess(folder, envName):
	folder = Path(folder)
	traj = "init_guess.yaml"
	subprocess.run(["python3",
		"../scripts/init_guess_unicycle.py",
		"--inp", folder / "output.yaml",
		"--out", folder / traj,
		"--envName", envName,
		"-w"], check=True)	

def gen_ref_init_guess(folder, envName=None):
	folder = Path(folder)
	traj = "init_guess.yaml"
	if envName is not None:
		# here we generate the initial guess for the optimizer, 
		# we only compute the geometric components and update the envs start and goal of dynobench
		subprocess.run(["python3",
			"../scripts/init_guess.py",
			"--inp", folder / "output.yaml",
			"--out", folder / traj,
			"--envName", envName,
			"-w"], check=True)
	else: 
		# -r: compute the velocity components for the geometric planner reference trajectory, 
		# otherwise the controller will not be able to track the traj 
		subprocess.run(["python3",
			"../scripts/init_guess.py",
			"--inp", folder / "output.yaml",
			"--out", folder / traj,
			"-w", 
			"-r"], check=True)

def run_unicycles_controller(folder, reftrajectory, output, model_path):
	subprocess.run(["python3",
				"../deps/dynoplan/dynobench/example/unicycle_sim.py",
				"-w",
				"--inp", folder / reftrajectory,
				"--out", folder / output,
				"--model_path", model_path,
				], env={"PYTHONPATH": "deps/dynoplan/dynobench"}, check=True)

def run_unicycles_visualizer(filename_env, reference_traj, filename_result, filename_output):
	subprocess.run(["python3",
		"../scripts/visualize_unicycles.py",
		"--env", str(filename_env),
		"--robot", "unicycle",
		"--ref", reference_traj,
		"--result", str(filename_result),
		"--output", str(filename_output)],
		check=True)

def run_controller(folder, reftrajectory, output, model_path):
	folder = Path(folder)
	subprocess.run(["python3",
		"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
		"-cff", "-w",
		"--inp", folder / reftrajectory,
		"--out", folder / output,
		"--model_path", model_path,
		], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"}, check=True)

		

def run_visualizer(filename_env, reference_traj, filename_result, filename_output):
	subprocess.run(["python3",
		"../scripts/visualize_payload.py",
		"--env", str(filename_env),
		"--robot", "point",
		"--ref", reference_traj,
		"--result", str(filename_result),
		"--output", str(filename_output)],
		check=True)
    

def run_opt(filename_init, filename_env, folder, timelimit, t_weight=None, t_ref=None):
	folder = Path(folder)
	try:
		if t_ref is None and t_weight is None: 
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
					"--solver_id", "1",
					"--max_iter", "50",
					"--collision_weight", "500.",
					"--weight_goal", "200",
					"--models_base_path", "../deps/dynoplan/dynobench/models/",
					"--results_file", folder / "output"],
					stdout=f, stderr=f, timeout=timelimit, check=True)

		elif t_ref is not None and t_weight is None:
			
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
					"--models_base_path", "../deps/dynoplan/dynobench/models/",
					"--results_file", folder / "output",
					"--solver_id", "1",
					"--max_iter", "50",
					"--time_ref", t_ref],
					stdout=f, stderr=f, timeout=timelimit, check=True)

		elif t_weight is not None and t_ref is None: 
			
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
					"--models_base_path", "../deps/dynoplan/dynobench/models/",
					"--results_file", folder / "output",
					"--solver_id", "1",
					"--max_iter", "50",					
					"--time_weight", t_weight],
					stdout=f, stderr=f, timeout=timelimit, check=True)

		elif t_weight is not None and t_ref is not None: 
			
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
					"--models_base_path", "../deps/dynoplan/dynobench/models/",
					"--results_file", folder / "output",
					"--time_weight", t_weight,
					"--solver_id", "1",
					"--max_iter", "50",
					"--time_ref", t_ref],
					stdout=f, stderr=f, timeout=timelimit, check=True)
	except Exception as e:
		print(e)

def run_checker(filename_env, filename_result, filename_log):
	with open(filename_log, 'w') as f:
		cmd = ["./deps/dynoplan/dynobench/check_trajectory",
					"--result_file", filename_result,
					"--env_file", filename_env,
					"--models_base_path" , "../deps/dynoplan/dynobench/models/",
					"--goal_tol" , "999",
					"--u_bound_tol", "0.101",
					"--col_tol", "0.01"]
		print(subprocess.list2cmdline(cmd))
		out = subprocess.run(cmd,
					stdout=f, stderr=f)
	return out.returncode == 0

def inflate_obstacles(filename_env_in, filename_env_out, inflation=0.0):
	with open(filename_env_in, "r") as env_file:
		env = yaml.safe_load(env_file)

	for o in env["environment"]["obstacles"]:
		for i in range(len(o["size"])):
			o["size"][i] += inflation

	with open(filename_env_out, "w") as env_file:
		yaml.safe_dump(env, env_file, default_flow_style=None)

def execute_task(task: ExecutionTask):
	results_path = Path("../results")
	# tuning_path = Path("../tuning")
	if "point" in task.model_path:
		env_path = Path().resolve() / "../examples/benchmark"
	elif "unicycle" in task.model_path:
		env_path = Path().resolve() / "../deps/dynoplan/dynobench/envs/benchmark_planners/coltransplanning/"
	env = (env_path / task.instance).with_suffix(".yaml")
	assert(env.is_file())

	try:

		result_folder = results_path / task.instance / "geom" / "{:03d}".format(task.trial)
		if result_folder.exists():
				print("Warning! {} exists already. Deleting...".format(result_folder))
				shutil.rmtree(result_folder)
		result_folder.mkdir(parents=True, exist_ok=False)


		# inflate obstacles
		inflate_obstacles(env, result_folder / "env_inflated.yaml")

		with open("../deps/dynoplan/dynobench/models/" + task.model_path, "r") as f:
			model_params = yaml.load(f,Loader=yaml.CSafeLoader)
		if "point" in task.model_path:
			robot_radius = model_params["col_size_robot"]
		else:
			robot_radius = 0
		# run_geom -> input:env output: output.yaml
		run_geom(str(result_folder / "env_inflated.yaml"), str(result_folder), task.timelimit_geom, robot_radius)

		# geometric baseline

		# gen_ref_init_guess -> inp: output.yaml + "-r" , output: reference trajectory geom_ref_traj.yaml
		if "point" in task.model_path:
			gen_ref_init_guess(str(result_folder))

		# optimization-based solution

		result_folder_geom = result_folder
		result_folder = results_path / task.instance / "opt" / "{:03d}".format(task.trial)
		if result_folder.exists():
				print("Warning! {} exists already. Deleting...".format(result_folder))
				shutil.rmtree(result_folder)
		result_folder.mkdir(parents=True, exist_ok=False)

		# copy output.yaml file
		shutil.copy(result_folder_geom / "output.yaml", result_folder)

		# inflate obstacles
		inflate_obstacles("../deps/dynoplan/dynobench/envs/benchmark_planners/coltransplanning/" + task.env, result_folder / "env_inflated.yaml")

		# gen_ref_init_guess -> inp: output.yaml, output: initial guess for optimizer
		if "point" in task.model_path:
			gen_ref_init_guess(str(result_folder), envName=result_folder / "env_inflated.yaml")
		elif "unicycle" in task.model_path:
			gen_unicycle_init_guess(str(result_folder), result_folder / "env_inflated.yaml")

		# filename_init, filename_env, folder, timelimit
		run_opt(result_folder / "init_guess.yaml", str(result_folder / "env_inflated.yaml"), str(result_folder), task.timelimit_opt)
		
		if "point" in task.model_path:
			# run_controller -> input: reference trajecetory to be tracked (output.trajopt.yaml), output: controller output (trajectory_opt.yaml)
			# TODO: do not forget to pass the model path
			run_controller(result_folder, "output.trajopt.yaml", "trajectory_opt.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path)

			# filename_env, reference_traj, filename_result, filename_output
			run_visualizer(result_folder / "env_inflated.yaml", result_folder / "output.trajopt.yaml", result_folder / "trajectory_opt.yaml", result_folder / "trajectory_opt.html")
		
		elif "unicycle" in task.model_path:
			run_unicycles_controller(result_folder, "output.trajopt.yaml", "trajectory_opt.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path)
			run_unicycles_visualizer(result_folder / "env_inflated.yaml", result_folder / "output.trajopt.yaml", result_folder / "trajectory_opt.yaml", result_folder / "trajectory_opt.html")

		run_checker(result_folder / "env_inflated.yaml",
					result_folder / "trajectory_opt.yaml", (result_folder / "trajectory_opt.yaml").with_suffix(".check.txt"))


	except:
		traceback.print_exc()

def main():
	parallel = True
	instances = [
		{ "name": "forest_2robots", "models_path": "point_2.yaml"},
		{ "name": "forest_3robots", "models_path": "point_3.yaml"},
		{ "name": "forest_4robots", "models_path": "point_4.yaml"},
		{ "name": "forest_5robots", "models_path": "point_5.yaml"},
		{ "name": "forest_6robots", "models_path": "point_6.yaml"},

		{ "name": "window_2robots", "models_path": "point_2.yaml"},
		{ "name": "window_3robots", "models_path": "point_3.yaml"},
		{ "name": "window_4robots", "models_path": "point_4.yaml"},
		{ "name": "window_5robots", "models_path": "point_5.yaml"},
		{ "name": "window_6robots", "models_path": "point_6.yaml"},

        {"name": "window_2robots_unicycle", "models_path": "unicyclesWithRods_2.yaml"},
        {"name": "window_3robots_unicycle", "models_path": "unicyclesWithRods_3.yaml"},
        {"name": "window_4robots_unicycle", "models_path": "unicyclesWithRods_4.yaml"},
        {"name": "window_5robots_unicycle", "models_path": "unicyclesWithRods_5.yaml"},
        {"name": "window_6robots_unicycle", "models_path": "unicyclesWithRods_6.yaml"},

        {"name": "forest_2robots_unicycle", "models_path": "unicyclesWithRods_2.yaml"},
        {"name": "forest_3robots_unicycle", "models_path": "unicyclesWithRods_3.yaml"},
        {"name": "forest_4robots_unicycle", "models_path": "unicyclesWithRods_4.yaml"},
        {"name": "forest_5robots_unicycle", "models_path": "unicyclesWithRods_5.yaml"},
        {"name": "forest_6robots_unicycle", "models_path": "unicyclesWithRods_6.yaml"},

        # {"name": "lego_3robots_unicycle", "models_path": "unicyclesWithRods_3.yaml"},

	]
	algs = [
		"opt",
	]
	# trials = 3
	trials = [i for i in range(10)]
	timelimit_geom = 350
	timelimit_opt = 15*60
	max_cpus = 32 # limit the number of CPUs due to high memory usage

	tasks = []
	for instance in instances:
		env = instance["name"].replace("_uniform", "") + ".yaml"
		for alg in algs:
			# "geom" is implicitly executed with "opt", so don't execute here
			if alg == "geom" and "opt" in algs:
				continue
			for trial in trials:
				tasks.append(ExecutionTask(instance["name"], env, instance["models_path"], alg, trial, timelimit_geom, timelimit_opt))

	if parallel and len(tasks) > 1:
		use_cpus = min(max_cpus, psutil.cpu_count(logical=False)-1)
		print("Using {} CPUs".format(use_cpus))
		with mp.Pool(use_cpus) as p:
			for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
				pass
	else:
		for task in tasks:
			execute_task(task)
	trials_ = []
	for i in trials:
		if i <= 9:
			trials_.append("00"+str(i))
		else: 
			trials_.append("0"+str(i))

	# compute_errors([instance["name"] for instance in instances], algs, trials_)

	# paper_tables.write_table1(Path("../results"), trials_)
	# paper_tables.write_plot1(Path("../results"), trials_, timelimit_geom)
	# paper_tables.runtime_results(Path("../results"), trials_)
if __name__ == '__main__':
	main()
