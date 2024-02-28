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

def run_controller(folder, reftrajectory, output, model_path, computeAcc=False, nocableTrack=False):
	folder = Path(folder)
	if nocableTrack:
		subprocess.run(["python3",
			"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
			"-cff", "-w", "-noC",
			"--inp", folder / reftrajectory,
			"--out", folder / output,
			"--model_path", model_path,
			], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"}, check=True)

	else:
		subprocess.run(["python3",
		"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
			"-cff", "-w",
			"--inp", folder / reftrajectory,
			"--out", folder / output,
			"--model_path", model_path,
		], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"}, check=True)

		if computeAcc:
			# this flag activates -a: it computes the mu_planned based on the reference actions
			subprocess.run(["python3",
				"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
					"-cff", "-w", "-a",
					"--inp", folder / reftrajectory,
					"--out", folder / output,
					"--model_path", model_path,
					"-a",
				], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"}, check=True)
		else: 
			subprocess.run(["python3",
				"../deps/dynoplan/dynobench/example/test_quad3dpayload_n.py",
					"-cff", "-w",
					"--inp", folder / reftrajectory,
					"--out", folder / output,
					"--model_path", model_path,
				], env={"PYTHONPATH": "deps/dynoplan/dynobench:../deps/crazyflie-firmware"}, check=True)
		

def run_visualizer(filename_env, reference_traj, filename_result, filename_output):
	subprocess.run(["python3",
		 "../deps/dynoplan/dynobench/utils/viewer/viewer_cli.py",
		 	"--robot", "point",
			"--ref", str(reference_traj),
			"--env", str(filename_env),
			"--result", str(filename_result),
			"--output", str(filename_output)
		 ], check=True)

def run_opt(filename_init, filename_env, folder, timelimit, t_weight=None, t_ref=None):
	folder = Path(folder)
	try:
		if t_ref is None and t_weight is None: 
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
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
					"--time_ref", t_ref],
					stdout=f, stderr=f, timeout=timelimit, check=True)

		elif t_weight is not None and t_ref is None: 
			
			with open(folder / "log.txt", 'w') as f:
				subprocess.run(["./deps/dynoplan/main_optimization",
					"--init_file", filename_init,
					"--env_file", filename_env,
					"--models_base_path", "../deps/dynoplan/dynobench/models/",
					"--results_file", folder / "output",
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
					"--col_tol", "0.02"]
		print(subprocess.list2cmdline(cmd))
		out = subprocess.run(cmd,
					stdout=f, stderr=f)
	return out.returncode == 0

def inflate_obstacles(filename_env_in, filename_env_out, inflation=0.05):
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
	env_path = Path().resolve() / "../examples/benchmark"
	env = (env_path / task.instance).with_suffix(".yaml")
	assert(env.is_file())

	try:
		# if task.alg == "geom":
		# 	# run_geom -> input:env output: output.yaml
		# 	run_geom(str(env), str(result_folder), task.timelimit_geom)
		# 	# gen_ref_init_guess -> inp: output.yaml + "-r" , output: reference trajectory geom_ref_traj.yaml
		# 	gen_ref_init_guess(str(result_folder)) # dont forget to add -r here for the geom planner reference 
		# 	#run_controller -> input: reference trajecetory to be tracked (geom_init_guess.yaml), output: controller output (trajectory_geom.yaml)
		# 	run_controller(result_folder, "init_guess.yaml", "trajectory_geom.yaml", task.num_robots)
		# 	# visualize: reference trajectory from the geometric planner, output of controller tracking the ref traj
		# 	run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env ,result_folder / "init_guess.yaml",  result_folder / "trajectory_geom.yaml", result_folder / "trajectory_geom.html")
		
		if task.alg == "opt" or task.alg == "geom":

			result_folder = results_path / task.instance / "geom" / "{:03d}".format(task.trial)
			if result_folder.exists():
					print("Warning! {} exists already. Deleting...".format(result_folder))
					shutil.rmtree(result_folder)
			result_folder.mkdir(parents=True, exist_ok=False)


			# inflate obstacles
			inflate_obstacles(env, result_folder / "env_inflated.yaml")

			with open("../deps/dynoplan/dynobench/models/" + task.model_path, "r") as f:
				model_params = yaml.load(f,Loader=yaml.CSafeLoader)
			robot_radius = model_params["col_size_robot"]
			# run_geom -> input:env output: output.yaml
			run_geom(str(result_folder / "env_inflated.yaml"), str(result_folder), task.timelimit_geom, robot_radius)

			# geometric baseline

			# gen_ref_init_guess -> inp: output.yaml + "-r" , output: reference trajectory geom_ref_traj.yaml
			gen_ref_init_guess(str(result_folder)) 
			#run_controller -> input: reference trajecetory to be tracked (geom_init_guess.yaml), output: controller output (trajectory_geom.yaml)
			run_controller(result_folder, "init_guess.yaml", "trajectory_geom.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path)

			run_checker("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env,
						result_folder / "trajectory_geom.yaml", (result_folder / "trajectory_geom.yaml").with_suffix(".check.txt"))

			# visualize: reference trajectory from the geometric planner, output of controller tracking the ref traj
			run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env ,result_folder / "init_guess.yaml",  result_folder / "trajectory_geom.yaml", result_folder / "trajectory_geom.html")

			# optimization-based solution
			if task.alg == "opt":

				result_folder_geom = result_folder
				result_folder = results_path / task.instance / "opt" / "{:03d}".format(task.trial)
				if result_folder.exists():
						print("Warning! {} exists already. Deleting...".format(result_folder))
						shutil.rmtree(result_folder)
				result_folder.mkdir(parents=True, exist_ok=False)

				# copy output.yaml file
				shutil.copy(result_folder_geom / "output.yaml", result_folder)

				# inflate obstacles
				inflate_obstacles("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env, result_folder / "env_inflated.yaml")

				# gen_ref_init_guess -> inp: output.yaml, output: initial guess for optimizer
				gen_ref_init_guess(str(result_folder), envName=result_folder / "env_inflated.yaml")

				# filename_init, filename_env, folder, timelimit
				run_opt(result_folder / "init_guess.yaml", str(result_folder / "env_inflated.yaml"), str(result_folder), task.timelimit_opt)

				# run_controller -> input: reference trajecetory to be tracked (output.trajopt.yaml), output: controller output (trajectory_opt.yaml)
				# TODO: do not forget to pass the model path
				run_controller(result_folder, "output.trajopt.yaml", "trajectory_opt.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path, computeAcc=True)

				run_checker("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env,
							result_folder / "trajectory_opt.yaml", (result_folder / "trajectory_opt.yaml").with_suffix(".check.txt"))

				# filename_env, reference_traj, filename_result, filename_output
				run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env, result_folder / "output.trajopt.yaml", result_folder / "trajectory_opt.yaml", result_folder / "trajectory_opt.html")

				previous_result = result_folder / "output.trajopt.yaml"
				for i in range(1, 1):
					result_folder_i = result_folder / "iter{:02d}".format(i)
					result_folder_i.mkdir(parents=True, exist_ok=False)
					run_opt(previous_result, str(result_folder / "env_inflated.yaml"), str(result_folder_i), task.timelimit_opt, t_weight="3.0", t_ref="0.8")
					previous_result = result_folder_i / "output.trajopt.yaml"

				# only run visualizer and checker on last iteration
				result_folder = previous_result.parent

				# run_controller -> input: reference trajecetory to be tracked (output.trajopt.yaml), output: controller output (trajectory_opt.yaml)
				# TODO: do not forget to pass the model path
				run_controller(result_folder, "output.trajopt.yaml", "trajectory_opt.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path, computeAcc=True)

				run_checker("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env,
							result_folder / "trajectory_opt.yaml", (result_folder / "trajectory_opt.yaml").with_suffix(".check.txt"))

				# filename_env, reference_traj, filename_result, filename_output
				run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env, result_folder / "output.trajopt.yaml", result_folder / "trajectory_opt.yaml", result_folder / "trajectory_opt.html")


		if task.alg == "payload":

			result_folder = results_path / task.instance / task.alg / "{:03d}".format(task.trial)
			if result_folder.exists():
					print("Warning! {} exists already. Deleting...".format(result_folder))
					shutil.rmtree(result_folder)
			result_folder.mkdir(parents=True, exist_ok=False)

			num_robots = int(task.model_path[6])
			env_0robots = env.with_name(env.name.replace("_{}robots.yaml".format(num_robots), "_0robots.yaml"))
			# inflate obstacles
			inflate_obstacles(env_0robots, result_folder / "env_inflated.yaml")

			with open("../deps/dynoplan/dynobench/models/" + task.model_path, "r") as f:
				model_params = yaml.load(f,Loader=yaml.CSafeLoader)
			robot_radius = model_params["col_size_robot"]

			# run_geom -> input:env output: output.yaml
			run_geom(str(result_folder / "env_inflated.yaml"), str(result_folder), task.timelimit_geom, robot_radius)

			# gen_ref_init_guess -> inp: output.yaml + "-r" , output: reference trajectory geom_ref_traj.yaml
			gen_ref_init_guess(str(result_folder)) # dont forget to add -r here for the geom planner reference 
			add_init_cable_states(str(result_folder), envName=env)
			
			#run_controller -> input: reference trajecetory to be tracked (geom_init_guess.yaml), output: controller output (trajectory_geom.yaml)
			run_controller(result_folder, "init_guess.yaml", "trajectory_geom.yaml", "../deps/dynoplan/dynobench/models/" + task.model_path, nocableTrack=True)

			run_checker("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env,
				result_folder / "trajectory_geom.yaml", (result_folder / "trajectory_geom.yaml").with_suffix(".check.txt"))

			# # visualize: reference trajectory from the geometric planner, output of controller tracking the ref traj
			run_visualizer("../deps/dynoplan/dynobench/envs/quad3d_payload/benchmark_envs/" + task.env ,result_folder / "init_guess.yaml",  result_folder / "trajectory_geom.yaml", result_folder / "trajectory_geom.html")

		if task.alg == "extraiter":

			result_folder = results_path / task.instance / "opt" / "{:03d}".format(task.trial)
			previous_result = result_folder / "output.trajopt.yaml"
			for i in range(1, 20):
				result_folder_i = result_folder / "iter{:02d}".format(i)
				result_folder_i.mkdir(parents=True, exist_ok=False)
				run_opt(previous_result, str(result_folder / "env_inflated.yaml"), str(result_folder_i), task.timelimit_opt, t_weight="3.0", t_ref="0.8")
				previous_result = result_folder_i / "output.trajopt.yaml"


	except:
		traceback.print_exc()

def main():
	parallel = True
	instances = [
		{ "name": "empty_2robots", "models_path": "point_2.yaml"},
		{ "name": "empty_3robots", "models_path": "point_3.yaml"},
		{ "name": "empty_4robots", "models_path": "point_4.yaml"},
		{ "name": "empty_5robots", "models_path": "point_5.yaml"},
		{ "name": "empty_6robots", "models_path": "point_6.yaml"},

		{ "name": "forest_2robots", "models_path": "point_2.yaml"},
		{ "name": "forest_3robots", "models_path": "point_3.yaml"},
		{ "name": "forest_4robots", "models_path": "point_4.yaml"},
		{ "name": "forest_5robots", "models_path": "point_5.yaml"},
		{ "name": "forest_6robots", "models_path": "point_6.yaml"},

		{ "name": "maze_2robots", "models_path": "point_2.yaml"},
		{ "name": "maze_3robots", "models_path": "point_3.yaml"},
		{ "name": "maze_4robots", "models_path": "point_4.yaml"},
		{ "name": "maze_5robots", "models_path": "point_5.yaml"},
		{ "name": "maze_6robots", "models_path": "point_6.yaml"},

		{ "name": "window_2robots", "models_path": "point_2.yaml"},
		{ "name": "window_3robots", "models_path": "point_3.yaml"},
		{ "name": "window_4robots", "models_path": "point_4.yaml"},
		{ "name": "window_5robots", "models_path": "point_5.yaml"},
		{ "name": "window_6robots", "models_path": "point_6.yaml"},

		{ "name": "empty_5robots_uniform", "models_path": "point_5.yaml"},
		{ "name": "forest_4robots_uniform", "models_path": "point_4.yaml"},
		{ "name": "window_3robots_uniform", "models_path": "point_3.yaml"},
		{ "name": "takeoff_2robots", "models_path": "point_2.yaml"},

		{ "name": "window_2robots_exp", "models_path": "point_2_exp.yaml"},
		{ "name": "window_3robots_exp", "models_path": "point_3_exp.yaml"},
		{ "name": "forest_2robots_exp", "models_path": "point_2_exp.yaml"},
		{ "name": "forest_3robots_exp", "models_path": "point_3_exp.yaml"},


	]
	algs = [
		"payload",
		"geom",
		"opt",
		# "extraiter",
	]
	# trials = 3
	trials = [0]#, 1, 2, 3, 4, 5, 6, 7, 8, 9]
	timelimit_geom = 300
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
	trials_ = ["00"+str(i) for i in trials]
	compute_errors([instance["name"] for instance in instances], algs, trials_)

	paper_tables.write_table1(Path("../results"), trials_)
	paper_tables.write_plot1(Path("../results"), trials_, timelimit_geom)
	paper_tables.runtime_results(Path("../results"), trials_)
if __name__ == '__main__':
	main()
