import yaml
from pathlib import Path
import shutil
import subprocess
from dataclasses import dataclass
import multiprocessing as mp
import tqdm
import psutil


@dataclass
class ExecutionTask:
	"""Class for keeping track of an item in inventory."""
	# env: Path
	# cfg: Path
	# result_folder: Path
	instance: str
	alg: str
	trial: int
	timelimit: float

def run_geom(filename_env, folder, timelimit):
	folder = Path(folder)
	try:
		with open(folder / "log.txt", 'w') as f:
			out = subprocess.run(["./nUavsPayloadPlanner",
						"--input", filename_env,
						"--output", folder / "output.yaml",
						"--stats" , folder / "stats.yaml"],
						stdout=f, stderr=f, timeout=timelimit)
	except Exception as e:
		print(e)


def execute_task(task: ExecutionTask):
	results_path = Path("../results")
	# tuning_path = Path("../tuning")
	env_path = Path().resolve() / "../examples"
	env = (env_path / task.instance / task.instance).with_suffix(".yaml")
	assert(env.is_file())

	result_folder = results_path / task.instance / task.alg / "{:03d}".format(task.trial)
	if result_folder.exists():
			print("Warning! {} exists already. Deleting...".format(result_folder))
			shutil.rmtree(result_folder)
	result_folder.mkdir(parents=True, exist_ok=False)

	if task.alg == "geom":
		run_geom(str(env), str(result_folder), task.timelimit)

def main():
	parallel = True
	instances = [
		"2cfs_pointmass",
	]
	algs = [
		"geom",
	]
	trials = 1
	timelimit = 5*60

	tasks = []
	for instance in instances:
		for alg in algs:
			for trial in range(trials):
				tasks.append(ExecutionTask(instance, alg, trial, timelimit))

	if parallel and len(tasks) > 1:
		use_cpus = psutil.cpu_count(logical=False)-1
		print("Using {} CPUs".format(use_cpus))
		with mp.Pool(use_cpus) as p:
			for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
				pass
	else:
		for task in tasks:
			execute_task(task)

if __name__ == '__main__':
	main()
