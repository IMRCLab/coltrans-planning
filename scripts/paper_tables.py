from pathlib import Path
import numpy as np
import yaml
import subprocess

def compute_result(result_path, instances, algs, trials):

	results = dict()
	for instance in instances:
		results[instance] = dict()
		for alg in algs:
			ep_trials = []
			power_trials = []
			for trial in trials:
				filepath = result_path / instance / alg / trial / "info.yaml"
				if filepath.exists():
					with open(result_path / instance / alg / trial/ "info.yaml", "r") as file:
						info = yaml.safe_load(file)
					ep_norm = np.linalg.norm(info["result"]["error_pos"], axis=1)
					ep_trials.extend(ep_norm.tolist())

					normalized_force = np.sum(info["result"]["actions_act"], axis=1)
					force = normalized_force * 34 / 4 # in grams
					# see https://wiki.bitcraze.io/misc:investigations:thrust
					# and https://github.com/IMRCLab/crazyflie-system-id
					power = force / 4 # watts
					power_trials.extend(power.tolist())
				else: 
					ep_trials = [np.nan for i in range(1000)]
			ep_trials = np.array(ep_trials)
			power_trials = np.array(power_trials)

			ep_mean = np.nanmean(ep_trials, axis=0)
			ep_std = np.nanstd(ep_trials, axis=0)

			energy = np.sum(power_trials)*0.01/60/60 # Wh

			results[instance][alg] = {
				"ep_mean": ep_mean,
				"ep_std": ep_std,
				"energy": energy,
			}
	return results

def gen_pdf(output_path):
	# run pdflatex
	subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
	# delete temp files
	output_path.with_suffix(".aux").unlink()
	output_path.with_suffix(".log").unlink()

def write_table1(result_path, trials):
	algs = [
		"geom",
		"opt",
	]
	robots = [2,3,4,5,6]
	envs = ["empty", "forest", "maze"]

	instances = ["{}_{}robots".format(env, n) for env in envs for n in robots]

	r = compute_result(result_path, instances, algs, trials)

	output_path = Path("../results/paper_table1.pdf")
	with open(output_path.with_suffix(".tex"), "w") as f:

		f.write(r"\documentclass{standalone}")
		f.write("\n")
		f.write(r"\usepackage{xcolor}")
		f.write("\n")
		f.write(r"\begin{document}")
		f.write("\n")
		f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")
		f.write(r"\begin{tabular}{l||l||l||l||l||l||l||l||c}" + "\n")
		f.write("Environment & Metrics &"+ r" & ".join([str(n)  + " robots" for n in robots]) + r" & \\" + "\n")

		for env in envs:
			f.write(r"\hline" + "\n")
			f.write(env + " & Error Geom [m] & "+ " & ".join(["{:.2f} {{\\color{{gray}}\\tiny {:.2f} }}".format(r["{}_{}robots".format(env, n)]["geom"]["ep_mean"], r["{}_{}robots".format(env, n)]["geom"]["ep_std"]) 
					if r["{}_{}robots".format(env, n)]["geom"].get('ep_mean') else "\\hspace{0.4cm}- " for n in robots])  + r" & \\")
			
			f.write(env + " & Error Opt [m] & "+ " & ".join(["{:.2f} {{\\color{{gray}}\\tiny {:.2f} }}".format(r["{}_{}robots".format(env, n)]["opt"]["ep_mean"], r["{}_{}robots".format(env, n)]["opt"]["ep_std"]) 
					if r["{}_{}robots".format(env, n)]["opt"].get('ep_mean') else "\\hspace{0.4cm}- " for n in robots])  + r" & \\")
			
			f.write(env + " & Energy Geom [Wh] & "+ " & ".join(["{:.2f}".format(r["{}_{}robots".format(env, n)]["geom"]["energy"]) 
					if r["{}_{}robots".format(env, n)]["geom"].get('energy') else "\\hspace{0.4cm}- " for n in robots])  + r" & \\")
			
			f.write(env + " & Energy Opt [Wh] & "+ " & ".join(["{:.2f}".format(r["{}_{}robots".format(env, n)]["opt"]["energy"]) 
					if r["{}_{}robots".format(env, n)]["opt"].get('energy') else "\\hspace{0.4cm}- " for n in robots])  + r" & \\")

		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	gen_pdf(output_path)


if __name__ == '__main__':
	trials = ["000"]
	r = write_table1(Path("../results"), trials)



