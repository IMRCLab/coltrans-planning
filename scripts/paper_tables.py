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
				"ep_mean": ep_mean if not np.isnan(ep_mean) else None,
				"ep_std": ep_std if not np.isnan(ep_std) else None,
				"energy": energy,
			}
	return results

def gen_pdf(output_path):
	# run pdflatex
	subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
	# delete temp files
	output_path.with_suffix(".aux").unlink()
	output_path.with_suffix(".log").unlink()

def print_and_highlight_best_min(key, result, alg, algs, digits=1):
	out = ""
	is_best = False
	if result[alg][key] is not None and result[alg][key] != "*":
		# we only look at one digit
		is_best = np.array([round(result[alg][key],1) <= round(result[other][key],1) for other in algs if result[other][key] is not None and result[other][key] != "*"]).all()
	if is_best:
		out += r"\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	return out

def print_and_highlight_best_max(key, result, alg, algs, digits=1):
	out = ""
	is_best = False
	if result[alg][key] is not None and result[alg][key] != "*":
		# we only look at one digit
		is_best = np.array([round(result[alg][key],1) >= round(result[other][key],1) for other in algs if result[other][key] is not None and result[other][key] != "*"]).all()
	if is_best:
		out += r"\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	return out


def write_table1(result_path, trials):
	algs = [
		"payload",
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
		f.write(r"\usepackage{multirow}")
		f.write("\n")
		f.write(r"\begin{document}")
		f.write("\n")
		f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")
		f.write(r"\begin{tabular}{l|l||l|l|l|l|l}" + "\n")
		f.write("Environment & Metrics &"+ r" & ".join([str(n)  + " robots" for n in robots]) + r" \\" + "\n")

		for env in envs:
			f.write(r"\hline" + "\n")
			f.write(r"\multirow{6}{*}{" + env + r"}")

			# tracking error
			for k, alg in enumerate(algs):
				out = " & Error " + alg + " [m] "
				for n in robots:
					inst = "{}_{}robots".format(env, n)
					out += " & " + print_and_highlight_best_min("ep_mean", r[inst], alg, algs, digits=2)
					if r[inst][alg]["ep_std"] is not None:
						out += " {{\\color{{gray}}\\tiny {:.2f} }} ".format(r[inst][alg]["ep_std"])
				out += r"\\"
				f.write(out)

			f.write(r"\cline{2-7}" + "\n")

			# energy
			for alg in algs:
				out = " & Energy " + alg + " [Wh] "
				for n in robots:
					inst = "{}_{}robots".format(env, n)
					out += " & " + print_and_highlight_best_min("energy", r[inst], alg, algs, digits=2)
				out += r"\\"
				f.write(out)


		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	gen_pdf(output_path)


if __name__ == '__main__':
	trials = ["000"]
	r = write_table1(Path("../results"), trials)



