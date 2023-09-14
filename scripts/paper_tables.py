from pathlib import Path
import numpy as np
import yaml
import subprocess

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)
from matplotlib.backends.backend_pdf import PdfPages

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
				"energy": energy if not np.isnan(ep_mean) else None,
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
		is_best = np.array([round(result[alg][key],digits) <= round(result[other][key],digits) for other in algs if result[other][key] is not None and result[other][key] != "*"]).all()
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
		is_best = np.array([round(result[alg][key],digits) >= round(result[other][key],digits) for other in algs if result[other][key] is not None and result[other][key] != "*"]).all()
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

def write_plot1(result_path, trials, T):
	envs = ['forest_3robots', 'forest_3robots_uniform']
	labels = [ 'forest:3 robots (new sampler)', 'forest:3 robots (uniform)']
	meancolors = ['r', 'g']
	stdcolors = ['r','g']
	with PdfPages(result_path / 'plot1.pdf') as pdf:
		fig, ax = plt.subplots()
		ax.grid('True', which='both', axis='x', linestyle='dashed')
		ax.grid(which='major', axis='y', linestyle='dashed')
		for env, label, color, stdcolor in zip(envs, labels, meancolors, stdcolors):

			dt = 0.1
			costs = []

			for trial in trials:
				filepath = result_path / env / "geom" / trial / "stats.yaml"
				with open(filepath, "r") as file:
					stats = yaml.safe_load(file)

				costsperrun = np.zeros(int(T / dt)) * np.nan
				if stats.get('stats') !=  None:    # ## Maze for 3 robots
					for d in stats['stats']:
						idx =  int(d["t"] / dt)
						costsperrun[idx:] = d["cost"]
					costs.append(costsperrun)
			times = np.arange(0, T, dt)
			
			costs = np.array(costs)
			rs = costs.shape[0]
			cs = costs.shape[1]
			indices = []
			for c in range(cs):
				nanNums = 0
				for r in range(rs):
					if np.isnan(costs[r, c]): 
						nanNums += 1
				if nanNums > 5:
					indices.append(c)

			costs = np.delete(costs, indices, axis=1)
			times = np.delete(times, indices, axis=0)
			
			mean = np.nanmean(costs, axis=0)
			std = np.nanstd(costs, axis=0)

			ax.plot(times, mean, label=label, color=color, lw=2.5)
			ax.set_xscale('log')
			ax.fill_between(times, mean+std, mean-std, color=stdcolor, alpha=0.1)
			ax.legend()
			ax.set_xlabel("Time [s]")
			ax.set_ylabel("Cost [s]")
		# plt.show()
		pdf.savefig(fig)
		plt.close()


if __name__ == '__main__':
	trials = ["000", "001", "002"]
	# r = write_table1(Path("../results"), trials)
	T = 300
	write_plot1(Path("../results"), trials, T)



