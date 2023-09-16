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
						info = yaml.load(file, Loader=yaml.CSafeLoader)
					ep_norm = np.linalg.norm(info["result"]["error_pos"], axis=1)
					ep_trials.extend(ep_norm.tolist())

					normalized_force = np.sum(info["result"]["actions_act"], axis=1)
					force = normalized_force * 34 / 4 # in grams
					# see https://wiki.bitcraze.io/misc:investigations:thrust
					# and https://github.com/IMRCLab/crazyflie-system-id
					power = force / 4 # watts
					energy = np.sum(power.tolist())*0.01/60/60 # Wh
					power_trials.append(energy)
				else: 
					ep_trials.append(np.nan)
					power_trials.append(np.nan)

			ep_trials = np.array(ep_trials)
			power_trials = np.array(power_trials)

			ep_mean = np.nanmean(ep_trials, axis=0)
			ep_std = np.nanstd(ep_trials, axis=0)
			energy_mean = np.nanmean(power_trials, axis=0)
			energy_std = np.nanstd(power_trials, axis=0)
			success = np.count_nonzero(~np.isnan(power_trials)) / len(trials) * 100
			success_count = np.count_nonzero(~np.isnan(power_trials))


			results[instance][alg] = {
				"ep_mean": ep_mean if not np.isnan(ep_mean) else None,
				"ep_std": ep_std if not np.isnan(ep_std) else None,
				"energy_mean": energy_mean if not np.isnan(energy_mean) else None,
				"energy_std": energy_std if not np.isnan(energy_std) else None,
				"success": success,
				"success_count": success_count,
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
		out += r"{\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	if is_best:
		out += r"}"
	return out

def print_and_highlight_best_max(key, result, alg, algs, digits=1):
	out = ""
	is_best = False
	if result[alg][key] is not None and result[alg][key] != "*":
		# we only look at one digit
		is_best = np.array([round(result[alg][key],digits) >= round(result[other][key],digits) for other in algs if result[other][key] is not None and result[other][key] != "*"]).all()
	if is_best:
		out += r"{\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	if is_best:
		out += r"}"
	return out


def write_table1(result_path, trials):
	algs = [
		"payload",
		"geom",
		"opt",
	]
	robots = [2,3,4,5,6]
	envs = ["empty", "forest", "window"]

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
		f.write(r"\begin{tabular}{l|l||l|l|l|l|l|l}" + "\n")
		f.write("Environment & Metrics &"+ r" & ".join([str(n)  + " robots" for n in robots]) + r" & success [\%]\\" + "\n")

		for env in envs:
			f.write(r"\hline" + "\n")
			f.write(r"\multirow{6}{*}{" + env + r"}")

			# tracking error
			for k, alg in enumerate(algs):
				out = " & Error " + alg + " [m] "
				success_count = 0
				for n in robots:
					inst = "{}_{}robots".format(env, n)
					out += " & "
					out += print_and_highlight_best_min("ep_mean", r[inst], alg, algs, digits=2)
					if r[inst][alg]["ep_std"] is not None:
						out += " {{\\color{{gray}}\\tiny {:.2f} }} ".format(r[inst][alg]["ep_std"])
						out += " {\\tiny " + print_and_highlight_best_max("success", r[inst], alg, algs, digits=0) + " \\% } "

					success_count += r[inst][alg]["success_count"]
				out += "& {:.0f} ".format(success_count / len(robots) / len(trials) * 100)
				out += r"\\"
				f.write(out)

			f.write(r"\cline{2-7}" + "\n")

			# energy
			for alg in algs:
				out = " & Energy " + alg + " [Wh] "
				for n in robots:
					inst = "{}_{}robots".format(env, n)
					out += " & " + print_and_highlight_best_min("energy_mean", r[inst], alg, algs, digits=2)
					if r[inst][alg]["energy_std"] is not None:
						out += " {{\\color{{gray}}\\tiny {:.2f} }} ".format(r[inst][alg]["energy_std"])
				out += r"\\"
				f.write(out)


		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	gen_pdf(output_path)

def write_plot1(result_path, trials, T):
	envs = ['empty_5robots', 'empty_5robots_uniform',
		 'forest_4robots', 'forest_4robots_uniform',
		 'window_3robots', 'window_3robots_uniform']
	labels = [ 'empty (N=5), our sampler', 'empty (N=5), uniform',
		   'forest (N=4), our sampler', 'forest (N=4), uniform', 
		   'window (N=3), our sampler', 'window (N=3), uniform']
	meancolors = ['r', 'r', 'g', 'g', 'b', 'b']
	stdcolors = ['r', 'r', 'g', 'g', 'b', 'b']
	linestyles = ['solid', 'dashed', 'solid', 'dashed', 'solid', 'dashed']
	# with PdfPages(result_path / 'plot1.pdf') as pdf:
	fig, ax = plt.subplots(figsize=(6, 4))
	ax.grid('True', which='both', axis='x', linestyle='dashed')
	ax.grid(which='major', axis='y', linestyle='dashed')
	for env, label, color, stdcolor, linestyle in zip(envs, labels, meancolors, stdcolors, linestyles):
		dt = 0.1
		costs = []

		for trial in trials:
			filepath = result_path / env / "geom" / trial / "stats.yaml"
			with open(filepath, "r") as file:
				stats = yaml.load(file, Loader=yaml.CSafeLoader)

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

		ax.plot(times, mean, label=label, color=color, linestyle=linestyle, lw=2.5)
		ax.set_xscale('log')
		ax.fill_between(times, mean+std, mean-std, color=stdcolor, alpha=0.1, linestyle=linestyle,lw=2.0)
		ax.legend()
		ax.set_xlabel("Time [s]")
		ax.set_ylabel("Cost [s]")

	# plt.savefig(result_path / 'plot1.pdf')
	fig.savefig(result_path / 'plot1.pdf')
	# plt.show()
	# pdf.savefig(fig)
	subprocess.call(['pdfcrop', result_path / 'plot1.pdf', result_path / 'plot1.pdf'])
	plt.close()


def create_whisker_plot(data, pdfname="result.pdf", title='Whisker Plot', ylabel='Values', xlabel=[2]):
	mv = []
	for d in data:
		mean = np.mean(d)
		std = np.std(d)
		mv.append((mean, std))
	print(pdfname, '\n', mv)
	plt.figure(figsize=(3, 2))
	plt.boxplot(data, labels=xlabel, patch_artist=True, boxprops=dict(facecolor='lightblue'), showfliers=True)
	plt.title(title)
	plt.xlabel("Number of robots")
	plt.ylabel(ylabel)
	# plt.yscale("log")
	plt.grid()
	plt.savefig(pdfname, bbox_inches = 'tight')  # Save
	subprocess.call(['pdfcrop', pdfname, pdfname])



def runtime_results(result_path, trials):
	robots = [2, 3, 4, 5, 6]
	envs = ["forest"]

	instances = ["{}_{}robots".format(env, n) for env in envs for n in robots]
	runtimes  = dict()
	data = []
	for instance in instances:
		trial_runtime = []
		for trial in trials:
			trajopt_path = result_path / instance / "opt" / trial / "output"
			if trajopt_path.exists():	
				with open(trajopt_path, "r") as f:
					trajopt = yaml.load(f, Loader=yaml.CSafeLoader)

				runtime = trajopt["info"]["time_ddp_total"] * 1e-3
				print(trajopt_path, runtime)
			else: 
				runtime = np.nan
			trial_runtime.append(runtime)
		data.append(trial_runtime)
	print(data)
	create_whisker_plot(data, pdfname=result_path /"whiskers_forest.pdf", title="", ylabel="Runtime [s]", xlabel=robots)


def plot_iter_vs_error(result_path, trials):
	robots = [4]
	envs = ["forest"]

	instances = ["{}_{}robots".format(env, n) for env in envs for n in robots]
	x = list(range(0,20))
	y1 = []
	y2 = []
	for instance in instances: 
		for trial in trials:
			y3 = []
			for iter in x:
				power_trials = []
				if iter == 0:
					trajopt_path = result_path / instance / "opt" / trial  /  Path("output.trajopt.yaml")
				else: 
					if iter < 10:
						iterstr = "iter0"
					else: 
						iterstr = "iter"
					trajopt_path = result_path / instance / "opt" / trial / Path(iterstr+"{}".format(str(iter)) + "/output.trajopt.yaml")
				if trajopt_path.exists():
					print(trajopt_path)
					with open(trajopt_path, "r") as f:
						trajopt = yaml.load(f, Loader=yaml.CSafeLoader)
					y1.append(trajopt["num_states"]*0.01)
					normalized_force = np.sum(trajopt["actions"], axis=1)
					# print(normalized_force)
					force = normalized_force * 34 / 4 # in grams
					# see https://wiki.bitcraze.io/misc:investigations:thrust
					# and https://github.com/IMRCLab/crazyflie-system-id
					power = force / 4 # watts
					power_trials.extend(power.tolist())
					power_trials = np.array(power_trials)
					energy = np.sum(power_trials)*0.01/60/60 # Wh
					y3.append(energy)
			y2.append(y3)

	fig, ax = plt.subplots(figsize=(2.5,2))
	# Plot on the second subplot (right)
	# print(x)
	y2 = np.array(y2)
	mean = np.mean(y2, axis=0)
	std = np.std(y2, axis=0)

	ax.plot(x, mean,color='k', label='forest 4 robots')
	ax.fill_between(x, mean+std, mean-std, alpha=0.1)

	# ax.set_xticks(x)
	# ax.legend()
	ax.set_xlabel("Iterations")
	ax.set_ylabel('Energy [Wh]')
	ax.grid()
	# Adjust the layout to prevent overlapping labels
	plt.tight_layout()

	# Save the figure as a PDF
	filename = result_path / Path('time_energy_plots.pdf')
	plt.savefig(filename)
	subprocess.call(['pdfcrop', filename, filename])


if __name__ == '__main__':
	trials = ["000", "001", "002", "003", "004", "005", "006", "007", "008", "009"]
	T = 300

	# write_table1(Path("../results"), trials)
	
	 # change settings to match latex
	plt.rcParams.update({
		"text.usetex": True,
		"font.family": "sans-serif",
		"font.sans-serif": "Helvetica",
		"font.size": 12,
		"figure.figsize": (6, 4),
	})
	
	write_plot1(Path("../results"), trials, T)
	runtime_results(Path("../results"), trials)
	plot_iter_vs_error(Path("../results"), trials)




