import argparse
from mimetypes import init
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.backends.backend_pdf import PdfPages
# from matplotlib.backends.backend_pgf import PdfPages
from matplotlib.cm import get_cmap
from collections import defaultdict

import subprocess

class Report:
    def __init__(self, filename, result_path):
        self.result_path = result_path
        self.filename = filename
        self.tableName = self.filename.replace('.pdf', '.tex')

    def write_info(self, instance, alg, trial):
        result_folder = self.result_path / instance / alg / Path(trial)
        if result_folder.exists():
            if alg == "geom":
                ref_traj_path = result_folder / "init_guess.yaml"
                act_traj_path = result_folder/ "trajectory_geom.yaml"
            elif alg == "opt":
                ref_traj_path = result_folder / "output.trajopt.yaml"
                act_traj_path = result_folder/ "trajectory_opt.yaml"
            
            if ref_traj_path.exists():
                with open(ref_traj_path, "r") as ref_file:
                    _ref = yaml.safe_load(ref_file)
                if "states" in _ref:
                    ref_states = np.array(_ref["states"])
                    ref_actions = np.array(_ref["actions"])
                elif "result" in _ref:
                    ref_states = np.array(_ref["result"]["states"])
                    ref_actions = np.array(_ref["result"]["actions"])

                with open(act_traj_path, "r") as act_file:
                    _act = yaml.safe_load(act_file)
                if "states" in _act:
                    act_states = np.array(_act["states"])
                    act_actions = np.array(_act["actions"])
                elif "result" in _act:
                    act_states =  np.array(_act["result"]["states"])
                    act_actions = np.array(_act["result"]["actions"])

                refpos = ref_states[:, 0:3]
                actpos = act_states[:, 0:3]
                
                error_p = (refpos - actpos).tolist()
                info = dict()
                info["result"] = {}

                info["result"]["error_pos"] = error_p
                info["result"]["actions_ref"] = ref_actions.tolist()
                info["result"]["actions_act"] = act_actions.tolist()
                info["path"] = str(result_folder)
                with open(result_folder / "info.yaml", "w") as err_file:
                    yaml.safe_dump(info, err_file, default_flow_style=None)
            else: 
                return None
        else: 
            return None
        return info
    

    def genandwriteTable(self, instances, algs, trials):

        with open(self.result_path / self.tableName, "w") as f:
            f.write(r"\documentclass{standalone}")
            f.write("\n")
            f.write(r"\usepackage{xcolor}")
            f.write("\n")
            f.write(r"\begin{document}")
            f.write("\n")

            out = r"\begin{tabular}{l||" # instances
            for _ in algs:
                out += r"cc|" # algorithms
            out += "}\n"
            f.write(out)

            out = r" "
            for alg in algs:
                out += r"&{} tracking &{} energy".format(alg, alg)
            out += r"\\ \hline"
            f.write(out)
            f.write("\n")
            out = r" "
            for instance in instances:    
                if "_" in instance: 
                    instance_ = instance.replace("_", " ")
                else: 
                    instance_ = instance
                out += r" {}".format(instance_)

                for alg in algs:
                    ep_trials = []
                    energy_trials = []
                    for trial in trials:
                        filepath = self.result_path / instance / alg / trial / "info.yaml"
                        if filepath.exists():
                            with open(self.result_path / instance / alg / trial/ "info.yaml", "r") as file:
                                info = yaml.safe_load(file)
                            ep_norm = np.linalg.norm(info["result"]["error_pos"], axis=1)
                            ep_trials.extend(ep_norm.tolist())

                            energy = np.sum(info["result"]["actions_act"], axis=1)
                            energy_trials.extend(energy.tolist())
                        else: 
                            ep_trials = [np.nan for i in range(1000)]
                    ep_trials = np.array(ep_trials)
                    energy_trials = np.array(energy_trials)

                    ep_mean = np.nanmean(ep_trials, axis=0)
                    ep_std = np.nanstd(ep_trials, axis=0)

                    out += r"&{:.2f} \color{{gray}} \tiny {:.2f} & {:.2f}".format(ep_mean, ep_std, np.sum(energy_trials))     
            
                out += r"\\" + "\n"
                out += r"\hline"
            out += r"\end{tabular}" + "\n"
            out += r"\end{document}" 
            f.write(out)
        subprocess.run(["pdflatex", self.tableName], check=True, cwd=self.result_path)

    def plot_info(self, info):
        pass

