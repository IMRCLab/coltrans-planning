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
            if alg == "geom" or alg == "payload":
                ref_traj_path = result_folder / "init_guess.yaml"
                act_traj_path = result_folder/ "trajectory_geom.yaml"
                check_traj_path = result_folder/ "trajectory_geom.check.txt"
            elif alg == "opt":
                ref_traj_path = result_folder / "output.trajopt.yaml"
                act_traj_path = result_folder/ "trajectory_opt.yaml"
                check_traj_path = result_folder/ "trajectory_opt.check.txt"

            solution_valid = False
            if check_traj_path.exists():
                with open(check_traj_path, "r") as f:
                    solution_valid = (f.readlines()[-1][0:2] == "OK")
            
            if ref_traj_path.exists() and act_traj_path.exists() and solution_valid:
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

            f.write(r"\begin{tabular}{l||cc|cc}")
            f.write("\n")
            f.write(r"& \multicolumn{2}{c|}{Tracking Error [m]} & \multicolumn{2}{c}{Energy [Wh]}\\")
            f.write("\n")

            out = r" "
            for _ in range(2):
                for alg in algs:
                    out += r"&{}".format(alg)
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

                results = dict()
                for alg in algs:
                    ep_trials = []
                    power_trials = []
                    for trial in trials:
                        filepath = self.result_path / instance / alg / trial / "info.yaml"
                        if filepath.exists():
                            with open(self.result_path / instance / alg / trial/ "info.yaml", "r") as file:
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

                    results[alg] = {
                        "ep_mean": ep_mean,
                        "ep_std": ep_std,
                        "energy": energy,
                    }

                for alg in algs:
                    out += r"&{:.2f} \color{{gray}} \tiny {:.2f} ".format(results[alg]["ep_mean"], results[alg]["ep_std"])
                for alg in algs:
                    out += r"& {:.2f}".format(results[alg]["energy"])
            
                out += r"\\" + "\n"
                out += r"\hline"
            out += r"\end{tabular}" + "\n"
            out += r"\end{document}" 
            f.write(out)
        subprocess.run(["pdflatex", self.tableName], check=True, cwd=self.result_path)

    def plot_info(self, info):
        pass

