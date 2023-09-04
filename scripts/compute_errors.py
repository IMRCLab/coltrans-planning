import argparse
import yaml
from pathlib import Path
import numpy as np
from report_info import Report


def compute_errors(instances, algs, trials):
    results_path = Path("../results")
    report = Report("result.pdf", results_path)
    # print(instances, algs, trials)
    for instance in instances:
        print("instance: ", instance)
        for alg in algs:
            print("alg: ", alg)
            for trial in trials:
                info = report.write_info(instance, alg, trial)
                if info != None: 
                    report.plot_info(info)
    report.genandwriteTable(instances, algs, trials)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('instances', help="instances")
    parser.add_argument('algs', help="algorithms")
    parser.add_argument('trials', help="trials")
    args = parser.parse_args()
    
    args.algs = args.algs.split(',')
    args.trials = args.trials.split(',')
    args.instances = args.instances.split(',')
    compute_errors(args.instances, args.algs, args.trials)


if __name__ == "__main__":

    main()