import os
import matplotlib
from matplotlib import pyplot as plt
import numpy as np

def is_float(string):
    # string.isdecimal()
    try:
        float_value = float(string)
        return True
    except ValueError:  # String is not a number
        return False



docs_path = "/home/fjod/repos/inspectrone/docs/"
path = "/home/fjod/repos/inspectrone/catkin_ws/downloads/retrained_models/"
log_dirs = [["with_cropping_0.025_hit_ratio",
            "with_cropping_0.075_hit_ratio"],
            ["with_cross_matching_0.025_hit_ratio",
            "with_cross_matching_0.075_hit_ratio"]]
log_names = ["cropping.txt", "crossmatch.txt"]
titles = ["Using Cropping", "Using Cross-matching"]
metric_names = ["loss", "RTE", "RRE", "hit_ratio", "match_ratio"]


# Remeber to clean the file so only one log is present in the log files

for i, log_dir_group in enumerate(log_dirs):
    hit_ratio_group = [None]*len(log_dir_group)
    metrics_group = [None]*len(log_dir_group)
    for j, log_dir in enumerate(log_dir_group):
        
        print(path+log_dir)
    
        f = open(os.path.join(path+log_dir, log_names[i]), "r")
        content = f.read()
        f.close()

        matched_lines = [line for line in content.split('\n') if "Final Loss" in line]

        hit_ratio_line = [line for line in content.split('\n') if "hit_ratio_thresh" in line]
        matched_lines = [line for line in content.split('\n') if "Final Loss" in line]
        final_lines = [line[line.find("Final Loss"):] for line in matched_lines]
        
        metrics_group[j] = [[float(word.replace(",","")) for word in final_line.split() if is_float(word.replace(",","")) ] for final_line in final_lines]
        
        # for word in final_lines[0].split():
            # word = word.replace(',', '')
            # print(word)
            # print("0.05".isdecimal())
        # [int(s) for s in txt.split() if s.isdigit()]

        idx = hit_ratio_line[0].find("hit_ratio_thresh")+len("hit_ratio_thresh")

        hit_ratio_group[j] = hit_ratio_line[0][idx:].replace(":","").strip()
        # print("hit_ratio_thresh", hit_ratio_group[j])
        print(hit_ratio_line[0], idx)
    
    # print(metrics_group[0])
    # print(metrics_group[1])
    # TODO only pick the last
    for k, metric_name in enumerate(metric_names):

        # loss, RTE, RRE, HIT, MatchRatio
        
        plt.rcParams.update({'font.size': 18,
                            'axes.titlesize': 22})
        fig, ax = plt.subplots() # ?

        max_met = 0
        for j, log_dir in enumerate(log_dir_group):
            metric = np.array(metrics_group[j])[:,k]
            if max(metric) > max_met:
                max_met = max(metric)
            
            plt.plot(metric, linewidth=4)
        
        plt.legend(["hr="+hit_ratio for hit_ratio in hit_ratio_group])
        plt.ylabel(metric_name)
        plt.yticks(np.arange(0, max_met+0.01, step=max_met/8))
        plt.xlabel("epoch")
        plt.xticks(np.arange(0, len(metrics_group[j]), step=10))
        plt.title(titles[i])
        plt.grid()
        plt.tight_layout()
        # plt.show()
        string_prefix = "[{}]".format(log_names[i])
        string_suffix = "[{}]".format("_".join(hit_ratio_group))
        file_name = "{}_{}_{}.eps".format(string_prefix, metric_name, string_suffix)
        print("saving file in", docs_path+file_name)
        fig.savefig(docs_path+file_name, format='eps')

        #print(final_lines)