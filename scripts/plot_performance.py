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




log_dir = "/home/fjod/repos/inspectrone/docs/performance"
log_names = ["0.025_timings.txt", "0.040_timings.txt"]
legends = ["0.025m voxels", "0.040m voxels"]

# Remeber to clean the file so only one log is present in the log files
all_lists = []
for i, log_name in enumerate(log_names):
    f = open(os.path.join(log_dir, log_name), "r")
    content = f.read()
    f.close()
    process_times = [float(time.split(':')[1]) for time in [line for line in content.split('\n') if "5 process" in line]]
    transform_times = [float(time.split(':')[1]) for time in [line for line in content.split('\n') if "6 find" in line]]
    pub_pcd_times = [float(time.split(':')[1]) for time in [line for line in content.split('\n') if "7 publish" in line]]
    pub_pose_times = [float(time.split(':')[1]) for time in [line for line in content.split('\n') if "8 publish" in line]]
    fitness_times = [float(time.split(':')[1]) for time in [line for line in content.split('\n') if "0 fit" in line]]
    pcd_id = [int(float(time.split(':')[1])) for time in [line for line in content.split('\n') if "0 pcd" in line]]

    # print(pcd_id)
    
    lists = [process_times, transform_times, pub_pcd_times, pub_pose_times, fitness_times]
    all_lists.append(lists)

# print(all_lists)
all_lists = (list(zip(*all_lists)))
all_labels = ["processing time", "transform time", "publish point cloud time", "publish pose time", "fitness"]
all_lists
for k, sets in enumerate(all_lists):
    plt.rcParams.update({'font.size': 18,
                        'axes.titlesize': 22})
    fig, ax = plt.subplots() # ?
    # print(data)
    # for data in sets :
        # print(data_1)
    plt.plot(list(zip(*sets)), linewidth=4)
    # plt.legend(["hr="+hit_ratio for hit_ratio in hit_ratio_group])
    plt.ylabel(all_labels[k])
    # plt.yticks(np.arange(0, max_met+0.01, step=max_met/8))
    plt.xlabel("pcd id")
    # plt.xticks(np.arange(0, len(metrics_group[j]), step=10))
    # plt.title(titles[i])
    plt.grid()
    
    plt.legend(legends)
    
    plt.tight_layout()
    plt.show()
    
    # string_prefix = "[{}]".format(log_names[i])
    # string_suffix = "[{}]".format("_".join(hit_ratio_group))
    # file_name = "{}_{}_{}.eps".format(string_prefix, metric_name, string_suffix)
    # print("saving file in", docs_path+file_name)
    # fig.savefig(docs_path+file_name, format='eps')

    # for k, metric_name in enumerate(metric_names):
    #     plt.rcParams.update({'font.size': 18,
    #                         'axes.titlesize': 22})
    #     fig, ax = plt.subplots() # ?

    #     max_met = 0
    #     for j, log_dir in enumerate(log_dir_group):
    #         metric = np.array(metrics_group[j])[:,k]
    #         if max(metric) > max_met:
    #             max_met = max(metric)
            
    #         plt.plot(metric, linewidth=4)

    #     plt.ylabel(metric_name)
    #     plt.yticks(np.arange(0, max_met+0.01, step=max_met/8))
    #     plt.xlabel("epoch")
    #     plt.xticks(np.arange(0, len(metrics_group[j]), step=10))
    #     plt.title(titles[i])
    #     plt.grid()
    #     plt.tight_layout()
    #     # plt.show()
    #     string_prefix = "[{}]".format(log_names[i])
    #     string_suffix = "[{}]".format("_".join(hit_ratio_group))
    #     file_name = "{}_{}_{}.eps".format(string_prefix, metric_name, string_suffix)
    #     print("saving file in", docs_path+file_name)
    #     fig.savefig(docs_path+file_name, format='eps')

#print(final_lines)
