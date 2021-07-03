import os
import matplotlib
from matplotlib import pyplot as plt
import numpy as np
import rosbag


figsize = (16, 8)
#pltratio = [8,4]
linewidth=2
bag_dir = "/home/fjod/repos/inspectrone/docs/results/"
log_dir = "/home/fjod/repos/inspectrone/docs/results/"
bag_file = "2021-07-02-15-20-13.bag"
topics = ["/graph", "/tagslam/path/body_rig", "/graph",]

legends = ["FCGF+ROVIO", "tagslam", "FCGF+ROVIO offset"]
xlabels = ["x pos", "y pos", "z pos"]

def get_bag_info(bag_file):
	bag = rosbag.Bag(bag_dir + bag_file)  # load bag
	bag_path_split = bag_file.split("/")  # split
	bag_file = bag_path_split[len(bag_path_split) - 1]  # select file only
	bag_prefix = bag_file.split(".")[0]  # remove extension

	seq_count = bag.get_message_count(topic)  # get entry count

	return bag, bag_prefix, seq_count


def get_bag_data(topic, legend):
	bag, bag_prefix, seq_count = get_bag_info(bag_file)

	pc_bag = bag.read_messages(topics=[topic])
	# print(list(pc_bag)[0])


	# print(pc_bag)
	if legends[0]==legend:
		msg = None
		times = []
		#times2 = []
		msg_temp = None
		found_times = False
		for (topic, msg, t) in pc_bag:
			marker = msg.markers[-1]
			if marker.type == 2:
				times.append(marker.header.stamp.to_sec())
			#print(msg.markers[-1].header.stamp.to_sec())

			'''
			if found_times:
				continue
			if msg_temp is None:
				msg_temp = msg
				continue
			if len(msg.markers) <= len(msg_temp.markers):
				# print(len(msg.markers), len(msg_temp.markers))
				for marker in msg_temp.markers:
					if marker.type == 2:
						times.append(msg_temp.markers[-1].header.stamp.to_sec())
						#times2.append(t)
				found_times = True
			
			msg_temp = msg
			'''
		
		#print(len(times), len(msg.markers))
		#print(times)
		#exit()

		data = []
		data_times = []
		for i, marker in enumerate(msg.markers):
			if marker.type != 2:
				continue

			pos = marker.pose.position
			time = times[i]
			#time = marker.header.stamp.to_sec()
			#data_times.append(time)
			data.append([float(marker.id)/10000.0, pos.x, pos.y, pos.z])
			#print([time, pos.x, pos.y, pos.z, marker.id])
		
		#print(data[-10:])
		data_sorted =  sorted(data,key=lambda x: x[0])
		#print(data_sorted[-10:])	
		#exit()	
		data = list(zip(*data_sorted))
		data = np.array(data)
		#print(data_sorted)
		data[0,:] = data[0,:] - data[0,0]
		#print(data)

		#print(data)
		print(data)
		data = data.reshape(4,-1)
		return data

	elif legends[1]==legend:
		msg = None
		for (topic, msg, t) in pc_bag:
			pass

		data = []
		for i, pose in enumerate(msg.poses):
			pos = pose.pose.position
			time = msg.poses[i].header.stamp.to_sec()
			data.append([time, pos.x, pos.y, pos.z])
			#print([time.to_sec(), pos.x, pos.y, pos.z])
		data = list(zip(*data))
		data = np.array(data)
		data[0,:] = data[0,:] - data[0,0]

		print(data)
		data = data.reshape(4,-1)
		return data

	elif legends[2]==legend:
		msg = None
		times = []
		#times2 = []
		msg_temp = None
		found_times = False
		for (topic, msg, t) in pc_bag:
			marker = msg.markers[-1]
			if marker.type == 2:
				times.append(marker.header.stamp.to_sec())
			#print(msg.markers[-1].header.stamp.to_sec())

			'''
			if found_times:
				continue
			if msg_temp is None:
				msg_temp = msg
				continue
			if len(msg.markers) <= len(msg_temp.markers):
				# print(len(msg.markers), len(msg_temp.markers))
				for marker in msg_temp.markers:
					if marker.type == 2:
						times.append(msg_temp.markers[-1].header.stamp.to_sec())
						#times2.append(t)
				found_times = True
			
			msg_temp = msg
			'''
		
		#print(len(times), len(msg.markers))
		#print(times)
		#exit()

		data = []
		data_times = []
		for i, marker in enumerate(msg.markers):
			if marker.type != 2:
				continue

			pos = marker.pose.position
			time = times[i]
			#time = marker.header.stamp.to_sec()
			#data_times.append(time)
			data.append([float(marker.id)/10000.0, pos.x, pos.y, pos.z])
			#print([time, pos.x, pos.y, pos.z, marker.id])
		
		#print(data[-10:])
		data_sorted =  sorted(data,key=lambda x: x[0])
		#print(data_sorted[-10:])	
		#exit()	
		data = list(zip(*data_sorted))
		data = np.array(data)
		#print(data_sorted)
		data[0,:] = data[0,:] - data[0,0]
		data[3,:] = data[3,:] - 0.1
		#print(data)
		print(data)

		#print(data)
		data = data.reshape(4,-1)
		return data

#print(data)

plt.rcParams.update({'font.size': 18,
                'axes.titlesize': 22})
fig, axs = plt.subplots(3, 1, figsize=figsize)#, gridspec_kw={'width_ratios': pltratio})

#axs[1].set(xlabel='time [s]', ylabel='pos [m]')
#axs[1].legend(legends)
#axs[2].set(xlabel='time [s]', ylabel='pos [m]')
#axs[2].legend(legends)

axs[0].legend(legends[0], bbox_to_anchor=(1.1, 1.05))
for i, topic in enumerate(topics):
	data = get_bag_data(topic, legends[i])

	if "offset" in legends[i]:
		axs[0].plot(data[0], data[1], "--", linewidth=linewidth)
		axs[1].plot(data[0], data[2], "--", linewidth=linewidth)
		axs[2].plot(data[0], data[3], "--", linewidth=linewidth)
	else: 
		axs[0].plot(data[0], data[1], linewidth=linewidth)
		axs[1].plot(data[0], data[2], linewidth=linewidth)
		axs[2].plot(data[0], data[3], linewidth=linewidth)


for ax in axs:
	ax.legend(legends, bbox_to_anchor=(1.1, 1.05))
	ax.grid(b=True,which='both')
	ax.set(xlabel='time [s]', ylabel='pos [m]')
	ax.legend(legends, bbox_to_anchor=(1.1, 1.05))

#axs[1].set(ylabel=all_labels[k])
#axs[1].set_xticks(box_label_range)
#axs[1].set_xticklabels(legends, rotation=40)


plt.tight_layout()
plt.show()

file_name = "{}.pdf".format("ground_truth_vs_graph")
print(os.path.join(log_dir, file_name))
fig.savefig(os.path.join(log_dir, file_name), format='pdf')
