import numpy as np
import os
import open3d
import open3d as o3d

if any(File.endswith(".npz") for File in os.listdir("/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/")):
    print("true")
else:
    print("false")




outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/threedmatch/7-scenes-chess@seq-01_004.npz"
outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/threedmatch/7-scenes-chess@seq-01_008.npz"

outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/groundtruth_imu_frame@seq_00134.npz"; outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/pcl_ballast_tank[-0.05_1.40_0.08][5.53_3.05_2.26].npz"

outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/groundtruth_imu_frame@seq_00742.npz"; outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/pcl_ballast_tank[-0.05_1.40_0.08][5.53_3.05_2.26].npz"

outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/groundtruth_imu_frame@seq_00204.npz"; outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/noisy_3_groundtruth_imu_frame@seq_00204.npz"

#outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/groundtruth_imu_frame@seq_00633.npz"; outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/pcl_ballast_tank[-0.05_1.40_0.08][5.53_3.05_2.26].npz"

#outfile1 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/groundtruth_imu_frame@seq_00289.npz"; outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/pcl_ballast_tank[-0.05_1.40_0.08][5.53_3.05_2.26].npz"


#outfile2 = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/ballast_tank.npz"
npzfile1 = np.load(outfile1)
npzfile2 = np.load(outfile2)
print(npzfile1.files)
print(npzfile1['pcd'])
print(npzfile1['color'])


xyz1 = npzfile1['pcd']
xyz2 = npzfile2['pcd']

source = open3d.geometry.PointCloud()
target = open3d.geometry.PointCloud()

source.points = open3d.utility.Vector3dVector(xyz1)
source.paint_uniform_color([0,1,0])
target.points = open3d.utility.Vector3dVector(xyz2)
target.paint_uniform_color([1,0,0])

threshold = 0.025
T_rough = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
 


#source.transform(T_rough)

evaluation = o3d.pipelines.registration.evaluate_registration(
	source, target, threshold)


pcd_source = open3d.geometry.PointCloud()
pcd_target = open3d.geometry.PointCloud()
pcd_source.points = o3d.utility.Vector3dVector(xyz1)
pcd_target.points = o3d.utility.Vector3dVector(xyz2)
pcd_source = pcd_source.voxel_down_sample(voxel_size=0.025)
pcd_target = pcd_target.voxel_down_sample(voxel_size=0.025)

pcd_combined = pcd_source + pcd_target

pcd_merged = pcd_combined.voxel_down_sample(voxel_size=0.025)

p_source = len(pcd_source.points)
p_target = len(pcd_target.points)
p_merged = len(pcd_merged.points)
p_rest = p_source + p_target - p_merged
p_overlap = p_rest/(p_merged)

print(p_source, p_target, p_merged)
print(p_rest, p_overlap)


open3d.visualization.draw([source, target])



'''
print("before:", evaluation)
reg_p2p = o3d.pipelines.registration.registration_icp(
	source, target, threshold, T_rough, 
	o3d.pipelines.registration.TransformationEstimationPointToPoint(),
	o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print("after:", reg_p2p)
print(reg_p2p.transformation)

source.transform(reg_p2p.transformation)
open3d.visualization.draw([source, target])
'''
       

