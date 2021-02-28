import os
import numpy as np
import argparse
import open3d as o3d
from urllib.request import urlretrieve
from util.visualization import get_colored_point_cloud_feature
from util.misc import extract_features

from model.resunet import ResUNetBN2C
import time

import torch
import copy

if not os.path.isfile('ResUNetBN2C-16feat-3conv.pth'):
	print('Downloading weights...')
	urlretrieve(
		  "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
		  'ResUNetBN2C-16feat-3conv.pth')

if not os.path.isfile('redkitchen-20.ply'):
  print('Downloading a mesh...')
  urlretrieve("https://node1.chrischoy.org/data/publications/fcgf/redkitchen-20.ply",
              'redkitchen-20.ply')
              
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use the distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def demo(config):
	device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
	print("cuda for torch: ", torch.cuda.is_available())
	#print("cuda for open3d: ", o3d.cuda.is_available())
	start_load = time.time()
	checkpoint = torch.load(config.model)
	start_model = time.time()
	model = ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
	model.load_state_dict(checkpoint['state_dict'])
	model.eval()
	
	model = model.to(device)
	start_point_cloud = time.time()
  
	# pcd = o3d.io.read_point_cloud(config.input)
	pcd = o3d.io.read_point_cloud('demo_clouds/cloud_bin_0.ply')
	pcd_map = o3d.io.read_point_cloud('demo_clouds/cloud_bin_1.ply')

	start_extraction_1 = time.time()
	xyz_down, feature = extract_features(
		model,
		xyz=np.array(pcd.points),
		voxel_size=config.voxel_size,
		device=device,
		skip_check=True)
		
	start_extraction_2 = time.time()
	xyz_down_map, feature_map = extract_features(
		model,
		xyz=np.array(pcd_map.points),
		voxel_size=config.voxel_size,
		device=device,
		skip_check=True)
  

	#  vis_pcd = o3d.geometry.PointCloud()
	#  vis_pcd.points = o3d.utility.Vector3dVector(xyz_down)
	#
	#  vis_pcd = get_colored_point_cloud_feature(vis_pcd,
	#                                            feature.detach().cpu().numpy(),
	#                                            config.voxel_size)
	#  o3d.visualization.draw_geometries([vis_pcd])
	
	ref = o3d.pipelines.registration.Feature()
	#ref.data = (feature_map.detach().cpu().numpy().T)#.astype(np.float64)
	ref.data = (feature_map.detach().cpu().numpy().T)#.astype(np.float64)

	test = o3d.pipelines.registration.Feature()
	#test.data = (feature.detach().cpu().numpy().T)#.astype(np.float64)
	test.data = (feature.detach().cpu().numpy().T)#.astype(np.float64)


	map_pcd = o3d.geometry.PointCloud()
	map_pcd.points = o3d.utility.Vector3dVector(xyz_down_map)

	sensor_pcd = o3d.geometry.PointCloud()
	sensor_pcd.points = o3d.utility.Vector3dVector(xyz_down)


	start_global = time.time()
	result_ransac = execute_global_registration(sensor_pcd, map_pcd, test, ref, config.voxel_size)
	#result_ransac = execute_fast_global_registration(sensor_pcd, map_pcd, test, ref, 0.02)
	print("config loading took %.3f sec.\n" % (start_model - start_load ))
	print("model loading took %.3f sec.\n" % (start_point_cloud - start_model ))
	print("point cloud loading took %.3f sec.\n" % (start_extraction_1 - start_point_cloud ))
	print("extraction on point cloud 1 took %.3f sec.\n" % (start_extraction_2 - start_extraction_1 ))
	print("extraction on point cloud 2 took %.3f sec.\n" % (start_global - start_extraction_2 ))
	print("Global registration took %.3f sec.\n" % (time.time() - start_global))
	print(result_ransac.transformation)

	in_pc = pcd # full size cloud
	map_pc = pcd_map  # full size cloud
	
	#draw_registration_result(sensor_pcd, map_pcd, result_ransac.transformation)
	draw_registration_result(in_pc, map_pc, result_ransac.transformation)

def draw_registration_result(source, target, transformation):
	source_temp = copy.deepcopy(source)
	target_temp = copy.deepcopy(target)
	source_temp.paint_uniform_color([1, 0.706, 0])
	target_temp.paint_uniform_color([0, 0.651, 0.929])
	source_temp.transform(transformation)
	o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '-i',
      '--input',
      default='redkitchen-20.ply',
      type=str,
      help='path to a pointcloud file')
  parser.add_argument(
      '-m',
      '--model',
      default='ResUNetBN2C-16feat-3conv.pth',
      type=str,
      help='path to latest checkpoint (default: None)')
  parser.add_argument(
      '--voxel_size',
      default=0.025,
      type=float,
      help='voxel size to preprocess point cloud')

  config = parser.parse_args()
  demo(config)
