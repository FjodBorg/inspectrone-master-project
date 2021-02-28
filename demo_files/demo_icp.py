import os
import numpy as np
import argparse
import copy
import open3d as o3d
import time

from urllib.request import urlretrieve
from util.visualization import get_colored_point_cloud_feature
from util.misc import extract_features

#from core.knn import find_knn_gpu

from model.resunet import ResUNetBN2C

import torch

if not os.path.isfile('ResUNetBN2C-16feat-3conv.pth'):
  print('Downloading weights...')
  urlretrieve(
      "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
      'ResUNetBN2C-16feat-3conv.pth')
#================== From Open3D Registration =============================

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

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

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

#===========================================================================

def demo(config):
  device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

  checkpoint = torch.load(config.model)
  #model = ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
  model = ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
  model.load_state_dict(checkpoint['state_dict'])
  model.eval()

  model = model.to(device)

  pcd = o3d.io.read_point_cloud(config.input)
  pcd_map = o3d.io.read_point_cloud('cloud0.ply')

  xyz_down, feature = extract_features(
      model,
      xyz=np.array(pcd.points),
      voxel_size=config.voxel_size,
      device=device,
      skip_check=True)

  xyz_down_map, feature_map = extract_features(
      model,
      xyz=np.array(pcd_map.points),
      voxel_size=config.voxel_size,
      device=device,
      skip_check=True)
  

  ref = o3d.pipelines.registration.Feature()
  ref.data = (feature_map.detach().cpu().numpy().T)#.astype(np.float64)

  test = o3d.pipelines.registration.Feature()
  test.data = (feature.detach().cpu().numpy().T)#.astype(np.float64)


  map_pcd = o3d.geometry.PointCloud()
  map_pcd.points = o3d.utility.Vector3dVector(xyz_down_map)

  sensor_pcd = o3d.geometry.PointCloud()
  sensor_pcd.points = o3d.utility.Vector3dVector(xyz_down)

  start = time.time()
  result_ransac = execute_global_registration(sensor_pcd, map_pcd, test, ref, config.voxel_size)
  #result_ransac = execute_fast_global_registration(sensor_pcd, map_pcd, test, ref, 0.02)
  print("Global registration took %.3f sec.\n" % (time.time() - start))
  print(result_ransac.transformation)

  draw_registration_result(sensor_pcd, map_pcd, result_ransac.transformation)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '-i',
      '--input',
      default='cloud1.ply',
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
      default=0.03,
      type=float,
      help='voxel size to preprocess point cloud')

  config = parser.parse_args()
  demo(config)
