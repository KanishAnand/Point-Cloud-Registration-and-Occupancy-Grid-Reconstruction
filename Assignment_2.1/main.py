import open3d as o3d
import numpy as np 

import utils

import os
import shutil

DATASET_PATH = '../dataset/01/'
DATASET_TR_PATH = '../dataset/01.txt'

if __name__ == "__main__":
    pcd = o3d.geometry.PointCloud()
    transf = utils.readData(DATASET_TR_PATH)

    for filename in os.listdir(DATASET_PATH):
        arr = utils.readPointCloud(os.path.join(DATASET_PATH, filename))[:, :3]
        arr = utils.lidar_to_world(arr)
        ind = int(filename[:-4])
        arr = utils.make_homogenous_and_transform(arr, transf[ind].reshape(3, 4))
        
        pcd_cur = o3d.geometry.PointCloud()
        pcd_cur.points = o3d.utility.Vector3dVector(arr)
        pcd_cur = pcd_cur.voxel_down_sample(voxel_size = 1)
        pcd += pcd_cur


    o3d.visualization.draw_geometries([pcd])

