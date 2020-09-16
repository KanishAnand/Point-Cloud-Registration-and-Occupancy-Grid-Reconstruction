import open3d as o3d
import numpy as np
from PIL import Image
import cv2

import os

import utils

THRESH = 0.00009
DATASET_PATH = '../dataset/01/'
DATASET_TR_PATH = '../dataset/01.txt'


def pcd_to_occupancy(pcd: np.ndarray):
    if pcd.shape[1] != 3:
        raise ValueError(f'pcd should have 3 columns, instead got {pcd.shape[1]}')
    
    pcd = pcd.round()

    x_min = pcd[:, 0].min()
    x_max = pcd[:, 0].max()

    z_min = pcd[:, 2].min()
    z_max = pcd[:, 2].max()

    print(x_min)
    print(x_max)

    occupancy = np.zeros((int(x_max - x_min + 1), int(z_max - z_min + 1)), dtype=np.float64)

    for i in range(pcd.shape[0]):
        x = int(pcd[i, 0] - x_min)
        z = int(pcd[i, 2] - z_min)

        occupancy[x, z] += 1

    occupancy /= pcd.shape[0]
    return occupancy > THRESH

def numpy_to_image(arr: np.ndarray, path: str):
    cv2.imwrite('test.png',arr)


if __name__ == "__main__":
    transf = utils.readData(DATASET_TR_PATH)
    pcd = o3d.geometry.PointCloud()

    for ind in range(0,5):
        file_name = '%06d.bin'%(ind) 
        arr = utils.readPointCloud(DATASET_PATH + file_name)[:, :3]
        arr = utils.lidar_to_world(arr)
        arr = utils.make_homogenous_and_transform(arr, transf[ind].reshape(3, 4))

        pcd_cur = o3d.geometry.PointCloud()
        pcd_cur.points = o3d.utility.Vector3dVector(arr)	
        pcd += pcd_cur


    final_arr = np.asarray(pcd.points)
    occ = pcd_to_occupancy(final_arr)
    numpy_to_image((occ * 255).astype(np.uint8), 'test')
    # o3d.visualization.draw_geometries([pcd])


    

