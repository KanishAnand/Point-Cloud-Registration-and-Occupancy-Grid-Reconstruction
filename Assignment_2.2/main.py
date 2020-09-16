import open3d as o3d
import numpy as np
import cv2

import os
import shutil

import utils

THRESH = 0.0002
DATASET_PATH = '../dataset/01/'
DATASET_TR_PATH = '../dataset/01.txt'
RESULT_PATH_1 = './results/part-2.1/'
RESULT_PATH_2 = './results/part-2.2/'
COUNTS_2 = (5, 10, 15)


def pcd_to_occupancy(pcd: np.ndarray):
    if pcd.shape[1] != 3:
        raise ValueError(f'pcd should have 3 columns, instead got {pcd.shape[1]}')
    
    pcd = pcd.round()

    x_min = pcd[:, 0].min()
    x_max = pcd[:, 0].max()

    z_min = pcd[:, 2].min()
    z_max = pcd[:, 2].max()


    occupancy = np.zeros((int(x_max - x_min + 1), int(z_max - z_min + 1)), dtype=np.float64)

    for i in range(pcd.shape[0]):
        x = int(pcd[i, 0] - x_min)
        z = int(pcd[i, 2] - z_min)

        occupancy[x, z] += 1

    occupancy /= pcd.shape[0]
    return occupancy > THRESH

def numpy_to_image(arr: np.ndarray, path: str):
    cv2.imwrite(f'{path}.png',(arr * 255).astype(np.uint8))


if __name__ == "__main__":
    transf = utils.readData(DATASET_TR_PATH)
    
    if os.path.exists(RESULT_PATH_1):
        shutil.rmtree(RESULT_PATH_1)
    os.makedirs(RESULT_PATH_1)

    for filename in os.listdir(DATASET_PATH):
        arr = utils.readPointCloud(DATASET_PATH + filename)[:, :3]
        arr = utils.lidar_to_world(arr)
        ind = int(filename[:-4])
        arr = utils.make_homogenous_and_transform(arr, transf[ind].reshape(3, 4))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(arr)
        pcd = pcd.voxel_down_sample(voxel_size = 1)
        arr = np.asarray(pcd.points)

        occ = pcd_to_occupancy(arr)
        numpy_to_image(occ, os.path.join(RESULT_PATH_1, filename[:-4]))
    
    
    # PART-2
    if os.path.exists(RESULT_PATH_2):
        shutil.rmtree(RESULT_PATH_2)
    os.makedirs(RESULT_PATH_2)

    for count in COUNTS_2:
        pcd = o3d.geometry.PointCloud()

        for ind in range(count):
            file_name = '%06d.bin'%(ind) 
            arr = utils.readPointCloud(DATASET_PATH + file_name)[:, :3]
            arr = utils.lidar_to_world(arr)
            arr = utils.make_homogenous_and_transform(arr, transf[ind].reshape(3, 4))

            pcd_cur = o3d.geometry.PointCloud()
            pcd_cur.points = o3d.utility.Vector3dVector(arr)	
            pcd += pcd_cur # takes care of ensuring uniqueness


        pcd = pcd.voxel_down_sample(voxel_size = 1)
        final_arr = np.asarray(pcd.points)
        occ = pcd_to_occupancy(final_arr)
        numpy_to_image(occ, os.path.join(RESULT_PATH_2, f'from-{count}'))


    

