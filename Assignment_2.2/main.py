import open3d as o3d
import numpy as np
from PIL import Image

import os

import utils

THRESH = 0.00004
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

    occupancy = np.zeros((int(x_max - x_min), int(z_max - z_min)), dtype=np.float64)

    for i in range(pcd.shape[0]):
        x = int(pcd[i, 0] + x_min)
        z = int(pcd[i, 2] + z_min)

        occupancy[x, z] += 1

    occupancy /= pcd.shape[0]
    return occupancy > THRESH

def numpy_to_image(arr: np.ndarray, path: str):
    img = Image.fromarray(arr)
    img.save(f'{path}.png')


transf = utils.readData(DATASET_TR_PATH)
arr = utils.readPointCloud(DATASET_PATH + '000000.bin')[:, :3]
arr = utils.lidar_to_world(arr)
arr = utils.make_homogenous_and_transform(arr, transf[0].reshape(3, 4))
occ = pcd_to_occupancy(arr)
numpy_to_image((occ * 255).astype(np.uint8), 'test')


    

