import os
import open3d as o3d
import numpy as np 

LIDAR_TO_WORLD_EULER = (np.pi/2,-np.pi/2,0)

def readData(filename):
	"""
	reads the ground truth file 
	returns a 2D array with each 
	row as GT pose(arranged row major form)
	array size should be 1101*12
	"""
	data = np.loadtxt(filename)
	#data[i].reshape(3,4)
	return data 


def readPointCloud(filename):
	"""
	reads bin file and returns
	as m*4 np array
	all points are in meters
	you can filter out points beyond(in x y plane)
	50m for ease of computation
	and above or below 10m
	"""
	pcl = np.fromfile(filename, dtype=np.float32,count=-1)
	pcl = pcl.reshape([-1,4])
	return pcl 


def rotation_from_euler_zyx(alpha, beta, gamma):
    R = np.zeros((3, 3), dtype='double')
    
    R[0, 0] = np.cos(alpha) * np.cos(beta)
    R[0, 1] = np.cos(alpha) * np.sin(beta) * np.sin(gamma) - np.sin(alpha) * np.cos(gamma)
    R[0, 2] = np.cos(alpha) * np.sin(beta) * np.cos(gamma) + np.sin(alpha) * np.sin(gamma)
    
    R[1, 0] = np.sin(alpha) * np.cos(beta)
    R[1, 1] = np.sin(alpha) * np.sin(beta) * np.sin(gamma) + np.cos(alpha) * np.cos(gamma)
    R[1, 2] = np.sin(alpha) * np.sin(beta) * np.cos(gamma) - np.cos(alpha) * np.sin(gamma)
    
    R[2, 0] = -np.sin(beta)
    R[2, 1] =  np.cos(beta) * np.sin(gamma)
    R[2, 2] =  np.cos(beta) * np.cos(gamma)
    
    return R

def lidar_to_world(arr: np.ndarray):
	'''
	just a rotation, so done separately
	'''
	if arr.shape[1] != 3:
		raise ValueError(f'arr should have 3 columns, instead got {arr.shape[1]}')

	T = rotation_from_euler_zyx(*LIDAR_TO_WORLD_EULER)
	return (T @ arr.T).T

def make_homogenous_and_transform(arr: np.ndarray, T: np.ndarray):
	if arr.shape[1] != 3 or T.shape != (3, 4):
		raise ValueError(f'arr should have 3 columns and T should be 3x4, instead got {arr.shape[1]} and {T.shape}')

	T_4 = np.vstack((T, [0, 0, 0, 1]))
	arr_4 = np.c_[arr, np.ones(arr.shape[0])]

	transformed_arr = T_4 @ arr_4.T
	return transformed_arr.T[:, :-1]

