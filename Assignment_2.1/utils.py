import os
import open3d as o3d
import numpy as np 

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


if __name__ == "__main__":
	pcd = o3d.geometry.PointCloud()
	transformation_matrix = readData('dataset/01.txt')

	for file in os.listdir('dataset/01'):
		arr = readPointCloud('dataset/01/' + file)
		
		pcd_cur = o3d.geometry.PointCloud()
		pcd_cur.points = o3d.utility.Vector3dVector(arr[:,:3])	

		lidar2world = pcd_cur.get_rotation_matrix_from_zyx((np.pi/2,-np.pi/2,0))
		pcd_cur.rotate(lidar2world, pcd_cur.get_center())

		fileno = file[:-4]
		fileno = int(fileno)

		transform_arr = transformation_matrix[fileno].reshape(3,4)
		transform_arr =  np.vstack((transform_arr, [0,0,0,1]))

		pcd_cur.transform(transform_arr)

		pcd_cur = pcd_cur.voxel_down_sample(voxel_size = 2)
		pcd += pcd_cur
		# break

	o3d.visualization.draw_geometries([pcd])

