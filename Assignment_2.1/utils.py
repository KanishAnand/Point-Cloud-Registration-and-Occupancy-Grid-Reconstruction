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

if __name__ == "__main__":
	pcd = o3d.geometry.PointCloud()
	transformation_matrix = readData('dataset/01.txt')

	for file in os.listdir('dataset/01'):
		arr = readPointCloud('dataset/01/' + file)
		
		pcd_cur = o3d.geometry.PointCloud()
		pcd_cur.points = o3d.utility.Vector3dVector(arr[:,:3])	

		#  applying rotation matrix to convert lidar to world
		lidar2world = rotation_from_euler_zyx(np.pi/2,-np.pi/2,0)
		mat = np.matmul(pcd_cur.points, lidar2world.T)
		pcd_cur.points = o3d.utility.Vector3dVector(mat)	

		fileno = file[:-4]
		fileno = int(fileno)

		transform_arr = transformation_matrix[fileno].reshape(3,4)
		transform_arr =  np.vstack((transform_arr, [0,0,0,1]))

		# applying transformation without use of transform function
		xyz = np.asarray(pcd_cur.points)
		points = np.c_[xyz, np.ones(xyz.shape[0])]
		P2 = np.matmul(points, transform_arr.T)

		pcd_cur.points = o3d.utility.Vector3dVector(P2[:,:3])	

		pcd_cur = pcd_cur.voxel_down_sample(voxel_size = 1)
		pcd += pcd_cur

	o3d.visualization.draw_geometries([pcd])

