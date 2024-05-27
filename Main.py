import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cv2
import pykitti
import plyfile as ply
from PIL import Image

# Base directory for KITTI dataset
base = r'C:\Users\Yassine\Desktop\M2\P_I\KiTTI\KITTI_SAMPLE\RAW'
date = '2011_09_26'
drive = '0009'
# Load KITTI dataset
dataset = pykitti.raw(base, date, drive, frames=range(0, 50, 1))

# Calibration matrix from IMU to Velodyne
calib_umu_velo_inv = np.linalg.inv(dataset.calib.T_velo_imu)

# Calibration matrix for camera
K = dataset.calib.K_cam2
# Transformation matrix from Velodyne to camera image plane
M_t = dataset.calib.T_cam2_velo[0:3, :]
# Projection matrix
P = K @ M_t

xyz = []  # List to store coordinates
rgbc = []  # List to store RGB colors

for i in range(50):
    # Get Velodyne points
    velo = dataset.get_velo(i)
    velo[:, -1] = 1  # Replace last column values with 1
    # Eliminate points with x-coordinate less than 5
    velo = velo[velo[:, 0] >= 5]

    # Transform points to camera coordinates
    M_P = (dataset.oxts[i][1] @ calib_umu_velo_inv @ velo.T).T
    M_P = M_P[:, 0:3]

    # Project 3D points to 2D camera image plane
    point_cam2 = (P @ velo.T).T
    point_cam2 = np.array([point_cam2[:, 0] / point_cam2[:, 2],
                           point_cam2[:, 1] / point_cam2[:, 2]], dtype='int32').T

    # Filter points within image boundaries
    I = np.array(dataset.get_cam2(i))
    l, c = I.shape[0], I.shape[1]
    p_i = point_cam2[((point_cam2[:, 0] > 0) & (point_cam2[:, 0] < c) &
                      (point_cam2[:, 1] > 0) & (point_cam2[:, 1] < l))]
    mask = (point_cam2[:, 0] > 0) & (point_cam2[:, 0] < c) & \
           (point_cam2[:, 1] > 0) & (point_cam2[:, 1] < l)
    M_P = M_P[mask]

    # Get colors of each point
    rgb = np.zeros([p_i.shape[0], 3])
    for j in range(rgb.shape[0]):
        rgb[j, 0:3] = I[p_i[j][1], p_i[j][0]]
    xyz.append(M_P)
    rgbc.append(rgb)

# Concatenate XYZ coordinates and RGB colors
xyzrgb = np.concatenate((np.concatenate(xyz), np.concatenate(rgbc)), axis=1)


# Uncomment this section if you want to save the result as a PLY file
vertex = np.array([tuple(e) for e in xyzrgb],
                  dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                         ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
el = ply.PlyElement.describe(vertex, 'vertex')
ply.PlyData([el]).write('TP5.ply')

