import pdb

import numpy as np
import open3d as o3d
import math
from from2Dto3D import compute_angle_from_rgb


def compute_dimensions(rgb, depth, mask, intrinsics, cam2plane_distance):

    # set intrinsics for open3d
    width = max(depth.shape[0], depth.shape[1])
    height = min(depth.shape[0], depth.shape[1])
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width, height, intrinsics['fx'], intrinsics['fy'], intrinsics['px'], intrinsics['py'])

    # binarize mask
    mask[mask > 254] = 1

    # extract mask from rgb
    for i in range(3):
        rgb[:, :, i] = np.multiply(rgb[:, :, i], mask)
    # extract mask from depth
    depth = np.multiply(depth, mask)

    angle = compute_angle_from_rgb(rgb, depth)

    depth = o3d.geometry.Image(depth)
    rgb = o3d.geometry.Image(rgb)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # pcd = pcd.voxel_down_sample(voxel_size=0.005)

    # flip the orientation, so it looks upright, not upside-down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    pcd.rotate([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    pcd = pcd.select_by_index(inliers)
    # pcd.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # o3d.visualization.draw_geometries([pcd])

    max_x = np.max(np.asarray(pcd.points)[:, 0])
    min_x = np.min(np.asarray(pcd.points)[:, 0])
    max_y = np.max(np.asarray(pcd.points)[:, 1])
    min_y = np.min(np.asarray(pcd.points)[:, 1])
    # max_z = np.max(np.asarray(pcd.points)[:, 2])
    min_z = np.min(np.asarray(pcd.points)[:, 2])

    dimX = max_x - min_x
    dimY = max_y - min_y
    # dimZ = max_z - min_z
    dimZ = cam2plane_distance - min_z

    dim = [dimX, dimY, dimZ]

    print('X: %.2f cm, Y: %.2f cm, Z: %.2f cm, ' % (dim[0] * 100, dim[1] * 100, dim[2] * 100))
    return dim

