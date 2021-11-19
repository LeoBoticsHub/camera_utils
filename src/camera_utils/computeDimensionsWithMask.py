import pdb

import numpy as np
import open3d as o3d
import math
from camera_utils.from2Dto3D import compute_angle_from_rgb


def compute_dimensions_with_angle(rgb, depth, mask, intrinsics, cam2plane_distance):

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

    if o3d.__version__ == '0.9.0.0':
        inlier_cloud = pcd.select_down_sample(inliers)
        # outlier_cloud = pcd.select_down_sample(inliers, invert=True)
    else:
        inlier_cloud = pcd.select_by_index(inliers)
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    # o3d.visualization.draw_geometries([inlier_cloud])

    max_x = np.max(np.asarray(inlier_cloud.points)[:, 0])
    min_x = np.min(np.asarray(inlier_cloud.points)[:, 0])
    max_y = np.max(np.asarray(inlier_cloud.points)[:, 1])
    min_y = np.min(np.asarray(inlier_cloud.points)[:, 1])
    # max_z = np.max(np.asarray(inlier_cloud.points)[:, 2])
    min_z = np.min(np.asarray(inlier_cloud.points)[:, 2])

    dimX = max_x - min_x
    dimY = max_y - min_y
    # dimZ = max_z - min_z
    dimZ = cam2plane_distance - min_z

    dim = [dimX, dimY, dimZ]

    print('X: %.2f cm, Y: %.2f cm, Z: %.2f cm, ' % (dim[0] * 100, dim[1] * 100, dim[2] * 100))
    return dim


def compute_dimensions_with_angles_points(rgb, depth, mask, intrinsics, cam2plane_distance):

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

    depth = o3d.geometry.Image(depth)
    rgb = o3d.geometry.Image(rgb)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # pcd = pcd.voxel_down_sample(voxel_size=0.005)

    #plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    try:
    	plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    except:
    	print('Error during plane generation')
    	return [0,0,0]
    if o3d.__version__ == '0.9.0.0':
        inlier_cloud = pcd.select_down_sample(inliers)
        # outlier_cloud = pcd.select_down_sample(inliers, invert=True)
    else:
        inlier_cloud = pcd.select_by_index(inliers)
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    max_x = np.max(np.asarray(inlier_cloud.points)[:, 0])
    min_x = np.min(np.asarray(inlier_cloud.points)[:, 0])
    max_y = np.max(np.asarray(inlier_cloud.points)[:, 1])
    min_y = np.min(np.asarray(inlier_cloud.points)[:, 1])
    max_z = np.max(np.asarray(inlier_cloud.points)[:, 2])
    min_z = np.min(np.asarray(inlier_cloud.points)[:, 2])

    inlier_array = np.asarray(inlier_cloud.points)

    down  = np.mean(inlier_array[np.where(inlier_array[:, 1] == min_y), :], 1)
    up    = np.mean(inlier_array[np.where(inlier_array[:, 1] == max_y), :], 1)
    left  = np.mean(inlier_array[np.where(inlier_array[:, 0] == min_x), :], 1)
    right = np.mean(inlier_array[np.where(inlier_array[:, 0] == max_x), :], 1)

    down_left  = np.linalg.norm(down - left)
    down_right = np.linalg.norm(down - right)
    up_right = np.linalg.norm(up - right)
    up_left = np.linalg.norm(up - left)

    side1 = np.mean([down_left, up_right])
    side2 = np.mean([down_right, up_left])
    # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(left))
    # pcd.paint_uniform_color([1.0, 0, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, pcd])

    dimX = max(side1, side2)
    dimY = min(side1, side2)
    # # dimZ = max_z - min_z
    dimZ = cam2plane_distance - min_z

    dim = [dimX, dimY, dimZ]

    print('X: %.2f cm, Y: %.2f cm, Z: %.2f cm, ' % (dim[0] * 100, dim[1] * 100, dim[2] * 100))
    return dim
