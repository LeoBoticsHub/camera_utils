import pdb

import numpy as np
import open3d as o3d
import math
from camera_utils.from2Dto3D import compute_angle_from_rgb
import cv2


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

    return dim


def compute_dimensions_with_angles_points(rgb, depth, mask, intrinsics, cam2plane_distance):

    # set intrinsics for open3d
    width = max(depth.shape[0], depth.shape[1])
    height = min(depth.shape[0], depth.shape[1])
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width, height, intrinsics['fx'], intrinsics['fy'], intrinsics['px'], intrinsics['py'])

    # binarize mask
    mask[mask > 254] = 1

    new_rgb = rgb.copy()
    new_depth = depth.copy()

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
#--------------------------------------------------------------------------------
    cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rect = cv2.minAreaRect(cnt[0])
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # turn into ints
    box = box[np.argsort(box[:, 0])]

    if box[0, 1] > box[1, 1]:
        ul = box[0, :]
        dl = box[1, :]
    else:
        dl = box[0, :]
        ul = box[1, :]
    if box[2, 1] > box[3, 1]:
        ur = box[2, :]
        dr = box[3, :]
    else:
        ur = box[3, :]
        dr = box[2, :]

    ul_mask = np.zeros(mask.shape)
    dl_mask = np.zeros(mask.shape)
    dr_mask = np.zeros(mask.shape)
    ur_mask = np.zeros(mask.shape)

    ul_rgb = np.zeros(new_rgb.shape)
    ur_rgb = np.zeros(new_rgb.shape)
    dl_rgb = np.zeros(new_rgb.shape)
    dr_rgb = np.zeros(new_rgb.shape)

    ul_mask[ul[1], ul[0]] = 1
    dl_mask[dl[1], dl[0]] = 1
    dr_mask[dr[1], dr[0]] = 1
    ur_mask[ur[1], ur[0]] = 1

    for i in range(3):
        ul_rgb[:, :, i] = np.multiply(new_rgb[:, :, i], ul_mask)
        ur_rgb[:, :, i] = np.multiply(new_rgb[:, :, i], ur_mask)
        dl_rgb[:, :, i] = np.multiply(new_rgb[:, :, i], dl_mask)
        dr_rgb[:, :, i] = np.multiply(new_rgb[:, :, i], dr_mask)

    ul_rgb = np.array(ul_rgb, dtype=np.uint8)
    ur_rgb = np.array(ur_rgb, dtype=np.uint8)
    dl_rgb = np.array(dl_rgb, dtype=np.uint8)
    dr_rgb = np.array(dr_rgb, dtype=np.uint8)

    # # extract mask from depth
    # new_depth = np.multiply(new_depth, new_mask)

    # new_rgb = np.array(new_rgb, dtype=np.uint8)
    # new_depth = np.array(new_depth, dtype=np.uint16)



    ul_depth = np.multiply(new_depth, ul_mask)
    ur_depth = np.multiply(new_depth, ur_mask)
    dl_depth = np.multiply(new_depth, dl_mask)
    dr_depth = np.multiply(new_depth, dr_mask)

    ul_depth = np.array(ul_depth, dtype=np.uint16)
    ur_depth = np.array(ur_depth, dtype=np.uint16)
    dl_depth = np.array(dl_depth, dtype=np.uint16)
    dr_depth = np.array(dr_depth, dtype=np.uint16)

    ul_depth = o3d.geometry.Image(ul_depth)
    ur_depth = o3d.geometry.Image(ur_depth)
    dl_depth = o3d.geometry.Image(dl_depth)
    dr_depth = o3d.geometry.Image(dr_depth)

    ul_rgb = o3d.geometry.Image(ul_rgb)
    ur_rgb = o3d.geometry.Image(ur_rgb)
    dl_rgb = o3d.geometry.Image(dl_rgb)
    dr_rgb = o3d.geometry.Image(dr_rgb)

    rgbd_ul = o3d.geometry.RGBDImage.create_from_color_and_depth(ul_rgb, ul_depth)
    rgbd_ur = o3d.geometry.RGBDImage.create_from_color_and_depth(ur_rgb, ur_depth)
    rgbd_dl = o3d.geometry.RGBDImage.create_from_color_and_depth(dl_rgb, dl_depth)
    rgbd_dr = o3d.geometry.RGBDImage.create_from_color_and_depth(dr_rgb, dr_depth)

    pcd_ul = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_ul, intrinsic)
    pcd_ur = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_ur, intrinsic)
    pcd_dl = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_dl, intrinsic)
    pcd_dr = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_dr, intrinsic)

    if np.sum(pcd_ul.points) == 0 or np.sum(pcd_ur.points) == 0 or np.sum(pcd_dl.points) == 0 or np.sum(pcd_dr.points) == 0:
        dim = [0, 0, 0]
        return dim



    # pcd_ul.paint_uniform_color([1.0, 0, 0])
    # pcd_ur.paint_uniform_color([1.0, 0, 0])
    # pcd_dl.paint_uniform_color([1.0, 0, 0])
    # pcd_dr.paint_uniform_color([1.0, 0, 0])

    # pcd2.paint_uniform_color([1.0, 0, 0])

    # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(new_rgb, new_depth)

    # pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # pcd2.paint_uniform_color([1.0, 0, 0])

    # o3d.visualization.draw_geometries([pcd, pcd_ur])


# --------------------------------------------------------------------------------

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

    # max_x = np.max(np.asarray(inlier_cloud.points)[:, 0])
    # min_x = np.min(np.asarray(inlier_cloud.points)[:, 0])
    # max_y = np.max(np.asarray(inlier_cloud.points)[:, 1])
    # min_y = np.min(np.asarray(inlier_cloud.points)[:, 1])
    # max_z = np.max(np.asarray(inlier_cloud.points)[:, 2])
    min_z = np.min(np.asarray(inlier_cloud.points)[:, 2])
    #
    # inlier_array = np.asarray(inlier_cloud.points)
    #
    # down  = np.mean(inlier_array[np.where(inlier_array[:, 1] == min_y), :], 1)
    # up    = np.mean(inlier_array[np.where(inlier_array[:, 1] == max_y), :], 1)
    # left  = np.mean(inlier_array[np.where(inlier_array[:, 0] == min_x), :], 1)
    # right = np.mean(inlier_array[np.where(inlier_array[:, 0] == max_x), :], 1)
    #
    # down_left  = np.linalg.norm(down - left)
    # down_right = np.linalg.norm(down - right)
    # up_right = np.linalg.norm(up - right)
    # up_left = np.linalg.norm(up - left)
    #
    # side1 = np.mean([down_left, up_right])
    # side2 = np.mean([down_right, up_left])
    # pcd_left = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(left))
    # pcd_right = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(right))
    # pcd_down = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(down))
    # pcd_up = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(up))
    # pcd_left.paint_uniform_color([1.0, 0, 0])
    # pcd_right.paint_uniform_color([1.0, 0, 0])
    # pcd_down.paint_uniform_color([1.0, 0, 0])
    # pcd_up.paint_uniform_color([1.0, 0, 0])
    inlier_cloud = inlier_cloud.voxel_down_sample(voxel_size=0.005)
    # o3d.visualization.draw_geometries([inlier_cloud, pcd_left, pcd_right, pcd_down, pcd_up])
    # o3d.visualization.draw_geometries([pcd_left, pcd_right, pcd_down, pcd_up])
    # o3d.visualization.draw_geometries([pcd])
    # inlier_cloud.paint_uniform_color([0, 1.0, 0])

    ul_min = 100000000
    ur_min = 100000000
    dl_min = 100000000
    dr_min = 100000000

    ul_point = pcd_ul.points[0][:2]
    ur_point = pcd_ur.points[0][:2]
    dl_point = pcd_dl.points[0][:2]
    dr_point = pcd_dr.points[0][:2]
    for point in inlier_cloud.points:

        ul_dist = np.linalg.norm(ul_point - point[:2])
        ur_dist = np.linalg.norm(ur_point - point[:2])
        dl_dist = np.linalg.norm(dl_point - point[:2])
        dr_dist = np.linalg.norm(dr_point - point[:2])
        if ul_dist < ul_min:
            ul_min = ul_dist
            ul_real = point

        if ur_dist < ur_min:
            ur_min = ur_dist
            ur_real = point

        if dl_dist < dl_min:
            dl_min = dl_dist
            dl_real = point

        if dr_dist < dr_min:
            dr_min = dr_dist
            dr_real = point

    # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([dr_real, dl_real, ul_real, ur_real]))
    # pcd.paint_uniform_color([1.0, 0, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, pcd])#, ur_real, dl_real, dr_real])

    left_side = np.linalg.norm(dl_real - ul_real)
    down_side = np.linalg.norm(dl_real - dr_real)
    up_side = np.linalg.norm(ul_real - ur_real)
    right_side = np.linalg.norm(ur_real - dr_real)

    side1 = np.mean([left_side, right_side])
    side2 = np.mean([down_side, up_side])

    dimX = max(side1, side2)
    dimY = min(side1, side2)
    # # dimZ = max_z - min_z
    dimZ = cam2plane_distance - min_z

    dim = [dimX, dimY, dimZ]

    return dim
