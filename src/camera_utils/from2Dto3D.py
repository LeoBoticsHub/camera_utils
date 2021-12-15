import pdb
import sys

import open3d as o3d
import cv2
import numpy as np
import math


def extract_contours(rgb):
    gray_cluster = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    cnt, _ = cv2.findContours(gray_cluster, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    return cnt


def compute_mu(rgb, depth, cnt):

    x_sum, y_sum = 0, 0
    for cont in cnt[0]:
        x_sum += cont[0, 0]
        y_sum += cont[0, 1]

    mu = [int(x_sum / cnt[0].shape[0]), int(y_sum / cnt[0].shape[0])]
    newX = (float(mu[0]) / rgb.shape[0]) * depth.shape[0]
    newY = (float(mu[1]) / rgb.shape[1]) * depth.shape[1]
    mu = [round(newX), round(newY)]


    return mu


def compute_angle(mu, cnt):
    Ixx, Ixy, Iyy = 0, 0, 0
    for cont in cnt[0]:
        Ixx += pow(cont[0, 0] - mu[0], 2)
        Ixy += (cont[0, 0] - mu[0]) * (cont[0, 1] - mu[1])
        Iyy += pow(cont[0, 1] - mu[1], 2)

    alpha = (0.5 * math.atan2((2 * Ixy), (Ixx - Iyy)))

    return alpha


def compute_angle_from_rgb(rgb, depth):
    cnt = extract_contours(rgb)
    mu = compute_mu(rgb, depth, cnt)
    alpha = compute_angle(mu, cnt)

    return alpha


def compute_centroids(rgb, depth, mask, intrinsics, use_pcd=True):
    if not rgb[:, :, 0].shape == depth.shape:
        sys.exit("\nfrom2Dto3D/compute_dimensions function: rgb and depth shapes are different."
                 " They should have equal dimensions.\n")
    focal_length = [intrinsics['fx'], intrinsics['fy']]
    principal_point = [intrinsics['px'], intrinsics['py']]

    if mask.shape[0] == 0 or mask is None:
        points_and_angles = [[[0, 0, 0], 0]]
        return points_and_angles
    # convert image to np array
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    mask = np.asarray(mask, np.uint8)

    points_and_angles = []

    if use_pcd:

        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(depth.shape[1], depth.shape[0], focal_length[0], focal_length[1], principal_point[0], principal_point[1])

        if mask.shape == depth.shape:
            mask = np.expand_dims(mask, axis=0)

        for j in range(mask.shape[0]):
            rgb_new = rgb.copy()
            depth_new = depth.copy()
            # extract mask from rgb and depth
            for i in range(3):
                rgb_new[:, :, i] = np.multiply(rgb[:, :, i], mask[j, :, :])
            depth_new = np.multiply(depth_new, mask[j, :, :])

            # compute angle
            try:
                alpha = compute_angle_from_rgb(rgb_new, depth_new)
            except:
                alpha = 0

            # compute center
            rgb_new = o3d.geometry.Image(rgb_new)
            depth_new = o3d.geometry.Image(depth_new)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_new, depth_new)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

            center = pcd.get_center()

            points_and_angles.append([center, alpha])

    else:

        for j in range(mask.shape[0]):
            rgb_new = rgb.copy()
            # extract mask from rgb
            for i in range(3):
                rgb_new[:, :, i] = np.multiply(rgb[:, :, i], mask[j, :, :])

            cnt = extract_contours(rgb_new)
            mu = compute_mu(rgb_new, depth, cnt)

            # compute 3D spatial coordinates with respect to the camera reference frame
            z = depth[mu[1], mu[0]] * 0.001
            x = ((mu[0] - principal_point[0]) * z) / focal_length[0]
            y = ((mu[1] - principal_point[1]) * z) / focal_length[1]

            center = [x, y, z]

            alpha = compute_angle(mu, cnt)

            # append the centroid and the angle of the considered mask in the list.
            points_and_angles.append([center, alpha])

    return points_and_angles
