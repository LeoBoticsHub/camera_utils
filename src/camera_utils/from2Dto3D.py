import open3d as o3d
import cv2
import numpy as np
import math

def compute_centroids_with_point_cloud(rgb, depth, out_mask,intrinsic):
    # convert image to np array
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    out_mask = np.asarray(out_mask, np.uint8)
    points_and_angles = []
    for j in range(out_mask.shape[0]):
        rgb_new = rgb.copy()
        depth_new = depth.copy()

        # extract mask from rgb and depth
        for i in range(3):
            rgb_new[:, :, i] = np.multiply(rgb[:, :, i], out_mask[j, :, :])
        depth_new = np.multiply(depth_new, out_mask[j, :, :])

#       compute angle

        gray_cluster = cv2.cvtColor(rgb_new, cv2.COLOR_BGR2GRAY)
        cnt, _ = cv2.findContours(gray_cluster, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        x_sum, y_sum = 0, 0

        for cont in cnt[0]:
            x_sum += cont[0, 0]
            y_sum += cont[0, 1]

        mu = [int(x_sum / cnt[0].shape[0]), int(y_sum / cnt[0].shape[0])]
        newX = (mu[0] / rgb.shape[0]) * depth.shape[0]
        newY = (mu[1] / rgb.shape[1]) * depth.shape[1]
        mu = [round(newX), round(newY)]

        Ixx, Ixy, Iyy = 0, 0, 0
        for cont in cnt[0]:
            Ixx += pow(cont[0, 0] - mu[0], 2)
            Ixy += (cont[0, 0] - mu[0]) * (cont[0, 1] - mu[1])
            Iyy += pow(cont[0, 1] - mu[1], 2)

        alpha = (0.5 * math.atan2((2 * Ixy), (Ixx - Iyy)))

#       compute center

        rgb_new = o3d.geometry.Image(rgb_new)
        depth_new = o3d.geometry.Image(depth_new)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_new, depth_new)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        center = pcd.get_center()

        points_and_angles.append([center, alpha])

    return points_and_angles


def compute_centroids(rgb, depth, out_mask):
    # convert image to np array
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    out_mask = np.asarray(out_mask, np.uint8)
    points_and_angles = []
    for j in range(out_mask.shape[0]):
        rgb_new = rgb.copy()
        # extract mask from rgb and depth
        # pdb.set_trace()
        for i in range(3):
            rgb_new[:, :, i] = np.multiply(rgb[:, :, i], out_mask[j, :, :])
        # depth_new = np.multiply(depth, out_mask[j, :, :])

        # convert masked rgb to gray image
        gray_cluster = cv2.cvtColor(rgb_new, cv2.COLOR_BGR2GRAY)
        # compute cluter contours
        cnt, _ = cv2.findContours(gray_cluster, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # compute image centroid mu = [x_mean, y_mean]
        x_sum, y_sum = 0, 0

        for cont in cnt[0]:
            x_sum += cont[0, 0]
            y_sum += cont[0, 1]

        mu = [int(x_sum / cnt[0].shape[0]), int(y_sum / cnt[0].shape[0])]
        newX = (mu[0] / rgb.shape[0]) * depth.shape[0]
        newY = (mu[1] / rgb.shape[1]) * depth.shape[1]
        mu = [round(newX), round(newY)]
        # pdb.set_trace()
        # compute 3D spatial coordinates with respect to the camera reference frame
        z = depth[mu[1], mu[0]] * 0.001
        x = ((mu[0] - principal_point[0]) * z) / focal_length[0]
        y = ((mu[1] - principal_point[1]) * z) / focal_length[1]

        center = [x, y, z]

        # compute cluster angle with respect to x axis using its "inertial" matrix
        Ixx, Ixy, Iyy = 0, 0, 0
        for cont in cnt[0]:
            Ixx += pow(cont[0, 0] - mu[0], 2)
            Ixy += (cont[0, 0] - mu[0]) * (cont[0, 1] - mu[1])
            Iyy += pow(cont[0, 1] - mu[1], 2)

        alpha = (0.5 * math.atan2((2 * Ixy), (Ixx - Iyy)))

        # append the centroid and the angle of the considered mask in the list.
        points_and_angles.append([center, alpha])
    return points_and_angles
