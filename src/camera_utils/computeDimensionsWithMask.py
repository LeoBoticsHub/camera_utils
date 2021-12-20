import pdb

import numpy as np
import open3d as o3d
import math
from camera_utils.from2Dto3D import compute_angle_from_rgb
import cv2


def find_vertices_from_mask(mask):
    '''
    Find the vertices of the rectangle containing the object in the mask

    @param mask: the object mask

    @return: A dictionary containing the X,Y image position of the four vertices
    {"UL": ul, "DL": dl, "UR": ur, "DR": dr}
    '''
    cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rect = cv2.minAreaRect(cnt[0])
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # turn into ints
    box = box[np.argsort(box[:, 0])]  # sort box vertices with respect x position

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

    vertices = {"UL": ul, "DL": dl, "UR": ur, "DR": dr}
    return vertices


def compute_dimensions_with_angle(rgb, depth, mask, intrinsics, cam2plane_distance):
    '''
    Compute the dimensions of an object using angle information to rotate the PointCloud

    @param rgb: the rgb image
    @param depth: the depth image
    @param mask: the object mask
    @param intrinsics: the camera intrinsics parameters
    @param cam2plane_distance: [m] the distance between the camera and the plane where the object is

    @return: the object dimensions in form [dim_x, dim_y, dim_z]
    '''
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

    dim_x = max_x - min_x
    dim_y = max_y - min_y
    # dim_z = max_z - min_z
    dim_z = cam2plane_distance - min_z

    dim = [dim_x, dim_y, dim_z]

    return dim


def compute_dimensions_with_angles_points(rgb, depth, mask, intrinsics, cam2plane_distance):
    '''
    Compute the dimensions of an object using the pointcloud library and the mask vertices

    @param rgb: the rgb image
    @param depth: the depth image
    @param mask: the object mask
    @param intrinsics: the camera intrinsics parameters
    @param cam2plane_distance: [m] the distance between the camera and the plane where the object is

    @return: the object dimensions in form [dim_x, dim_y, dim_z]
    '''

    # set intrinsics for open3d
    width = max(depth.shape[0], depth.shape[1])
    height = min(depth.shape[0], depth.shape[1])
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width, height, intrinsics['fx'], intrinsics['fy'], intrinsics['px'], intrinsics['py'])

    cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    # pdb.set_trace()
    for cont in cnt:
        if cv2.contourArea(cont) > max_area:
            max_area = cv2.contourArea(cont)
            max_cnt = cont

    rect = cv2.minAreaRect(max_cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # turn into ints
    mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.fillPoly(mask, pts=[box], color=(255, 255, 255))
    mask = np.array(mask, dtype=np.uint8)

    cv2.namedWindow('Mask rect', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask rect', mask)

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

# # -------------------------FIND VERTICES----------------------------------------------
    vertices = find_vertices_from_mask(mask)
    vertices_pcd = {}

    for key, point in vertices.items():
        # initialize mask and rgb for points
        point_mask = np.zeros(mask.shape)
        point_rgb = np.zeros(new_rgb.shape)


        # change a single value of the mask to one in order to have only the vertex position on the mask
        point_mask[point[1], point[0]] = 1

        # lets extract the point position
        point_depth = np.multiply(depth, point_mask)

        # If the point found on the depth has a zero value we need to found another value different from zero around
        # the point found
        counter = 0
        # pdb.set_trace()
        while np.sum(point_depth) == 0:
            # print('entering while with %s' % key)
            counter += 1
            if counter == 6:
                dim = [0, 0, 0]
                return dim
            # search around the vertex point for a depth value different from zero
            for i in range(-counter, counter+1):
                for j in range(-counter, counter+1):
                    # print(i, j)
                    point_mask = np.zeros(mask.shape)
                    point_mask[point[1] + i, point[0] + j] = 1
                    point_depth = np.multiply(depth, point_mask)
                    if np.sum(point_depth) != 0:
                        break
                else:
                    continue
                break

        # extract rgb
        for j in range(3):
            point_rgb[:, :, j] = np.multiply(new_rgb[:, :, j], point_mask)

        point_rgb = np.array(point_rgb, dtype=np.uint8)
        point_depth = np.array(point_depth, dtype=np.uint16)

        point_rgb = o3d.geometry.Image(point_rgb)
        point_depth = o3d.geometry.Image(point_depth)

        point_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(point_rgb, point_depth)
        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(point_rgbd, intrinsic)

        vertices_pcd[key] = point_cloud


# --------------------------------------------------------------------------------

    # plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    try:
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    except:
        print('Error during plane generation')
        return [0, 0, 0]
    if o3d.__version__ == '0.9.0.0':
        inlier_cloud = pcd.select_down_sample(inliers)
        # outlier_cloud = pcd.select_down_sample(inliers, invert=True)
    else:
        inlier_cloud = pcd.select_by_index(inliers)
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    min_z = np.min(np.asarray(inlier_cloud.points)[:, 2])
    inlier_cloud = inlier_cloud.voxel_down_sample(voxel_size=0.01)

#     # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([dr_real, dl_real, ul_real, ur_real]))
#     # pcd.paint_uniform_color([1.0, 0, 0])
#     # o3d.visualization.draw_geometries([inlier_cloud, pcd])#, ur_real, dl_real, dr_real])
#

    real_points = {}
    min_values = {}
    for key in vertices_pcd.keys():
        min_values[key] = 1000000

    for point in inlier_cloud.points:
        for key, pcd_point in vertices_pcd.items():

            pcd_point = pcd_point.points[0][:2]
            distance = np.linalg.norm(pcd_point - point[:2])

            if distance < min_values[key]:
                min_values[key] = distance
                real_points[key] = point

    left_side = np.linalg.norm(real_points['DL'] - real_points['UL'])
    down_side = np.linalg.norm(real_points['DL'] - real_points['DR'])
    up_side = np.linalg.norm(real_points['UL'] - real_points['UR'])
    right_side = np.linalg.norm(real_points['UR'] - real_points['DR'])


    side1 = np.mean([left_side, right_side])
    side2 = np.mean([down_side, up_side])

    # -------------- COMPUTE DIMENSIONS---------------

    dim_x = max(side1, side2)
    dim_y = min(side1, side2)
    # # dimZ = max_z - min_z
    dim_z = cam2plane_distance - min_z

    dim = [dim_x, dim_y, dim_z]

    return dim


# rgb is a list composed by left and right rgb and the same is for intrinsics
def compute_dimensions_with_stereo(masks, intrinsics, cam2cam_distance, cam2plane_distance):
    '''
    Compute the dimensions of an object using parallel stereo images with no depth

    @param masks: the object masks [left, right]
    @param intrinsics: the cameras intrinsics parameters [left, right]
    @param cam2plane_distance: [m] the distance between the camera and the plane where the object is
    @param cam2cam_distance: [cm] the distance between the two cameras centers

    @return: the object dimensions in form [dim_x, dim_y, dim_z]
    '''

    left_mask = masks[0]
    right_mask = masks[1]

    cv2.namedWindow('left_mask', cv2.WINDOW_NORMAL)
    cv2.imshow('left_mask', left_mask)

    left_vertices = find_vertices_from_mask(left_mask)
    right_vertices = find_vertices_from_mask(right_mask)

    vertices3d = {}
    min_z = 100000

    for key in left_vertices.keys():

        x_left = left_vertices[key][0]
        x_right = right_vertices[key][0]

        if x_left is not None and x_right is not None:

            x_l = x_left - intrinsics['px'][0]
            x_r = x_right - intrinsics['px'][1]

            disparity = -(x_r - x_l * (intrinsics['px'][1] / intrinsics['px'][0]))
            depth = (intrinsics['fx'][1] * cam2cam_distance) / disparity
            mu = left_vertices[key]

            z = depth * 0.01
            x = ((mu[0] - intrinsics['px'][0]) * z) / intrinsics['fx'][1]
            y = ((mu[1] - intrinsics['py'][0]) * z) / intrinsics['fy'][1]

            vertices3d[key] = np.array([x, y, z])

            if z < min_z:
                min_z = z

    left_side = np.linalg.norm(vertices3d['DL'] - vertices3d['UL'])
    down_side = np.linalg.norm(vertices3d['DL'] - vertices3d['DR'])
    up_side = np.linalg.norm(vertices3d['UL'] - vertices3d['UR'])
    right_side = np.linalg.norm(vertices3d['UR'] - vertices3d['DR'])

    side1 = np.mean([left_side, right_side])
    side2 = np.mean([down_side, up_side])

    dim_x = max(side1, side2)
    dim_y = min(side1, side2)
    dim_z = cam2plane_distance - min_z

    dim = [dim_x, dim_y, dim_z]

    return dim