from enum import Enum
import open3d as o3d

class Camera:

    class Resolution(Enum):
        HD = 0
        FullHD = 1
        QHD = 2

    camera_name = ""
    intr = []
    pipeline = 0
    rgb_resolution = 0
    depth_resolution = 0
    fps = 0
    serial_number = ""

    o3d_intr = 0

    def __init__(self, rgb_resolution=Resolution.FullHD, depth_resolution=Resolution.HD, fps=30, serial_number=""):
        self.rgb_resolution = rgb_resolution
        self.depth_resolution = depth_resolution
        self.fps = fps
        self.serial_number = serial_number

        print("%s initialization" % self.camera_name)

    def get_intrinsics(self):
        '''
        :return: camera intrinsics
        '''
        return self.intr

    def get_name(self):
        '''
        :return: camera name
        '''
        return self.camera_name

    def get_serial_number(self):
        '''
        :return: camera serial number
        '''
        return self.serial_number

    def get_fps(self):
        '''
        :return: camera frames per second
        '''
        return self.fps

    def get_rgb(self):
        raise NotImplementedError

    def get_depth(self):
        raise NotImplementedError

    def get_frames(self):
        raise NotImplementedError

    def get_aligned_frames(self):
        raise NotImplementedError
    
    def set_option(self, option, value):
        raise NotImplementedError

    def get_option(self, option):
        raise NotImplementedError

    def get_pcd(self, depth_truncation=5.0, display_pcd=False):
        '''
        :param depth_truncation: [m] only depth values smaller than this distance will be considered
        
        :return: open3d pcd 
        '''
        rgb, depth = self.get_frames()

        depth = o3d.geometry.Image(depth)
        rgb = o3d.geometry.Image(rgb)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, depth_trunc=depth_truncation, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.o3d_intr)

        if display_pcd:
            pcd_display = o3d.geometry.PointCloud(pcd)
            pcd_display.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            o3d.visualization.draw_geometries([pcd_display])

        return pcd