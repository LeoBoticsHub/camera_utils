from enum import Enum

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

    def get_pcd(self):
        raise NotImplementedError