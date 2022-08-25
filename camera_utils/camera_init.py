import pyrealsense2 as rs
import numpy as np
from enum import Enum

try:
    import pyzed.sl as sl
except ImportError:
    print("pyzed.sl module not found")


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


class IntelRealsense(Camera):

    def __init__(self, rgb_resolution=Camera.Resolution.HD, depth_resolution=Camera.Resolution.HD, fps=30,
                 serial_number="", depth_in_meters=False):

        self.camera_name = "Intel Realsense"
        Camera.__init__(self, rgb_resolution, depth_resolution, fps, serial_number)

        # start camera
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.serial_number != "":
            config.enable_device(self.serial_number)

        # set resolutions
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, self.fps)  # max 1280x720 at 90 fps
        if self.rgb_resolution == Camera.Resolution.HD:
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, self.fps)  # max 1920x1080 at 30 fps
        else:
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, self.fps)  # max 1920x1080 at 30 fps

        # Start streaming
        try:
            cfg = self.pipeline.start(config)
        except RuntimeError:
            print("\n\033[1;31;40mError during camera initialization.\nMake sure to have set the right RGB camera resolution. Some cameras doesn't have FullHD resolution (e.g. Intel Realsense D455).\nIf you have connected more cameras make sure to insert the serial numbers to distinguish cameras during initialization.\033[0m\n")
            exit(1)

        profile = cfg.get_stream(rs.stream.color)
        intr = profile.as_video_stream_profile().get_intrinsics()
        self.intr = {'fx': intr.fx, 'fy': intr.fy, 'px': intr.ppx, 'py': intr.ppy}

        if depth_in_meters:
            self.mm2m_conversion = 1000
        else:
            self.mm2m_conversion = 1

        print("%s camera configured.\n" % self.camera_name)

    def __del__(self):
        try:
            self.pipeline.stop()
            print("%s camera closed" % self.camera_name)
        except RuntimeError as ex:
            print("\033[0;33;40mException (%s): %s\033[0m" % (type(ex).__name__, ex))
        

    def get_rgb(self):
        '''
        :return: An rgb image as numpy array
        '''
        color_frame = None
        while not color_frame:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        color_frame = np.asanyarray(color_frame.get_data())
        return color_frame

    def get_depth(self):
        '''
        :return: A depth image (1 channel) as numpy array
        '''
        depth_frame = None
        while not depth_frame:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
        depth_frame = np.asanyarray(depth_frame.get_data(), dtype=np.uint16) 
        return np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)

    def get_frames(self):
        '''
        :return: rgb, depth images as numpy arrays
        '''
        depth_frame_cam, color_frame_cam = None, None
        while not color_frame_cam or not depth_frame_cam:
            frames = self.pipeline.wait_for_frames()
            depth_frame_cam = frames.get_depth_frame()
            color_frame_cam = frames.get_color_frame()
        depth_frame = np.asanyarray(depth_frame_cam.get_data()) / self.mm2m_conversion
        color_frame = np.asanyarray(color_frame_cam.get_data())

        return color_frame, np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)

    def get_aligned_frames(self):
        '''
        :return: rgb, depth images aligned with post-processing as numpy arrays
        '''
        depth_frame_cam, color_frame_cam = None, None
        while not color_frame_cam or not depth_frame_cam:
            frames = self.pipeline.wait_for_frames()
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            color_frame_cam = aligned_frames.first(rs.stream.color)
            depth_frame_cam = aligned_frames.get_depth_frame()
        depth_frame = np.asanyarray(depth_frame_cam.get_data()) / self.mm2m_conversion
        color_frame = np.asanyarray(color_frame_cam.get_data())

        return color_frame, np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)
        
    def set_option(self, option, value):
        '''
        :param option: the option to be set (rs.option.OPTION_NAME)
        :param value: the value of the option
        '''
        option_name = str(option)
        try:
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
            sensor.set_option(option, value)
            option_name = str(option).replace('option.', '').upper()
            print("Option %s changed to value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))

    def get_option(self, option):
        '''
        :param option: the option to be got (rs.option.OPTION_NAME)
        '''
        option_name = str(option).replace('option.', '').upper()
        try:
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
            value = sensor.get_option(option)
            print("Option %s value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[1;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))


class Zed(Camera):

    single_camera_mode = True

    def __init__(self, single_camera_mode=True, rgb_resolution=Camera.Resolution.FullHD,
                 depth_resolution=Camera.Resolution.FullHD, fps=30, serial_number=""):
        '''
        :param single_camera_mode: boolean to choose if using the camera as a single camera or a dual camera (two images ...)
        :param rgb_resolution: the rgb resolution of the camera (e.g., Zed.Resolution.HD)
        :param depth_resolution: the depth resolution of the camera (e.g., Zed.Resolution.HD)
        :param fps: the camera frames per second
        :param serial_number: the camera serial number
        '''
        self.camera_name = "ZED"
        Camera.__init__(self, rgb_resolution, depth_resolution, fps, serial_number)

        self.single_camera_mode = single_camera_mode
        self.pipeline = sl.Camera()
        # set initial parameters
        init_params = sl.InitParameters()

        # set resolution
        if self.rgb_resolution == Camera.Resolution.QHD:
            init_params.camera_resolution = sl.RESOLUTION.HD2K
            if self.fps > 15:
                self.fps = 15
        elif self.rgb_resolution == Camera.Resolution.HD:
            init_params.camera_resolution = sl.RESOLUTION.HD720
        else:
            init_params.camera_resolution = sl.RESOLUTION.HD1080

        init_params.camera_fps = self.fps

        # start the camera
        err = self.pipeline.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("ERROR: problem starting the camera")
            exit()

        # get intrinsics
        zed_params = self.pipeline.get_camera_information().calibration_parameters
        left_intr = zed_params.left_cam

        if self.single_camera_mode:
            self.intr = {'fx': left_intr.fx, 'fy': left_intr.fy, 'px': left_intr.cx, 'py': left_intr.cy}
        else:
            right_intr = zed_params.right_cam
            self.intr = {'fx': [left_intr.fx, right_intr.fx],
                         'fy': [left_intr.fy, right_intr.fy],
                         'px': [left_intr.cx, right_intr.cx],
                         'py': [left_intr.cy, right_intr.cy]}

        print("%s camera configured.\n" % self.camera_name)

    def __del__(self):
        self.pipeline.close()
        print("%s camera closed" % self.camera_name)

    def get_rgb(self):
        '''
        :return: An rgb image as numpy array or [left, right] rgb images depending on camera mode (single or double)
        '''
        color_frame_left = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_image(color_frame_left, sl.VIEW.LEFT)
        color_frame_left = np.asanyarray(color_frame_left.get_data())[:, :, :3]
        if self.single_camera_mode:
            return color_frame_left
        else:
            color_frame_right = sl.Mat()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame_right, sl.VIEW.RIGHT)
            color_frame_right = np.asanyarray(color_frame_right.get_data())[:, :, :3]
            return color_frame_left, color_frame_right

    def get_depth(self):
        '''
        :return: A depth image as numpy array or [left, right] depth images depending on camera mode (single or double)
        '''
        depth_frame = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)
        depth_frame = np.asanyarray(depth_frame.get_data())
        depth_frame = np.nan_to_num(depth_frame)
        return depth_frame

    def get_frames(self):
        '''
        :return: rgb, depth image as numpy array or [left rgb, right rgb], depth images depending on camera mode
        '''
        color_frame_left = sl.Mat()
        depth_frame = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_image(color_frame_left, sl.VIEW.LEFT)
            self.pipeline.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)
        depth_frame = np.asanyarray(depth_frame.get_data())
        depth_frame = np.nan_to_num(depth_frame)
        color_frame_left = np.asanyarray(color_frame_left.get_data())[:, :, :3]
        if self.single_camera_mode:
            return color_frame_left, depth_frame
        else:
            color_frame_right = sl.Mat()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame_right, sl.VIEW.RIGHT)
            color_frame_right = np.asanyarray(color_frame_right.get_data())[:, :, :3]
            return [color_frame_left, color_frame_right], depth_frame

    def get_aligned_frames(self):
        return self.get_frames()

    def set_option(self, option, value):
        '''
        :param option: the option to be set (sl.)
        :param value: the value of the option
        '''
        option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
        try:
            self.pipeline.set_camera_settings(option, value)
            print("Option %s changed to value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))

    def get_option(self, option):
        '''
        :param option: the option to be got (sl.)
        '''
        option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
        try:
            value = self.pipeline.get_camera_settings(option)
            print("Option %s value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))
