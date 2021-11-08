import pdb

import pyrealsense2 as rs
import numpy as np
try:
    import pyzed.sl as sl
except ImportError:
    print("pyzed.sl module not found")



class Camera:

    camera_name = ""
    intr = []
    pipeline = 0
    rgb_resolution = ""
    depth_resolution = ""
    fps = 0
    serial_number = ""

    def __init__(self, name, rgb_resolution="fullHD", depth_resolution="HD", fps=30, serial_number = ""):

        self.camera_name = name
        self.rgb_resolution = rgb_resolution
        self.depth_resolution = depth_resolution
        self.fps = fps
        self.serial_number = serial_number

        print(f"{self.camera_name} initialization")

        if name == "zed":
            self.initialize_zed()
        elif name == "intel":
            self.initialize_intel()
        else:
            print("Camera DOES NOT exist. Choose one between (zed, intel)")
            exit(1)

        print(f"{self.camera_name} camera configured.\n")

    def __del__(self):
        if self.camera_name == "intel":
            self.pipeline.stop()
        elif self.camera_name == "zed":
            self.pipeline.close()
        print("Pipeline closed.")

    def initialize_zed(self):
        self.pipeline = sl.Camera()

        # set initial parameters
        init_params = sl.InitParameters()

        # set resolution
        if self.rgb_resolution == "2K":
            init_params.camera_resolution = sl.RESOLUTION.HD2K
            if self.fps > 15:
                self.fps = 15
        elif self.rgb_resolution == "HD":
            init_params.camera_resolution = sl.RESOLUTION.HD720
        else:
            init_params.camera_resolution = sl.RESOLUTION.HD1080

        init_params.camera_fps = self.fps

        # start the camera
        err = self.pipeline.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit()

        # get intrinsics
        zed_params = self.pipeline.get_camera_information().calibration_parameters
        self.intr = zed_params.left_cam

    def initialize_intel(self):

        # start camera
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.serial_number != "":
            config.enable_device(self.serial_number)

        # set resolutions
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, self.fps)  # max 1280x720 at 90 fps

        if self.rgb_resolution == "HD":

            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, self.fps)  # max 1920x1080 at 30 fps
        else:
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, self.fps)  # max 1920x1080 at 30 fps

        # Start streaming
        cfg = self.pipeline.start(config)

        profile = cfg.get_stream(rs.stream.color)
        intr = profile.as_video_stream_profile().get_intrinsics()
        self.intr = {'fx': intr.fx, 'fy': intr.fy, 'px': intr.ppx, 'py': intr.ppy}

    def get_intrinsics(self):
        return self.intr

    def get_name(self):
        return self.camera_name

    def get_fps(self):
        return self.fps

    def get_rgb(self):
        if self.camera_name == "intel":
            color_frame = None
            while not color_frame:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

        elif self.camera_name == "zed":
            color_frame = sl.Mat()
            runtime_parameters = sl.RuntimeParameters()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame, sl.VIEW.LEFT)

        color_frame = np.asanyarray(color_frame.get_data())
        return color_frame

    def get_depth(self):
        if self.camera_name == "intel":
            while not depth_frame:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()

        elif self.camera_name == "zed":
            depth_frame = sl.Mat()
            runtime_parameters = sl.RuntimeParameters()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)

        depth_frame = np.asanyarray(depth_frame.get_data())
        return depth_frame

    def get_frames(self):
        if self.camera_name == "intel":
            depth_frame_cam, color_frame_cam = None, None
            while not color_frame_cam or not depth_frame_cam:
                frames = self.pipeline.wait_for_frames()
                depth_frame_cam = frames.get_depth_frame()
                color_frame_cam = frames.get_color_frame()

        elif self.camera_name == "zed":
            color_frame_cam = sl.Mat()
            depth_frame_cam = sl.Mat()
            runtime_parameters = sl.RuntimeParameters()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame_cam, sl.VIEW.LEFT)
                self.pipeline.retrieve_measure(depth_frame_cam, sl.MEASURE.DEPTH)
        depth_frame = np.asanyarray(depth_frame_cam.get_data())
        color_frame = np.asanyarray(color_frame_cam.get_data())

        return color_frame, depth_frame

    def set_option(self, option, value):
        option_name = str(option)
        try:
            if self.camera_name == "intel":
                sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
                sensor.set_option(option, value)
                option_name = str(option).replace('option.', '').upper()
            elif self.camera_name == "zed":
                self.pipeline.set_camera_settings(option, value)
                option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
            print(f"Option {option_name} changed to value: {value}")

        except TypeError as ex:
            print(f"\033[0;33;40m Exception ({type(ex).__name__}): the option {option_name} has NOT been set.")

    def get_option(self, option):
        try:
            if self.camera_name == "intel":
                sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
                value = sensor.get_option(option)
                option_name = str(option).replace('option.', '').upper()
            elif self.camera_name == "zed":
                value = self.pipeline.get_camera_settings(option)
                option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
            print(f"Option {option_name} value: {value}")

        except TypeError as ex:
            print(f"\033[0;33;40m Exception ({type(ex).__name__}): the option {option_name} does NOT exist.")
