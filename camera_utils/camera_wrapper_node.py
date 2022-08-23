from camera_init import IntelRealsense
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class IntelRosWrapper(IntelRealsense):

    Resolution = IntelRealsense.Resolution

    bridge = CvBridge()


    def __init__(self, rgb_topic="rgb_image_raw", 
                 depth_topic="depth_image_raw", 
                 camera_info_topic="camera_info", rgb_resolution=Resolution.HD,
                 depth_resolution=Resolution.HD,
                 fps=30, serial_number="", depth_in_meters=False):

        self.rgb_publisher = rospy.Publisher(rgb_topic, Image, queue_size=5)
        self.depth_publisher = rospy.Publisher(depth_topic, Image, queue_size=5)
        self.camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)

        IntelRealsense.__init__(self, rgb_resolution, depth_resolution, fps, serial_number, depth_in_meters)
        rospy.init_node("intel_ros_wrapper")


    def __del__(self):
        IntelRealsense.__del__(self)


    def get_rgb(self):
        '''
        :return: An rgb image as numpy array
        '''
        color_frame = IntelRealsense.get_rgb(self)

        self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "bgr8"))


    def get_depth(self):
        '''
        :return: A depth image (1 channel) as numpy array
        '''
        depth_frame = IntelRealsense.get_depth(self)

        self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))


    def get_frames(self):
        '''
        :return: rgb, depth images as numpy arrays
        '''
        color_frame, depth_frame = IntelRealsense.get_frames(self)

        self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "bgr8"))
        self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))


    def get_aligned_frames(self):
        '''
        :return: rgb, depth images aligned with post-processing as numpy arrays
        '''
        color_frame, depth_frame = IntelRealsense.get_aligned_frames(self)

        self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "bgr8"))
        self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))


    def get_intrinsics(self):

        intr =  IntelRealsense.get_intrinsics(self)

        camera_info = CameraInfo()

        camera_info.K[0] = intr['fx']
        camera_info.K[4] = intr['fy']
        camera_info.K[2] = intr['px']
        camera_info.K[5] = intr['py']

        self.camera_info_publisher.publish(camera_info)




