from camera_init import IntelRealsense
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class IntelRosWrapper(IntelRealsense):

    Resolution = IntelRealsense.Resolution

    bridge = CvBridge()

    rgb_publisher = rospy.Publisher("rgb_image_raw", Image, queue_size=5)
    depth_publisher = rospy.Publisher("depth_image_raw", Image, queue_size=5)

    def __init__(self, rgb_resolution=Resolution.HD,
                 depth_resolution=Resolution.HD,
                 fps=30, serial_number=""):

        IntelRealsense.__init__(self, rgb_resolution, depth_resolution, fps, serial_number)
        rospy.init_node("intel_ros_wrapper")

    def __del__(self):
        IntelRealsense.__del__(self)

    def get_rgb(self, ros_publish=False):
        '''
        :return: An rgb image as numpy array
        '''
        color_frame = IntelRealsense.get_rgb(self)

        if ros_publish:
            self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "rgb8"))

        return color_frame

    def get_depth(self, ros_publish=False):
        '''
        :return: A depth image (1 channel) as numpy array
        '''
        depth_frame = IntelRealsense.get_depth(self)

        if ros_publish:
            self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))

        return depth_frame

    def get_frames(self, ros_publish=False):
        '''
        :return: rgb, depth images as numpy arrays
        '''
        color_frame, depth_frame = IntelRealsense.get_frames(self)

        if ros_publish:
            self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "rgb8"))
            self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))

        return color_frame, depth_frame

    def get_aligned_frames(self, ros_publish=False):
        '''
        :return: rgb, depth images aligned with post-processing as numpy arrays
        '''
        color_frame, depth_frame = IntelRealsense.get_aligned_frames(self)

        if ros_publish:
            self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(color_frame, "rgb8"))
            self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth_frame, "mono16"))

        return color_frame, depth_frame



