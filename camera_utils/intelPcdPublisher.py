from cameras.IntelRealsense import IntelRealsense
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import select
import struct
import pdb
import sys

# function used to stop loop functions
def stop_loop(stop_entry):
    '''
    Used to quit an infinite loop with a char/string entry
    '''
    rlist = select.select([sys.stdin], [], [], 0.001)[0]
    if rlist and sys.stdin.readline().find(stop_entry) != -1:
        return True
    return False


if __name__ == "__main__":

    rospy.init_node("publish_intel_pcd")
    camera = IntelRealsense(camera_resolution=IntelRealsense.Resolution.LOW)

    point_pub = rospy.Publisher("intel_pcd", PointCloud2, queue_size=3)

    intr = camera.get_intrinsics()

    dims = camera.get_depth().shape

    header = Header()
    header.frame_id = "map"

    print("\nRunning\nTo quit the program press q and then Enter.")

    while not stop_loop('q'):

        rgb, depth = camera.get_frames()

        points = []

        for u in range(dims[0]):
            for v in range(dims[1]):

                z = depth[u, v] * 0.001
                x = ((u - intr["px"]) * z) / intr["fx"]
                y = ((v - intr["py"]) * z) / intr["fy"]

                color = rgb[u, v]

                r = int(color[0])
                g = int(color[1])
                b = int(color[2])
                a = 255

                color = struct.unpack('I', struct.pack('BBBB', r, g, b, a))[0]

                points.append([x, y, z, color])
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1)]


        pcd = point_cloud2.create_cloud(header, fields, points)
        pcd.header.stamp = rospy.Time.now()
        point_pub.publish(pcd)