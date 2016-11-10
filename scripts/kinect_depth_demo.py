#!/usr/bin/env python

""" 
Demo of reading from image topic, reading correspoding point cloud
information and using markers to show output in rviz.

Author: Nathan Sprague
        Corey Brundage, Jasmyn Kelly, Nicole Maguire
Version: 11/15
"""
import rospy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
import numpy as np
from message_filters import ApproximateTimeSynchronizer
import message_filters



def make_sphere_marker(x, y, z, scale, frame_id, stamp):
    """ Create a red sphere marker message at the indicated position. """
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = 'spheres'
    m.id = 0
    m.type = m.SPHERE
    m.action = m.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.scale.x = m.scale.y = m.scale.z =  scale    
    m.color.r = 1.0
    m.color.a = 1.0
    return m

def find_reddest_pixel_fast(img):
    """ Return the pixel location of the reddest pixel in the image.

       Redness is defined as: redness = (r - g) + (r - b)

       Arguments:
            img - height x width x 3 numpy array of uint8 values.

       Returns:
            A tuple (x,y) containg the position of the reddest pixel.
    """

    cast = np.array(img, dtype='int32')

    cur_red = (cast[:, :, 2] - cast[:, :, 1]) + (cast[:, :, 2] - cast[:, :, 0])

    most_red = cv2.minMaxLoc(cur_red)[3]

    return most_red

class RedDepthNode(object):
    """ This node reads from the Kinect color stream and
    publishes an image topic with the most red pixel marked.
    Subscribes:
         /camera/rgb/image_color
    Publishes:
         red_marked_image
    """

    def __init__(self):
        """ Construct the red-pixel finder node."""
        rospy.init_node('red_depth_node')
        self.image_pub = rospy.Publisher('red_marked_image', Image,
                                         queue_size=10)
        self.marker_pub = rospy.Publisher('red_marker', Marker,
                                          queue_size=10)
        self.cv_bridge = CvBridge()

        # Unfortunately the depth data and image data from the kinect aren't
        # perfectly time-synchronized.  The code below handles this issue.
        img_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
        cloud_sub = message_filters.Subscriber(\
                            '/camera/depth_registered/points',
                            PointCloud2)
        self.kinect_synch = ApproximateTimeSynchronizer([img_sub, cloud_sub],
                                                        queue_size=10,
                                                        slop=.02)

        self.kinect_synch.registerCallback(self.image_points_callback)

        rospy.spin()

    def image_points_callback(self, img, cloud):
        """ Handle image/point_cloud callbacks. """

        # Convert the image message to a cv image object
        cv_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")

        # Do the image processing
        red_pos = find_reddest_pixel_fast(cv_img)
        cv2.circle(cv_img, red_pos, 5, (0, 255, 0), -1)

        # Extract just the point we want from the point cloud message
        # as an iterable of (x,y,z) tuples
        points = point_cloud2.read_points(cloud, skip_nans=False,
                                          uvs=[red_pos])

        # Iterate through the returned points (really just one)
        for p in points:
            if not np.isnan(p[0]):
                # Publish a marker at the point.
                marker = make_sphere_marker(p[0], p[1], p[2], .05,
                                            cloud.header.frame_id,
                                            cloud.header.stamp)
                self.marker_pub.publish(marker)

        # Convert the modified image back to a message.
        img_msg_out = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.image_pub.publish(img_msg_out)



if __name__ == "__main__":
    r = RedDepthNode()
