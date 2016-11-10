#!/usr/bin/env python

""" 
Demo of reading from and writing to  image topics. 

Author: Nathan Sprague
Version: 11/15
"""

import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

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

class RedNode(object):
    """ This node reads from the Kinect color stream and
    publishes an image topic with the most red pixel marked.
    Subscribes:
         /camera/rgb/image_color
    Publishes:
         red_marked_image
    """

    def __init__(self):
        """ Construct the red-pixel finder node."""
        rospy.init_node('red_node')
        self.image_pub = rospy.Publisher('red_marked_image', Image)
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, img):
        """ Handle image callbacks. """

        # Convert the image message to a cv image object
        cv_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")

        # Do the image processing
        red_pos = find_reddest_pixel_fast(cv_img)
        cv2.circle(cv_img, red_pos, 5, (0, 255, 0), -1)

        # Convert the modified image back to a message.
        img_msg_out = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.image_pub.publish(img_msg_out)


if __name__ == "__main__":
    r = RedNode()
