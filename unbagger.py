#!/usr/bin/python
"""
Modified for getting RGB and D images from our robot

Look at:
    https://github.com/PalouseRobosub/vision_dev
"""
import rosbag
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
import os
import sys

class ImageCreator():
    def __init__(self, save_dir, filename):

        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        self.bridge = CvBridge()

        # Open bag file.
        with rosbag.Bag(filename, 'r') as bag:
            for topic, msg, t in bag.read_messages(connection_filter=self.filter_std_image):
                self.saveImg(topic, msg)

            for topic, msg, t in bag.read_messages(connection_filter=self.filter_wfov_image):
                self.saveImg(topic, msg.image)

    def saveImg(self, topic, msg):
        if rospy.is_shutdown():
            print("ROS Shutdown while reading WFOV images")
            sys.exit(0)

        try:
            prefix = "depth" if "depth" in topic else "rgb" if "rgb" in topic else "other"

            # Hack to get around "[16UC1] ... The conversion does not make sense"
            # https://gist.github.com/awesomebytes/30bf7eae3a90754f82502accd02cbb12
            if msg.encoding == "16UC1": # Depth
                msg.encoding = "mono16"
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            else: # RGB
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            timestr = "%.6f" % msg.header.stamp.to_sec()
            image_name = str(save_dir) + prefix + "_" + timestr + ".png"
            print ("saving image:" + image_name)
            cv.imwrite(image_name, cv_image)
        except CvBridgeError, e:
            print (e)

    # Allows only sensor_msgs/Image messages
    def filter_std_image(self, topic, datatype, md5sum, msg_def, header):
        return (True if "sensor_msgs/Image" in datatype else False)

    # Allows only WFOVImage messages
    def filter_wfov_image(self, topic, datatype, md5sum, msg_def, header):
        return (True if "wfov_camera_msgs/WFOVImage" in datatype else False)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it
    rospy.init_node('unbagger_script')

    if len(sys.argv) < 2:
        raise RuntimeError('python unbagger.py <save_dir>/ <filename>')
    else:
        save_dir = sys.argv[1]
        filename = sys.argv[2]
        rospy.loginfo("Bag filename = %s", filename)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        image_creator = ImageCreator(save_dir, filename)
    except rospy.ROSInterruptException, e:
        pass
