#!/usr/bin/env python3
import rospy
import copy
from sensor_msgs.msg import PointCloud2
# TODO also support the TensorFlow bounding boxes
from darknet_ros_msgs.msg import BoundingBoxes
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs

lastSeen = None

def getIndex(x, y, width):
    """
    Get point array index for this particular (x,y) pixel
    """
    return int(x + y * width)

def callback_point(data):
    global lastSeen

    # Use last bounding boxes we've received (points are at much higher rate than boxes)
    if lastSeen:
        points = []

        """
        # Get all points
        #
        # Note: really we *do* want to skip NaNs, but if we do, then we can't find
        # the point that corresponds to the x/y positions in the image. Thus, maybe
        # search bounding box for non-NaNs?
        for p in pc2.read_points(data, field_names=("x","y","z"), skip_nans=False):
            points.append((p[0],p[1],p[2]))

        # Find 3D points of each bounding box
        for b in lastSeen:
            # Find center (TODO bad idea... just look at the 3D point clouds)
            center_x = (b.xmin + b.xmax)/2
            center_y = (b.ymin + b.ymax)/2
            # Find which 3D point this center corresponds to (and round)
            i = int(center_x + center_y * data.width)

            print("%s x %f y %f z %f" %(b.Class,points[i][0],points[i][1],points[i][2]))
        """
        # Convert point cloud to map reference frame
        try:
            # target frame, source frame, time
            # TODO replace "base_link" with "map" once we have a map
            transform = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            print("Error with tf2")

        cloud = tf2_sensor_msgs.do_transform_cloud(data, transform)

        # Get list of points we want -- note: (u,v) is in image, (x,y,z) is in 3D space
        uvs = []

        for b in lastSeen:
            # Find center (TODO bad idea... just look at the 3D point clouds)
            center_x = int((b.xmin + b.xmax)/2)
            center_y = int((b.ymin + b.ymax)/2)

            uvs.append((center_x, center_y))

        # Get 3D points
        points = []

        # TODO try skip_nans=True
        for p in pc2.read_points(cloud, field_names=("x","y","z"), uvs=uvs, skip_nans=False):
            points.append((p[0],p[1],p[2]))

        # TODO is there a locking issue here...?

        # Look at 3D points for each bounding box
        for i, b in enumerate(lastSeen):
            print("%s x %f y %f z %f" %(b.Class,points[i][0],points[i][1],points[i][2]))

            # TODO what is this (x,y,z) relative to? base_link, camera_link,
            # odom?  how do we convert it then to be w.r.t. the map, which is
            # what we really care about

def callback_box(data):
    # TODO maybe process here since there's no point in giving new positions
    # with old bounding box data unless we're doing some sort of tracking on
    # the point clouds or camera data
    global lastSeen

    for b in data.boundingBoxes:
        print("Box: xmin : %d ymin : %d xmax : %d ymax : %d prob : %d class : %s" % (
                b.xmin, b.ymin, b.xmax, b.ymax, b.probability, b.Class
            ))

    lastSeen = copy.copy(data.boundingBoxes)

def listener():
    rospy.init_node('findObjects', anonymous=True)

    # Listen to point cloud
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_point)

    # Listen to bounding boxes
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_box)

    # Listen to reference frames, for the coordinate transformations
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.spin()

if __name__ == '__main__':
    listener()
