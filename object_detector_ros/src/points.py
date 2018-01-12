#!/usr/bin/env python2
import rospy
import copy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# TODO also support the TensorFlow bounding boxes
from darknet_ros_msgs.msg import BoundingBoxes

# Coordinate transformation
import tf2_ros
import PyKDL
from tf2_sensor_msgs.tf2_sensor_msgs import transform_to_kdl

# We could rewrite into a class or use this global variable
lastSeen = None
lastSeenCount = 0
lastSeenTimeout = 30

def callback_point(cloud, args):
    """
    Handle when we get a new point cloud

    We will find the locations of objects in this rather than in callback_box
    since as the robot is moving, tipping, etc. we want to keep updating where
    the object is. This does assume that the object does not move between
    updated bounding boxes, but we can get 20 fps with SSD MobileNet. It likely
    only matters if using YOLO which right now is ~2.5 fps.
    """
    global lastSeen
    global lastSeenCount
    global lastSeenTimeout

    # Get passed-in arguments
    target, source, tf_buffer = args

    # If we haven't seen a bounding box update for this many point clouds
    # frames, then set lastSeen back to None. This is so we don't keep using
    # old data when the objects go out of frame.
    lastSeenCount += 1

    if lastSeenCount > lastSeenTimeout:
        lastSeen = None

    # Use last bounding boxes we've received (points are at much higher rate than boxes)
    if lastSeen:
        points = []

        # Convert point cloud to map reference frame
        try:
            # Arguments: target frame, source frame, time
            transform = tf_buffer.lookup_transform(target, source,
                    rospy.Time())

            # See tf2_sensor_msgs.py
            #
            # We will do it per-point so we don't have to transform the entire
            # point cloud, which is very slow.
            transform_kdl = transform_to_kdl(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            print("Error with tf2")

        for b in lastSeen:
            # Get list of points we want, all of them in the bounding box
            #
            # Note: (u,v) is in image, (x,y,z) is in 3D space
            uvs = []

            for x in range(b.xmin, b.xmax+1):
                for y in range(b.ymin, b.ymax+1):
                    uvs.append((x,y))

            # Get the points from the point cloud, but ignore NaNs
            points = []

            for p in pc2.read_points(cloud, field_names=("x","y","z"), uvs=uvs, skip_nans=True):
                points.append((p[0],p[1],p[2]))

            if len(points) > 0:
                # Average point locations
                mean = np.array(points).mean(axis=0)

                # Coordinate transformation
                p = transform_kdl*PyKDL.Vector(mean[0],mean[1],mean[2])

                # Debugging
                print("%s x %f y %f z %f" %(b.Class,p[0],p[1],p[2]))
            else:
                print("Error: all points are NaN in bounding box")

def callback_box(data):
    """
    Handle when we get a new bounding box

    All we do is save it so we'll be able to process it next time we get a
    point cloud, via callback_point
    """
    global lastSeen
    global lastSeenCount

    # We just received a frame, so reset this count
    lastSeenCount = 0

    # Debugging
    for b in data.boundingBoxes:
        print("Box: xmin : %d ymin : %d xmax : %d ymax : %d prob : %d class : %s" % (
                b.xmin, b.ymin, b.xmax, b.ymax, b.probability, b.Class
            ))

    # Copy so we can use in the callback_point function
    lastSeen = copy.copy(data.boundingBoxes)

def waitTillTransform(tf_buffer, target, source):
    """
    Block until the desired transform from target to source is available

    See: http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28Python%29
    """
    found = False
    transform = None

    while not rospy.is_shutdown() and transform == None:
        try:
            transform = tf_buffer.lookup_transform(target, source,
                    rospy.Time(), rospy.Duration(1.0))
            found = True
        except:
            continue

    return found

def listener():
    rospy.init_node('findObjects', anonymous=True)

    # Params
    target = rospy.get_param("~target", "base_link") # TODO change default to map
    source = rospy.get_param("~source", "camera_depth_optical_frame")

    # Listen to reference frames, for the coordinate transformations
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait till we get the map
    if waitTillTransform(tf_buffer, target, source):
        # Listen to point cloud
        rospy.Subscriber("/camera/depth_registered/points", PointCloud2,
                callback_point, (target, source, tf_buffer))

        # Listen to bounding boxes
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes,
                callback_box)

        rospy.spin()
    else:
        print("Error: failed to get transform from tf2")

if __name__ == '__main__':
    listener()
