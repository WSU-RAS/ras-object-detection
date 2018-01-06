#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def panTilt():
    pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=2)
    tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=2)
    rospy.init_node('pan_tilt', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0

    while not rospy.is_shutdown():
        pan.publish(0)
        tilt.publish(0.25)
        rate.sleep()

        # Run only briefly
        i += 1
        if i > 10:
            break

if __name__ == '__main__':
    try:
        panTilt()
    except rospy.ROSInterruptException:
        pass
