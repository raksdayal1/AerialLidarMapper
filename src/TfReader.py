#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi


def main():
    # Initialize the ros node
    rospy.init_node('ControlNode', anonymous=True)
    # create a tf listener
    listener = tf.TransformListener()

    # Set the rate
    rate = rospy.Rate(100)

    while True:

        # Listen to the TransformListener to get the odom to chassis transform
        try:
            (trans, rot) = listener.lookupTransform('map','lidar', rospy.Time(0))
        except:
            continue

        (roll, pitch, yaw) = euler_from_quaternion(rot)
        print("Roll = %f, Pitch = %f, Yaw = %f" % (roll*180/pi -pi/2.0, pitch*180/pi -pi/2.0, pi/2.0 + yaw*180/pi))

        rate.sleep()


if __name__ == '__main__':
    main()
