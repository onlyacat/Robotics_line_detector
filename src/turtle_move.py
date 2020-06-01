#!/usr/bin/env python
import sys
import numpy as np
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Vector3


def create_vel_msg(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    return Twist(
        linear=Vector3(lx, ly, lz),
        angular=Vector3(ax, ay, az)
    )


class Turtlebot:

    def __init__(self):
        rospy.init_node('thymio_controller')  # name of the node
        self.dir_sub = rospy.Subscriber("thymio10/direction", Int16, self.sub_callback)
        self.vel_pub = rospy.Publisher("thymio10/cmd_vel", Twist, queue_size=10)

    def sub_callback(self, data):
        direction = data.data
        # print(direction)
        if direction == -1:
            msg = create_vel_msg(lx=0.15, az=0.15)
            rospy.loginfo('Turning left')
        elif direction == 0:
            msg = create_vel_msg(lx=0.22)
            rospy.loginfo('Going straight')
        elif direction == 1:
            msg = create_vel_msg(lx=0.15, az=-0.15)
            rospy.loginfo('Turning right')
        else:
            msg = create_vel_msg(az=0.25)
            rospy.loginfo('Searching')

        self.vel_pub.publish(msg)

    def stop(self):
        msg = create_vel_msg(az=0.25)
        rospy.loginfo('Shutting down')
        self.vel_pub.publish(msg)


def main(args):
    tb = Turtlebot()
    try:
        rospy.spin()
    except Exception:
        tb.stop()


if __name__ == '__main__':
    main(sys.argv)
