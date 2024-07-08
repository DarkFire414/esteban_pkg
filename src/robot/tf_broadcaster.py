#!/usr/bin/env python3

import sys
import rospy
import tf
from nav_msgs.msg import Odometry

class Tf_broadcaster:
    def __init__ (self):
        """
        Se subscribe a los mensajes de odometría y los convierte en 
        una transformación que utiliza ROS para ubicar al robot en 
        RVIZ.
        """
        self.x = 0
        self.y = 0
        self.theta = 0

        subs = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.br = tf.TransformBroadcaster()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

    def sendTf(self):
        self.br.sendTransform((self.x, self.y, 0), 
                                tf.transformations.quaternion_from_euler(0, 0, self.theta),
                                rospy.Time.now(), 
                                'base_link', 
                                'odom')

def main(args):
    rospy.init_node('tf_broadcaster_node')
    tfb = Tf_broadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        tfb.sendTf()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)