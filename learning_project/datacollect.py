#!/usr/bin/env python
# Software License Agreement (BSD License


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import pickle

class LaserScanner():

    def __init__(self):
        rospy.init_node('laserscanner', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.collect_data)
        self.scans = []

    def collect_data(self, msg):
        self.scans.append(msg)
        print "starting pickle " + str(len(self.scans))
        pickle.dump( self.scans, open( "save.p", "wb" ) )
        print "pickle complete"

    def run(self):
        
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            move_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            self.pub.publish(move_msg)
            r.sleep()


if __name__ == '__main__':
    try:
        node = LaserScanner()
        node.run()
    except rospy.ROSInterruptException: pass
