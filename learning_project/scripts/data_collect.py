googg#!/usr/bin/env python
# Software License Agreement (BSD License


import rospy
import math
import pickle
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


class DataCollector():

    def __init__(self):
        rospy.init_node('datacollector', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.collect_data)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.get_teleop)
        self.scans = []

    def collect_data(self, msg):
        self.scans.append((msg, self.last_vel))
        print "starting pickle " + str(len(self.scans))
        pickle.dump( self.scans, open( "save.p", "wb" ) )
        print "pickle complete"

    def get_teleop(self, msg):
        # Get velocity here and save it
        self.last_vel = msg

    def run(self):
        
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            move_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            self.pub.publish(move_msg)
            r.sleep()


if __name__ == '__main__':
    try:
        node = DataCollector()
        node.run()
    except rospy.ROSInterruptException: pass
