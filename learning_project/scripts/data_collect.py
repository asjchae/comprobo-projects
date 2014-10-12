#!/usr/bin/env python
# Software License Agreement (BSD License)


# Collects training data for supervised learning.
# Saves the laser scan reading and the corresponding linear/angular velocity as a tuple.
# After 50 readings, saves all the data in a pickle file.

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
        self.last_vel = Twist()

    def collect_data(self, msg):

        ff = [] # front front
        fr = [] # front right
        fl = [] # front left
        bb = [] # back back
        br = [] # back right
        bl = [] # back left

        laserscan = []        

        # Averaging the laser scan.
        # Front: 330-360, 0-30

        for i in range(30):
            if msg.ranges[329+i] > 0:
                ff.append(msg.ranges[329+i])
            if msg.ranges[30-i] > 0:
                ff.append(msg.ranges[30-i])
        if len(ff) > 0:
            laserscan.append(sum(ff)/float(len(ff)))
        else:
            laserscan.append(float(0))

        # Front left: 30-90

        for i in range(60):
            if msg.ranges[30+i] > 0:
                fl.append(msg.ranges[30+i])
        if len(fl) > 0:
            laserscan.append(sum(fl)/float(len(fl)))
        else:
            laserscan.append(float(0))

        # Back right: 90-150

        for i in range(60):
            if msg.ranges[90+i] > 0:
                br.append(msg.ranges[90+i])
        if len(br) > 0:
            laserscan.append(sum(br)/float(len(br)))
        else:
            laserscan.append(float(0))

        # Back: 150-210

        for i in range(60):
            if msg.ranges[150+i] > 0:
                bb.append(msg.ranges[150+i])
        if len(bb) > 0:
            laserscan.append(sum(bb)/float(len(bb)))
        else:
            laserscan.append(float(0))


        # Back left: 210-270

        for i in range(60):
            if msg.ranges[210+i] > 0:
                bl.append(msg.ranges[210+i])
        if len(bl) > 0:
            laserscan.append(sum(bl)/float(len(bl)))
        else:
            laserscan.append(float(0))

        # Front right: 270-330

        for i in range(60):
            if msg.ranges[270+i] > 0:
                fr.append(msg.ranges[270+i])
        if len(fr) > 0:
            laserscan.append(sum(fr)/float(len(fr)))
        else:
            laserscan.append(float(0))

        self.scans.append((laserscan, self.last_vel))


        # Saving to a pickle file
        if len(self.scans) == 50:
            print "starting pickle " + str(len(self.scans))
            pickle.dump(self.scans, open( "neato_data.p", "a" ) )
            print "pickle complete"

    def get_teleop(self, msg):
        # Get velocity here and save it
        self.last_vel = msg
        return self.last_vel

    def run(self):
        
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    try:
        node = DataCollector()
        node.run()
    except rospy.ROSInterruptException: pass
