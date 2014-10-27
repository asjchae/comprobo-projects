#!/usr/bin/env python
# Software License Agreement (BSD License)

# Collects laser scan from the robot
# Uses regression fit to adjust the velocity of the robot based on its laser scan

import pickle
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class RobotMover():

    def __init__(self):
        rospy.init_node('robotmover', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.run_regression)
        self.vel = 0.0
        self.turning = 0.0
        with open('regr_data.p', 'rb') as f:
            self.regr = pickle.load(f)

    
    def run_regression(self, msg):

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

        rdata = self.regr.coef_[0]
        intercept = self.regr.intercept_[0]
        ndata = laserscan
        moving = [0.0,0.0,0.0,0.0,0.0,0.0]


        # using regression data to control the neato
        for i in range(len(rdata)):
            print intercept
            moving[i] = (rdata[i]*(intercept+ndata[i]))

            msg = Twist(Vector3(float(moving[0]),0.0,0.0),Vector3(0.0,0.0,0.0))
            self.pub.publish(msg)

    def run(self):
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            # msg = Twist(Vector3(vel,0.0,0.0),Vector3(0.0,0.0,turning))
            # pub.publish(msg)
            r.sleep()


if __name__ == '__main__':
    try:
        node = RobotMover()
        node.run()
    except rospy.ROSInterruptException: pass