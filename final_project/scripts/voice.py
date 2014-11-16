#!/usr/bin/env python
# Software License Agreement (BSD License)

# Voice commands to control the NEATO

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class VoiceCommander():

    def __init__(self):
        rospy.init_node('voicecommander', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.sub = rospy.Subscriber('scan', LaserScan, self.



    def run(self):

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':
    try:
        node = MonsterTracker()
        node.run()
    except rospy.ROSInterruptException: pass