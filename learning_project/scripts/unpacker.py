from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def unpack():
 
    with open('neato_data.p', 'rb') as f:
        data = pickle.load(f)
        laser_data = []
        twist_messages = []

        for i in range(len(data)):
            laserMsg = data[i][0]
            twist = data[i][1]

            laser_data.append(laserMsg)
            twist_messages.append([twist.linear.x,twist.angular.z])

        regr = linear_model.Ridge(alpha=.1)
        regr.fit(laser_data,twist_messages)

        print "here"
        print regr