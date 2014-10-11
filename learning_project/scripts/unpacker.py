import pickle
from sklearn import linear_model
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def unpack():
 
    with open('neato_data.p', 'rb') as f:
        data = pickle.load(f)
        laser_data = []
        twist_messages = []
        start_adding = False

        for i in range(len(data)):
            laserMsg = data[i][0]
            twist = data[i][1]

            if twist.linear.x > 0 or twist.angular.z > 0:
                start_adding = True
            if start_adding:
                laser_data.append(laserMsg)
                twist_messages.append([twist.linear.x,twist.angular.z])

        regr = linear_model.Ridge(alpha=.1)
        ridgeFit = regr.fit(laser_data,twist_messages)
        ridgePredict = regr.predict(laser_data)
        print regr.__dict__
        
        pickle.dump(regr, open('regr_data.p', 'wb'))
        # print twist_messages

        # print ridgePredict
        # print regr
        

unpack()
