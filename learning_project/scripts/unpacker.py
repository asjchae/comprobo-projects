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
         """I'm assuming that the laser scan stored in pickle is
         still a laserscan ros message, I may be wrong, If tis is not how you stored
         that data or pickle doesn't allow the object type to be saved this will probably break
         in which case email me """
         laserdata.append(laserMsg.ranges)
         twist_messages.append([twist.linear.x,twist.angular.z])#SHoudl be all the components of the twist message that matter

        """ after the for loop completes this can be fed into the function I sent earlier. It seems the
        function fit most wants two numpy arrays, I don't think it should matter that I'm not technically giving
        it numpy arrays, but if it doesn't like these inputs its pretty easy to convert from a list of lists
        to a numpy array. Goodluck!"