""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from neato_node.msg import Accel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from visualization_messages.msgs import Marker
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy

###############################################################################
#Sending classes
###############################################################################
class SendSpeed(object):
    '''
    Class container that handles sending the speed to a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        '''Initializer will return instance from which you can call the important
        functions'''
        rospy.init_node('Get_Speed')   #initialize with roscore
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def send_speed(self,move_forward = None, turn_left = None):
        '''Sends a translational and rotational speed to the neato. Since the
        neato can only control one axis in both translational and rotational
        movement there are only 2 inputs
        move_forward:
            positive --> move forward
            negative --> move backward
        turn_left:
            positive --> turn left
            negative --> turn right'''
        if move_forward == None:
            move_forward = 0
        if turn_left == None:
            turn_left = 0
        my_point_stamped = Twist(linear=Vector3(move_forward,0,0), angular=Vector3(0,0,turn_left))
        self.publisher.publish(my_point_stamped)

    def low_rider(self):
        '''Ignore
        Joke function to make the neato bounce.
        Like so: https://www.youtube.com/watch?v=HEEHmiAR71Y'''
        t = 10
        r = rospy.Rate(5)
        for i in range(t):
            my_point_stamped = Twist(linear=Vector3(1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()
            my_point_stamped = Twist(linear=Vector3(-1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()

###############################################################################
#Receiving classes
###############################################################################

class ReceiveLidar(object):
    '''
    Class container that handles reading Lidar values from a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        # rospy.init_node('receive_lidar')
        '''Class currently only supports the range attribute of the Lidar
        message.'''
        rospy.Subscriber("/scan", LaserScan, self.process_range)
        self.ranges = None

    def process_range(self, m):
        self.ranges = m.ranges #set range

    def get_range(self):
        return self.ranges #get range for outside scripts

    def run(self):
        rospy.spin()

    def get_wall(self, threshold = .8, min_values = 10):
        '''This method will find likely directions towards a wall given lidar data
        returns: direction towards nearest wall
                 distance to that wall
        '''
        if self.ranges is None: #if Lidar data has not been received
            print('NO RANGES')
            return None, None
        ranges = np.array(self.ranges) #convert to numpy array
        no_zeros_index = np.where(ranges)[0] #find indices where we have data
        if not len(no_zeros_index): #make sure there is data
            print('NO REAL VALUES')
            return None, None
        no_zeros = ranges[no_zeros_index] #get the corresponding values for the indices
        least = no_zeros.argsort() #order the indices by closest values
        thresholds = []
        forbidden = set()
        number_checked = 0
        for least_val in least:
            '''In this loop we check the lowest values and determine likelyhood
            that they belong to a wall. After a point has been deemed unworthy,
            we don't consider the points immediately around it because they
            will not be different.'''
            if no_zeros_index[least_val] in forbidden:#too close to another point
                continue
            elif self.get_cost(least_val, no_zeros, no_zeros_index) < threshold:
                return no_zeros_index[least_val], no_zeros[least_val]
            else:
                for degree in range(no_zeros_index[least_val] - 5,no_zeros_index[least_val] + 5):
                    forbidden.add(degree)
                number_checked += 1

            if number_checked >= min_values: #set max on how many points we visit
                print('NOT GOOD ENOUGH')
                return None, None
        print('RAN OUT OF POINTS')
        return None, None

    def get_cost(self, index, values, indices):
        '''Determines the likelyhood that a given index is the closest point of
        a wall by comparing its surroundings' readings to d/cos(theta) aka their
        expected readings'''
        d = values[index]
        valid_range = 45
        required_points = 30
        total_diff = 0.0
        number_found = 0
        for i,v in enumerate(indices):
            diff = min([abs(indices[index] - v), abs(indices[index] - 360 + v)])
            if diff <= valid_range:
                supposed = d / math.cos(math.pi * diff / 180)
                total_diff += abs(supposed - values[i])/supposed
                number_found +=1
        if number_found < required_points:
            print('Not enough data')
            return 100 #random high number
        return total_diff/number_found



class ReceiveBump(object):
    '''
    Class container that handles reading Bump values from a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        rospy.init_node('receive_bump')
        rospy.Subscriber("/bump", Bump, self.process_bump)
        self.m = 0

    def process_bump(self, m):
        self.m  = m

    def run(self):
        rospy.spin()

class ReceiveAccel(object):
    '''
    Class container that handles reading acceleration values from a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        rospy.init_node('receive_bump')
        rospy.Subscriber("/accel", Accel, self.process_accel)
        self.m = 0

    def process_accel(self, m):
        self.m  = m

    def run(self):
        rospy.spin()

class ReceiveOdom(object):
    '''
    Class container that handles reading odometry values from a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        rospy.init_node('receive_odom')
        rospy.Subscriber("/odom", Odometry, self.process_odom)
        self.odom = None

    def process_odom(self, m):
        self.odom = m
        print(m)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # node = ReceiveLidar()
    # while not rospy.is_shutdown():
    #     print(node.get_wall())
    #     rospy.sleep(.3)
    node = ReceiveOdom()
    node.run()
