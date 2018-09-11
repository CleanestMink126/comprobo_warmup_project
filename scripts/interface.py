""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from neato_node.msg import Accel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy

###############################################################################
#Sending classes
###############################################################################

class MessageNode(object):
    """This node publishes a message at 2 Hz"""
    def __init__(self):
        rospy.init_node('test_message')   #initialize with roscore
        self.publisher = rospy.Publisher('/my_point', PointStamped, queue_size=10)

    def run(self):
        r = rospy.Rate(2)  #Publishing at 2 Hz
        while not rospy.is_shutdown():
            my_point_stamped = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(1.0,2.0,0.0))
            publisher.publish(my_point_stamped)
            r.sleep()

class SendSpeed(object):
    def __init__(self):
        rospy.init_node('Get_Speed')   #initialize with roscore
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def send_speed(self,move_forward = None, turn_left = None):
        if move_forward == None:
            move_forward = 0
        if turn_left == None:
            turn_left = 0
        my_point_stamped = Twist(linear=Vector3(move_forward,0,0), angular=Vector3(0,0,turn_left))
        self.publisher.publish(my_point_stamped)

    def low_rider(self):
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
    def __init__(self):
        # rospy.init_node('receive_lidar')
        rospy.Subscriber("/scan", LaserScan, self.process_range)
        self.ranges = None

    def process_range(self, m):
        self.ranges = m.ranges

    def get_range(self):
        return self.ranges

    def run(self):
        rospy.spin()

    def get_wall(self, threshold = .8):
        if self.ranges is None:
            print('NO RANGES')
            return None, None
        values = np.array(self.ranges)
        # print(values)
        no_zeros_index = np.where(values)[0]
        # print(no_zeros_index)
        if not len(no_zeros_index):
            print('NO REAL VALUES')
            return None, None
        no_zeros = values[no_zeros_index]
        least = no_zeros.argsort()[0]
        thresholds = []
        if self.get_cost(least, no_zeros, no_zeros_index) < threshold:
            return no_zeros_index[least], no_zeros[least]
        else:
            print('NOT GOOD ENOUGH')
            return None, None

    def get_cost(self, index, values, indices):
        # print(index)
        d = values[index]
        exploration = 45
        total_diff = 0.0
        number_found = 0
        for i,v in enumerate(indices):
            diff = min([abs(indices[index] - v), abs(indices[index] - 360 + v)])
            if diff <= 45:
                supposed = d / math.cos(math.pi * diff / 180)
                total_diff += abs(supposed - values[i])/supposed
                number_found +=1
        if number_found < 30:
            print('Not enough data')
            return 1
        return total_diff/number_found



class ReceiveBump(object):
    def __init__(self):
        rospy.init_node('receive_bump')
        rospy.Subscriber("/bump", Bump, process_bump)

    def process_bump(self, m):
        return m

    def run(self):
        rospy.spin()

class ReceiveAccel(object):
    def __init__(self):
        rospy.init_node('receive_bump')
        rospy.Subscriber("/accel", Accel, process_accel)

    def process_accel(self, m):
        return m

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveLidar()
    while not rospy.is_shutdown():
        print(node.get_wall())
        rospy.sleep(.3)
